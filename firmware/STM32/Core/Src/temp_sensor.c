/**
 * temp_sensor.c — STM32H7 internal sensors on ADC3.
 *
 *   IN18 = TEMPSENSOR  → centi-degrees C
 *   IN19 = VREFINT     → derive Vdd in mV from factory VREFINT_CAL
 *
 * Both are slow-changing signals; Console polls them roughly once a
 * second via REQ_GET_STATUS wValue=14 / 16. We share one ADC3 init and
 * just reconfigure the channel right before each conversion.
 *
 * Factory calibration (system memory, mirrored across H7 variants):
 *   TS_CAL1     @ 0x1FF1E820 — raw at 30°C,  VREF=3.3V
 *   TS_CAL2     @ 0x1FF1E840 — raw at 110°C, VREF=3.3V
 *   VREFINT_CAL @ 0x1FF1E860 — raw VREFINT at 3.3V Vdda
 *
 * Vdd (mV) = 3300 × VREFINT_CAL / VREFINT_RAW
 * T  (°C)  = 30 + (raw - CAL1) × (110 - 30) / (CAL2 - CAL1)
 */

#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Factory-calibrated raw ADC values (16-bit) burned at production. */
#define TS_CAL1_ADDR     ((uint16_t *)0x1FF1E820UL)  /* 30°C  */
#define TS_CAL2_ADDR     ((uint16_t *)0x1FF1E840UL)  /* 110°C */
#define VREFINT_CAL_ADDR ((uint16_t *)0x1FF1E860UL)  /* VREFINT @ 3.3V Vdda */
#define VDDA_CAL_MV      3300
#define TS_CAL_TEMP_LO   30
#define TS_CAL_TEMP_HI   110

static ADC_HandleTypeDef hadc3;
static int adc_inited = 0;

void TempSensor_Init(void) {
    if (adc_inited) return;

    __HAL_RCC_ADC3_CLK_ENABLE();

    hadc3.Instance                       = ADC3;
    hadc3.Init.ClockPrescaler            = ADC_CLOCK_ASYNC_DIV4;
    hadc3.Init.Resolution                = ADC_RESOLUTION_16B;
    hadc3.Init.ScanConvMode              = ADC_SCAN_DISABLE;
    hadc3.Init.EOCSelection              = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait          = DISABLE;
    hadc3.Init.ContinuousConvMode        = DISABLE;
    hadc3.Init.NbrOfConversion           = 1;
    hadc3.Init.DiscontinuousConvMode     = DISABLE;
    hadc3.Init.ExternalTrigConv          = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge      = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ConversionDataManagement  = ADC_CONVERSIONDATA_DR;
    hadc3.Init.Overrun                   = ADC_OVR_DATA_OVERWRITTEN;
    hadc3.Init.LeftBitShift              = ADC_LEFTBITSHIFT_NONE;
    hadc3.Init.OversamplingMode          = DISABLE;

    if (HAL_ADC_Init(&hadc3) != HAL_OK) return;

    /* Calibrate (single-ended). Mandatory on H7 — without it the reading
     * is offset by hundreds of LSBs. */
    if (HAL_ADCEx_Calibration_Start(&hadc3,
                                     ADC_CALIB_OFFSET,
                                     ADC_SINGLE_ENDED) != HAL_OK) return;

    adc_inited = 1;
}

/* Switch ADC3 to the requested internal channel and run one conversion.
 * Returns the raw 16-bit reading or 0 on failure. */
static uint32_t adc3_read_channel(uint32_t hal_chan) {
    if (!adc_inited) {
        TempSensor_Init();
        if (!adc_inited) return 0;
    }
    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = hal_chan;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    ch.SingleDiff   = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc3, &ch) != HAL_OK) return 0;

    if (HAL_ADC_Start(&hadc3) != HAL_OK) return 0;
    if (HAL_ADC_PollForConversion(&hadc3, 5) != HAL_OK) {
        HAL_ADC_Stop(&hadc3);
        return 0;
    }
    uint32_t raw = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
    return raw;
}

int16_t read_temperature_cdeg(void) {
    uint32_t raw16 = adc3_read_channel(ADC_CHANNEL_TEMPSENSOR);
    if (raw16 == 0) return 0;

    int32_t cal1 = (int32_t)(*TS_CAL1_ADDR);
    int32_t cal2 = (int32_t)(*TS_CAL2_ADDR);
    int32_t denom = cal2 - cal1;
    if (denom == 0) return 0;

    /* Compute in centi-degrees directly to avoid float in the hot path:
     *   T_cdeg = lo·100 + (raw - cal1) × (hi - lo) × 100 / (cal2 - cal1) */
    int32_t span_cdeg = (TS_CAL_TEMP_HI - TS_CAL_TEMP_LO) * 100;  // 8000
    int32_t base_cdeg = TS_CAL_TEMP_LO * 100;                     // 3000
    int32_t numer = ((int32_t)raw16 - cal1) * span_cdeg;
    int32_t cdeg = base_cdeg + numer / denom;

    /* Sanity clamp — anything past ±150 °C is a bad reading. */
    if (cdeg >  15000) cdeg =  15000;
    if (cdeg < -5000)  cdeg = -5000;
    return (int16_t)cdeg;
}

/* Read VREFINT and back-compute Vdda in mV.
 *   Vdda = VDDA_CAL × VREFINT_CAL / VREFINT_RAW
 * Returns 0 on conversion failure (Console will display 0 mV). */
uint32_t read_vdda_mv(void) {
    uint32_t raw = adc3_read_channel(ADC_CHANNEL_VREFINT);
    if (raw == 0) return 0;
    uint32_t cal = *VREFINT_CAL_ADDR;
    if (cal == 0) return 0;
    /* (3300 × cal) max ≈ 3300 × 65535 ≈ 2.16e8 — fits uint32_t. */
    return (VDDA_CAL_MV * cal) / raw;
}
