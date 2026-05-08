/**
 * platform_compat.h — Pico-SDK shim for the STM32 port
 *
 * The DSP-pipeline source files imported from firmware/DSPi/ reach for
 * a handful of Pico SDK barrier / IRQ helpers. This header provides
 * minimal CMSIS-equivalent inline definitions so those files compile
 * unmodified on the STM32H723 (Cortex-M7) target.
 */

#ifndef DSPI_STM32_PLATFORM_COMPAT_H
#define DSPI_STM32_PLATFORM_COMPAT_H

#include <stdint.h>
#include "stm32h7xx.h"   /* CMSIS: __DMB, __disable_irq, etc. */

/* hardware/sync.h — Pico SDK barrier */
static inline void __dmb(void) { __DMB(); }

/* save_and_disable_interrupts() / restore_interrupts(flags)
 *
 * Pico SDK pattern: snapshot PRIMASK, disable IRQs, return the
 * snapshot; later restore from snapshot. Cortex-M7 PRIMASK is the
 * one bit that gates all interrupts (NMI/HardFault excepted). */
static inline uint32_t save_and_disable_interrupts(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void restore_interrupts(uint32_t flags) {
    __set_PRIMASK(flags);
}

#endif
