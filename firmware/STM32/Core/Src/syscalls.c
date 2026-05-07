/* Newlib-nano stubs so linking succeeds and printf can route to UART via
 * __io_putchar() defined in main.c.
 */

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <unistd.h>

extern int __io_putchar(int ch);

int _write(int file, char *ptr, int len) {
    (void)file;
    for (int i = 0; i < len; ++i) __io_putchar((unsigned char)ptr[i]);
    return len;
}

int _read(int file, char *ptr, int len)  { (void)file; (void)ptr; (void)len; return 0; }
int _close(int file)                     { (void)file; return -1; }
int _fstat(int file, struct stat *st)    { (void)file; st->st_mode = S_IFCHR; return 0; }
int _isatty(int file)                    { (void)file; return 1; }
int _lseek(int file, int ptr, int dir)   { (void)file; (void)ptr; (void)dir; return 0; }
int _getpid(void)                        { return 1; }
int _kill(int pid, int sig)              { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int status)                   { (void)status; while (1) {} }
