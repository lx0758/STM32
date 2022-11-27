#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/times.h>
#include "retarget.h"

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

UART_HandleTypeDef *gReTargetHuart;

void RETARGET_Init(UART_HandleTypeDef *huart)
{
  gReTargetHuart = huart;

  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  // setvbuf(stdout, NULL, _IONBF, 0);
}

__attribute__((unused)) int _isatty(int fd)
{
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 1;

  errno = EBADF;
  return 0;
}

__attribute__((unused)) int _write(int fd, char *ptr, int len)
{
  HAL_StatusTypeDef hstatus;

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
  {
    hstatus = HAL_UART_Transmit(gReTargetHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

__attribute__((unused)) int _close(int fd)
{
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}

__attribute__((unused)) int _lseek(int fd, int ptr, int dir)
{
  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

__attribute__((unused)) int _read(int fd, char *ptr, int len)
{
  HAL_StatusTypeDef hstatus;

  if (fd == STDIN_FILENO)
  {
    hstatus = HAL_UART_Receive(gReTargetHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

__attribute__((unused)) int _fstat(int fd, struct stat *st)
{
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
  {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)