
#include "main.h"

#if defined ( __CC_ARM )

/* ARMCC, MDK-ARM compiler*/
#include <stdio.h>

#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

FILE __stdout;
/**
 * @brief define _sys_exit() to avoid using semihosting mode
 * @param void
 * @return  void
 */
void _sys_exit(int x)
{
    x = x;
}

int fputc(int ch, FILE *f)
{
  #ifdef _REDIRECTION_UART_PRINTF
    HAL_UART_Transmit(&_REDIRECTION_UART_PRINTF, (uint8_t *)&ch, 1, 0xFFFF);
  #else
    ITM_SendChar(ch);
  #endif

  return(ch);
}

#elif defined ( __ICCARM__ )

/* IAR C compiler */

int fputc(int ch, FILE *f)
{
  #ifdef _REDIRECTION_UART_PRINTF
    HAL_UART_Transmit(&_REDIRECTION_UART_PRINTF, (uint8_t *)&ch, 1, 0xFFFF);
  #else
    ITM_SendChar(ch);
  #endif

  return(ch);
}

#elif defined ( __GNUC__ )

/* GNU GCC */

int _write(int fd, char * ptr, int len)
{
  #ifdef _REDIRECTION_UART_PRINTF
  HAL_UART_Transmit(&_REDIRECTION_UART_PRINTF, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  #else
  for (int i = 0; i < len; i++) {
	  ITM_SendChar(*(ptr+i));
  }
  #endif
  return len;
}

#endif
