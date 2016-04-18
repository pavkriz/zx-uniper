#ifndef __UART_H_
#define __UART_H_

/* Use UART0 for LPC-Xpresso boards */
#define LPC_UARTX       LPC_USART0
#define UARTx_IRQn      USART0_IRQn
#define UARTx_IRQHandler UART0_IRQHandler

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 128	/* Send */
#define UART_RRB_SIZE 32	/* Receive */


void UART_Init();

void UART_printf(const char *format, ...);

#endif
