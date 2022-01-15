
#ifndef BIBLIOTECAS_UART_UART_H_
#define BIBLIOTECAS_UART_UART_H_


#include "frames.h"
#include "util.h"

#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

#define UART_SUCCESS	(0)
#define UART_DEVERROR	(1)
#define UART_TIMEOUT	(2)

#define UART_TIMEOUT_FOREVER (0xFFFFFFFFU)

typedef int8_t UART_status;


UART_status serial_receive( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout  );

UART_status serial_send( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout );


#endif /* BIBLIOTECAS_UART_UART_H_ */
