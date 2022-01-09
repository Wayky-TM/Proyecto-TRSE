/*
 * UART.h
 *
 *  Created on: Jan 3, 2022
 *      Author: Álvaro
 */

#ifndef _UART_H_
#define _UART_H_


#include <stdio.h>
#include <stdlib.h>
#include <libserialport.h>
#include "frames.h"
#include "util.h"

#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

#define UART_DEBUG  (1)

#define UART_SUCCESS	(0)
#define UART_DEVERROR	(1)
#define UART_TIMEOUT	(2)

#define UART_TIMEOUT_FOREVER (0xFFFFFFFF)

typedef int8_t UART_status;

UART_status serial_init( const char * term_path );

UART_status serial_receive( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout  );

UART_status serial_send( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout );


#endif /* BIBLIOTECAS_UART_UART_H_ */
