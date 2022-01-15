
#ifndef _CLIENT_H_
#define _CLIENT_H_

#include "UART.h"

typedef int8_t TxRx_status;

#define TxRx_SUCCESS		(0)
#define TxRx_DEV_ERR		(-1)
#define TxRx_TIMEOUT_ERR	(-2)
#define TxRx_INV_ADDR_ERR	(-3)
#define TxRx_INV_CMD_ERR	(-4)
#define TxRx_INV_DATA_ERR	(-5)



/*
 * Comandos para la lectura de los registros definidos en
 * 'registros.h'
 *
 * */


/* Función genérica */
TxRx_status send_command( uint8_t cmd,
						  uint8_t address,
						  const union Data * tx_data,
						  union Data * rx_data,
						  uint32_t timeout,
						  unsigned int max_retries );


#endif /* _UART_MASTER_H_ */
