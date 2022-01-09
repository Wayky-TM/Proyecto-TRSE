/*
 * server.h
 *
 *  Created on: Jan 5, 2022
 *      Author: Álvaro
 */

#ifndef _SLAVE_SERVER_H_
#define _SLAVE_SERVER_H_


#include "UART.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"


/* Valores de retorno esperados para los comandos registrados
 * con 'UART_register_command(...)' */
#define CMD_SUCCESS	 (0)
#define CMD_INVALID_CMD  (-1)
#define CMD_INVALID_ADDR (-2)
#define CMD_INVALID_DATA (-3)


/* Inicialización de los parámetros internos del servidor.
 * El argumento 'timeout' especifica el tiempo máximo que puede
 * transcurrir entre intercambios de mensaje antes de que se aborte
 * la transmisión */
void UART_initialize( UART_HandleTypeDef * huart, uint32_t timeout );


/* Ejecuta el servidor, procesando comandos indefinidamente */
void UART_start_server(void);


/* Vincula el comando 'cmd' con la función 'f_ptr', de forma que sea posible
 * definir nuevos comandos sin modificar la implementación */
int UART_register_command( uint8_t cmd, int (*f_ptr)(uint8_t, const union Data *, union Data *) );


/* Notifica a la tarea del servidor que se ha completado una recepción por UART. */
void UART_notify_RxCplt(void);


/* Notifica a la tarea del servidor que se ha completado una transmisión por UART. */
void UART_notify_TxCplt(void);



#endif /* UART_SLAVE_SERVER_H_ */
