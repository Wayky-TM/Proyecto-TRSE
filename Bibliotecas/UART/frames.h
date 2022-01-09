/*
 * frames.h
 *
 *  Created on: Jan 3, 2022
 *      Author: Álvaro
 */

#ifndef _UART_FRAMES_H_
#define _UART_FRAMES_H_

#include <stdint.h>


union Data
{
	uint8_t B[4];
	uint16_t HW[2];
	uint32_t W;
	float F;
};




/*
 * Transmisión
 *
 * */


/* Máximo número de comandos definibles,
 * con códigos de comando entre 0x0 y SUPPORTED_COMMANDS-1 */
#define SUPPORTED_COMMANDS	(192)

/* Control de errores */
#define CMD_ACK				(0xFF)
#define CMD_RESEND			(0xFE)


#pragma pack(1)
struct UART_Tx_Frame
{
	uint8_t command;
	uint8_t address;
	union Data data;
	uint16_t crc;
};





/*
 * Recepción
 *
 * */

/* Ack's */
#define ACK_SUCCESS			(0x1)
#define ACK_CRC_ERR			(0x2)
#define ACK_INVALID_ADDR	(0x3)
#define ACK_INVALID_DATA	(0x4)
#define ACK_INVALID_CMD		(0x5)


#pragma pack(1)
struct UART_Rx_Frame
{
	uint8_t ack;
	union Data data;
	uint16_t crc;
};



#endif /* _UART_FRAMES_H_ */
