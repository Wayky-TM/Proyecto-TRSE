
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "client.h"



int main( int argc, char ** argv )
{
	union Data tx_data, rx_data;

	uint8_t contador = 0;
	float X, Y, Z, Temp;

	printf("\n");

	if( serial_init( "/dev/ttyS0" ) != UART_SUCCESS )
	{
		printf("Error al abrir el puerto serie\n");

		return -1;
	}


	while(1)
	{
//		switch(  )
//		{
//			case TxRx_SUCCESS:
//				printf("Exito!\n");
//				break;
//
//			case TxRx_DEV_ERR:
//				printf("Error de dispositivo\n");
//				break;
//
//			case TxRx_TIMEOUT_ERR:
//				printf("Timeout\n");
//				break;
//
//			case TxRx_INV_ADDR_ERR:
//				printf("Direccion invalida\n");
//				break;
//
//			case TxRx_INV_CMD_ERR:
//				printf("Comando invalidos\n");
//				break;
//
//			case TxRx_INV_DATA_ERR:
//				printf("Datos invalidos\n");
//				break;
//		}

		tx_data.W = contador++;

		send_command( 0x02, 0x00, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 ); // Leds


		send_command( 0x03, 0x00, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );
		X = rx_data.F;

		send_command( 0x03, 0x01, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );
		Y = rx_data.F;

		send_command( 0x03, 0x02, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );
		Z = rx_data.F;

		send_command( 0x03, 0x03, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );
		Temp = rx_data.F;

		printf( "Medidas: (X: %f, Y: %f, Z: %f, T: %f)\n", X, Y, Z, Temp );

		sleep(1);
	}

	return 0;
}
