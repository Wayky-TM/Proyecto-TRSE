
#include "client.h"



TxRx_status send_command( uint8_t cmd,
						  uint8_t address,
						  const union Data * tx_data,
						  union Data * rx_data,
						  uint32_t timeout,
						  unsigned int max_retries )
{
	static struct UART_Tx_Frame tx_frame;
	static struct UART_Rx_Frame rx_frame;

	UART_status ret;
	int success = 0;

	tx_frame.command = cmd;
	tx_frame.address = address;
	tx_frame.data.W = tx_data->W;
	tx_frame.crc = 0xFFFF;

	do{
		ret = serial_send( (uint8_t*)&tx_frame, sizeof(struct UART_Tx_Frame), timeout );

		if( ret == UART_SUCCESS )
		{
			ret = serial_receive( (uint8_t*)&rx_frame, sizeof(struct UART_Rx_Frame), timeout );

			if( ret == UART_SUCCESS && rx_frame.ack == ACK_SUCCESS )
			{
				rx_data->W = rx_frame.data.W;
				success = 1;
			}

		}

	}while( !success && max_retries-- > 0 );

	return ( !success ? ( ret==UART_TIMEOUT ? TxRx_TIMEOUT_ERR : TxRx_DEV_ERR ) : TxRx_SUCCESS );
}


