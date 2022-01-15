
#include "server.h"

#define UART_EOTx_FLAG	(0x1)
#define UART_EORx_FLAG	(0x2)


enum ConnectionState
{
	AWAITING_CONNECTION,
	AWAITING_RESEND,
	AWAITING_ACK
};


static int (*command_pointers[SUPPORTED_COMMANDS])(uint8_t, const union Data *, union Data *);


static uint32_t UART_timeout;
static UART_HandleTypeDef * UART_handler;
static enum ConnectionState current_state;

typedef StaticEventGroup_t osStaticEventGroupDef_t;

static osEventFlagsId_t UART_EventFlagsHandle;



void UART_initialize( UART_HandleTypeDef * huart, uint32_t timeout )
{
	UART_handler = huart;
	UART_timeout = timeout;
	UART_EventFlagsHandle = osEventFlagsNew(NULL);
	current_state = AWAITING_CONNECTION;
}



static int _run_command( uint8_t command, uint8_t address, const union Data * param, union Data * ret_param )
{
	if( command >= SUPPORTED_COMMANDS || command_pointers[command] == NULL )
	{
		return CMD_INVALID_CMD;
	}

	return command_pointers[command](address, param, ret_param);
}



static uint8_t _error_check( const struct UART_Tx_Frame * tx_frame )
{
	return 0;
}



/* TODO: ACK/deteccion de errores */
void UART_start_server( void )
{
	static struct UART_Tx_Frame tx_frame;
	static struct UART_Rx_Frame rx_frame;
	UART_status ret;
	int cmd_ret;
	union Data ret_param;

	while(1)
	{
		switch( current_state )
		{
		case AWAITING_CONNECTION:
			ret = serial_receive( (uint8_t *)&tx_frame, sizeof(struct UART_Tx_Frame), UART_TIMEOUT_FOREVER );

			if( ret == UART_SUCCESS )
			{
				if( !_error_check(&tx_frame) )
				{
					cmd_ret = _run_command( tx_frame.command, tx_frame.address, &tx_frame.data, &ret_param );

					switch( cmd_ret )
					{
						case CMD_SUCCESS:
							rx_frame.ack = ACK_SUCCESS;
							break;

						case CMD_INVALID_CMD:
							rx_frame.ack = ACK_INVALID_CMD;
							break;

						case CMD_INVALID_ADDR:
							rx_frame.ack = ACK_INVALID_ADDR;
							break;

						case CMD_INVALID_DATA:
							rx_frame.ack = ACK_INVALID_DATA;
							break;
					}

					rx_frame.data = ret_param;

					ret = serial_send( (uint8_t *)&rx_frame, sizeof(struct UART_Rx_Frame), UART_timeout );

//					if( ret == UART_SUCCESS )
//					{
//						current_state = AWAITING_ACK;
//					}
				}
//				else
//				{
//					current_state = AWAITING_RESEND;
//				}
			}
			break;

//		case AWAITING_RESEND:
//			break;
//
//		case AWAITING_ACK:
//			ret = serial_receive( (uint8_t *)&tx_frame, sizeof(struct UART_Tx_Frame), UART_timeout );
//			break;
		}
	}
}



int UART_register_command( uint8_t cmd, int (*f_ptr)(uint8_t, const union Data *, union Data *) )
{
	if( cmd >= SUPPORTED_COMMANDS || f_ptr == NULL )
	{
		return -1;
	}

	command_pointers[cmd] = f_ptr;

	return 0;
}



void UART_notify_RxCplt(void)
{
	osEventFlagsSet( UART_EventFlagsHandle, UART_EORx_FLAG );
}



void UART_notify_TxCplt(void)
{
	osEventFlagsSet( UART_EventFlagsHandle, UART_EOTx_FLAG );
}



UART_status serial_receive( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout )
{
	int ret;

	HAL_UART_Receive_IT(UART_handler, buffer_ptr, length);

	ret = osEventFlagsWait( UART_EventFlagsHandle, UART_EORx_FLAG, osFlagsWaitAny, timeout );

	if( ret == osFlagsErrorTimeout)
	{
		return UART_TIMEOUT;
	}
	else if( ret < 0 )
	{
		return UART_DEVERROR;
	}

	return UART_SUCCESS;
}



UART_status serial_send( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout )
{
	int ret;

	HAL_UART_Transmit_IT(UART_handler, buffer_ptr, length);

	ret = osEventFlagsWait( UART_EventFlagsHandle, UART_EOTx_FLAG, osFlagsWaitAny, timeout );

	if( ret == osFlagsErrorTimeout)
	{
		return UART_TIMEOUT;
	}
	else if( ret < 0 )
	{
		return UART_DEVERROR;
	}

	return UART_SUCCESS;
}

