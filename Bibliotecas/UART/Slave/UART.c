
#include "UART.h"



__WEAK UART_status serial_receive( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout  )
{
	// ...

	return UART_SUCCESS;
}



__WEAK UART_status serial_send( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout )
{
	// ...

	return UART_SUCCESS;
}
