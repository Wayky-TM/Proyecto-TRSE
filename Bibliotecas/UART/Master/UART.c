/*
 * UART.c
 *
 *  Created on: Jan 5, 2022
 *      Author: Álvaro
 */


#include "UART.h"


/* Helper function for error handling. */
int error_check(enum sp_return result);


#define ERROR_CHECK(result)	do{ \
								if( (result)!= SP_OK ){ \
									return UART_DEVERROR \
								} \
							}while(0)

static struct sp_port * port;


#if UART_DEBUG
#define check(result)	error_check(result)
#else
#define check(result)	ERROR_CHECK(result)
#endif


UART_status serial_init( const char * term_path )
{
	struct sp_port_config *config;

	check(sp_get_port_by_name(term_path, &port));
	check(sp_open(port, SP_MODE_READ_WRITE));

    check(sp_set_baudrate(port, 115200));
    check(sp_set_bits(port, 8));
    check(sp_set_parity(port, SP_PARITY_NONE));
    check(sp_set_stopbits(port, 1));
    check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

	check(sp_new_config(&config));
	check(sp_get_config(port, config));

    check(sp_set_config_baudrate(config, 115200));
    check(sp_set_config_bits(config, 8));
    check(sp_set_config_parity(config, SP_PARITY_NONE));
    check(sp_set_config_stopbits(config, 1));
    check(sp_set_config_flowcontrol(config, SP_FLOWCONTROL_NONE));
    check(sp_set_config(port, config));

    sp_free_config(config);

	return UART_SUCCESS;
}



UART_status serial_receive( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout  )
{
	enum sp_return ret;
	ret = sp_blocking_read(port, buffer_ptr, length, timeout);
	check(ret);

	if( ret >= 0 && ret < length )
	{
		return UART_TIMEOUT;		
	}

	return UART_SUCCESS;
}



UART_status serial_send( uint8_t * buffer_ptr, uint8_t length, uint32_t timeout )
{
	enum sp_return ret;
	ret = sp_blocking_write(port, buffer_ptr, length, timeout);
	check(ret);

	if( ret >= 0 && ret < length )
	{
		return UART_TIMEOUT;		
	}

	return UART_SUCCESS;
}



/* Helper function for error handling. */
int check(enum sp_return result)
{
        /* For this example we'll just exit on any error by calling abort(). */
        char *error_message;
 
        switch (result) {
        case SP_ERR_ARG:
                printf("Error: Invalid argument.\n");
                abort();
        case SP_ERR_FAIL:
                error_message = sp_last_error_message();
                printf("Error: Failed: %s\n", error_message);
                sp_free_error_message(error_message);
                abort();
        case SP_ERR_SUPP:
                printf("Error: Not supported.\n");
                abort();
        case SP_ERR_MEM:
                printf("Error: Couldn't allocate memory.\n");
                abort();
        case SP_OK:
        default:
                return result;
        }
}
