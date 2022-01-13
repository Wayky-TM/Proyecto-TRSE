
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include "client.h"


#define CMD_WRITE_LED			(0x1)
#define CMD_WRITE_MULTIPLE_LEDS	(0x2)
#define CMD_READ_ANALOG			(0x3)

#define ADDR_ANALOG_X	(0)
#define ADDR_ANALOG_Y	(1)
#define ADDR_ANALOG_Z	(2)
#define ADDR_ANALOG_T	(3)


pthread_t storeLog;
pthread_t interfaceController;
pthread_t commandLeds;

int modo = 0;

int configureUART(int fd)
{
    struct termios device;

    if (tcgetattr(fd, &device) != 0)
    {
        perror("Error in tcgetattr \n");
        return -1;
    }

    cfsetospeed(&device, BAUDRATE);
    cfsetispeed(&device, BAUDRATE);

    device.c_cflag = (device.c_cflag & ~CSIZE) | CS8; 
    device.c_iflag &= ~IGNBRK; 
    device.c_lflag = 0;        
    device.c_oflag = 0;        
    device.c_iflag &= ~(IXON | IXOFF | IXANY); 
    device.c_cflag |= (CLOCAL | CREAD);   
    device.c_cflag &= ~PARENB
    device.c_cflag &= ~CSTOPB
    device.c_cflag &= ~CSIZE;
    device.c_cflag |= CS8;

    if (tcsetattr(fd, TCSANOW, &device) != 0) {
        perror("Error from tcsetattr \n");
        return -1;
    }

    return 0;
}

void* logger()
{
    FILE *fd;

	char input[20];

	printf("Introduce nombre de fichero: \n");
	scanf("%s", &input);
	strcat (input, ".txt");
	//Igual si hay tiempo poner en la interfaz que se pueda
	//crear un archivo si se quiere en lugar de uno generico cada vez
    fd = fopen(input, "w+");
    if (fd == -1)
    {
        perror("Error opening file");
        pthread_exit(NULL);
    }

    while(1)
    {
		if(modo == 1)
		{
			//poner aqui la funcion de leer la uart
			fprintf(fd, "Valores de la uart\n");
		}
    }

	fclose(fd);
	pthread_exit(NULL);
}

void* commandLeds()
{
	while(1)
    {
		if(modo == 2)
		{
			//poner aqui la funcion de escribir en la uart

		}
    }
}

void* interface()
{
	int input;

	while(true)
	{
		printf("Selecciona el modo: \n");
		printf("1. Lectura de sensores.\n");
		printf("2. Comandar LEDS.\n");
		scanf("%d", &input);
		if (input == 1)
		{
			modo = 1;
		}
		else if (input == 2)
		{
			modo = 2;
		}
		else
		{
			printf("Modo erroneo seleccionado.");
		}
	}
	
}

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

	/*int fd;
    char* tty = DEVICE;

    fd = open(tty, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("error opening port");
        return -1;
    }*/

    //configureUART(fd);

	if (pthread_create(&storeLog, NULL, (void *)logger,NULL) != 0) 
	{
		exit(2);
  	}

	if (pthread_create(&interfaceController, NULL, (void *)interface,NULL) != 0) 
	{
		exit(2);
  	}

	if (pthread_create(&commandLeds, NULL, (void *)commandLeds, NULL) != 0)
	{
		exit(2);
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
