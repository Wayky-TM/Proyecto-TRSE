#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <dirent.h>
#include "client.h"


#define USE_TERMIOS	(0)


#define CMD_WRITE_LED			(0x1)
#define CMD_WRITE_MULTIPLE_LEDS	(0x2)
#define CMD_READ_ANALOG			(0x3)

#define ADDR_ANALOG_X	(0)
#define ADDR_ANALOG_Y	(1)
#define ADDR_ANALOG_Z	(2)
#define ADDR_ANALOG_T	(3)


pthread_t storeLog;
pthread_t interfaceController;
pthread_t commandLedsTask;

pthread_mutex_t serial_mutex;

int modo = 0;

char nombre_logs[50];
char ruta_logs[100];

#if USE_TERMIOS
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
#endif

void* logger()
{
    FILE *fd;
    union Data tx_data, rx_data;
    float X, Y, Z, Temp;

    time_t t;
    struct tm tm;

	char ruta_completa[150];

//	printf("Introduce nombre de fichero: \n");
//	scanf("%s", &input);
//	strcat (input, ".txt");

	//Igual si hay tiempo poner en la interfaz que se pueda
	//crear un archivo si se quiere en lugar de uno generico cada vez


	t = time(NULL);
	tm = *localtime(&t);

	sprintf( nombre_logs, "Log_%d-%02d-%02d_%02d:%02d:%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec );
	strcpy(ruta_completa, ruta_logs);
	strcat(ruta_completa, nombre_logs);

    fd = fopen(ruta_completa, "w+");

    if (fd < 0)
    {
        perror("Error opening file");
        pthread_exit(NULL);
    }

    fprintf(fd, "angulo_x,angulo_y,angulo_z,temperatura\n"); // Cabeceras para formato .csv

    while(1)
    {
		if(modo == 1)
		{

			pthread_mutex_lock (&serial_mutex);

			send_command( CMD_READ_ANALOG, ADDR_ANALOG_X, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );
			X = rx_data.F;

			send_command( CMD_READ_ANALOG, ADDR_ANALOG_Y, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );
			Y = rx_data.F;

			send_command( CMD_READ_ANALOG, ADDR_ANALOG_Z, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );
			Z = rx_data.F;

			send_command( CMD_READ_ANALOG, ADDR_ANALOG_T, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );
			Temp = rx_data.F;

			pthread_mutex_unlock (&serial_mutex);

			/* Escritura al log */
			fprintf(fd, "%f,%f,%f,%f\n", X, Y, Z, Temp);

			sleep(1);
		}
    }

	fclose(fd);
	pthread_exit(NULL);
}



void* commandLeds()
{
	union Data tx_data, rx_data;
	uint8_t contador = 0;

	while(1)
    {
		if(modo == 2)
		{
			tx_data.W = contador++;

			pthread_mutex_lock (&serial_mutex);

			send_command( CMD_WRITE_MULTIPLE_LEDS, 0x00, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );

			pthread_mutex_unlock (&serial_mutex);

			sleep(1);
		}
    }
}



void* interface()
{
	int input;

	while(1)
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
			printf("Modo erroneo seleccionado. \n");
		}
	}

}

int main( int argc, char ** argv )
{

#if USE_TERMIOS
	int fd;
    char* tty = DEVICE;

    fd = open(tty, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("error opening port");
        return -1;
    }

    configureUART(fd);

#else
    if( serial_init( "/dev/ttyS0" ) != UART_SUCCESS )
	{
		printf("Error al abrir el puerto serie\n");

		return -1;
	}
#endif

    printf("\n");

    uint8_t path_is_valid=0;
    DIR* dir;


    while( !path_is_valid ){
    	printf("Introduzca la ruta donde se guardaran los logs:\n");
    	scanf("%s", ruta_logs);

    	dir = opendir(ruta_logs);
    	if (dir) {
    		path_is_valid = 1;
    	    closedir(dir);
    	}
    	else{
    		printf("La ruta introducida no es valida\n\n");
    	}

    }

    //printf("\e[1;1H\e[2J"); // Limpia el terminal
    printf("\n");

    pthread_mutex_init(&serial_mutex, NULL);

	if (pthread_create(&storeLog, NULL, (void *)logger,NULL) != 0)
	{
		exit(2);
  	}

	if (pthread_create(&interfaceController, NULL, (void *)interface,NULL) != 0)
	{
		exit(2);
  	}

	if (pthread_create(&commandLedsTask, NULL, (void *)commandLeds, NULL) != 0)
	{
		exit(2);
	}

	pthread_join( storeLog, NULL);
	pthread_join( interfaceController, NULL);
	pthread_join( commandLedsTask, NULL);

	return 0;
}
