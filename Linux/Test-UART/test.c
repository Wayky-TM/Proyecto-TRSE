#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include "client.h"


#define USE_TERMIOS	(0)


#define CMD_SWITCH_LED_ANIMATION	(0x1)
#define CMD_READ_ANALOG				(0x3)

#define ADDR_ANALOG_X	(0)
#define ADDR_ANALOG_Y	(1)
#define ADDR_ANALOG_Z	(2)
#define ADDR_ANALOG_T	(3)


pthread_t storeLog;
pthread_t interfaceController;
pthread_t commandLedsTask;

pthread_mutex_t serial_mutex;

int modo = 0;
uint8_t animacion_actual = 1;

char nombre_logs[60];
char folder_path[100];
char file_path[150];

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

	t = time(NULL);
	tm = *localtime(&t);

	sprintf( nombre_logs, "Log_%d-%02d-%02d_%02d:%02d:%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec );
	strcpy( file_path, folder_path );
	strcat( file_path, nombre_logs );

    fd = fopen(file_path, "w+");

    if (fd < 0)
    {
        perror("Error opening file");
        pthread_exit(NULL);
    }

    fprintf(fd, "%55s\n\n", "--- Log valores de sensores ---");
    fprintf(fd, "%10s %15s %20s %25s\n", "angulo_x", "angulo_y", "angulo_z", "temperatura"); // Cabeceras para formato .csv

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
			fprintf(fd, "%10f %15f %20f %25f\n", X, Y, Z, Temp);
//			printf( "(%f,%f,%f,%f)\n", X, Y, Z, Temp);

		}
		else if( modo == 9 )
		{
			break;
		}

		sleep(1);
    }

	fclose(fd);
	pthread_exit(NULL);
}



void* commandLeds()
{
	union Data tx_data, rx_data;

	while(1)
    {
		if(modo == 2)
		{
			tx_data.W = animacion_actual;

			pthread_mutex_lock (&serial_mutex);

			send_command( CMD_SWITCH_LED_ANIMATION, 0x00, &tx_data, &rx_data, UART_TIMEOUT_FOREVER, 3 );

			pthread_mutex_unlock (&serial_mutex);
		}
		else if( modo == 9 )
		{
			break;
		}

		sleep(1);
    }

	pthread_exit(NULL);
}



void* interface()
{
	int input;
	int configuration;

	while(1)
	{
		printf("Selecciona el modo: \n");
		printf("1. Lectura de sensores.\n");
		printf("2. Comandar LEDS.\n");
		printf("9. Finalizar programa.\n");
		scanf("%d", &input);


		if (input == 1)
		{
			modo = 1;
		}
		else if (input == 2)
		{
			modo = 2;

			do{
				printf("\nElige la configuración:\n");
				printf("1. Animación 1\n");
				printf("2. Animación 2\n");
				scanf("%d", &configuration);
			}while( configuration!=1 && configuration!=2 );

			animacion_actual = configuration;


		}
		else if (input == 9)
		{
			modo = 9;
			break;
		}
		else
		{
			printf("Modo erroneo seleccionado. \n");
		}
	}

	pthread_exit(NULL);
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

    printf("%50s\n", "--- Proyecto tiempo real ---");

    DIR* dir;

    while(1)
    {
    	printf("\nIntroduzca el directorio donde guardar los logs:");
    	scanf( "%s", folder_path );

    	dir = opendir(folder_path);

    	if (dir){ // La carpeta existe
			closedir(dir);
			break;
		}
		else
		{
			printf("Error: la carpeta elegida no existe\n\n");
		}
    }

    printf("\n");
//   	strcat(current_path, "/logs/");
//
//   	DIR* dir = opendir(current_path);
//
//   	if (dir){ // La carpeta existe
//   	    closedir(dir);
//   	}
//   	else
//   	{
//   		if( mkdir(current_path, 0777) < 0 ){
//			perror("'mkdir' error");
//			return -1;
//   		}
//   	}

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
