/*  ***************************************************************************
     *                    Carlos Antonio Orrego Mu�oz                        *
     *                                                                       *
    ***************************************************************************/

/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

/* Se definen las prioridades */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 ) /* M�xima prioridad */

/* Velocidad (ms) a la que los datos se env�an a la cola */
#define mainQUEUE_SEND_PERIOD_MS			( 200 / portTICK_RATE_MS )

/* Temporizador software */
#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS )

/* N�mero de elementos que almacenar� la cola */
#define mainQUEUE_LENGTH					( 1 )

/*-----------------------------------------------------------*/

/* Funciones para recibir y enviar a la cola */
static void prvQueueReceiveTask( void *pvParameters ); 	/* Implementaci�n de la funci�n de recepci�n de mensajes de la cola */
static void prvQueueSendTaskGreen( void *pvParameters );/* Implementaci�n de la funci�n de envi� de mensajes de la cola, Color verde */
static void prvQueueSendTaskOrange( void *pvParameters );/* Implementaci�n de la funci�n de envi� de mensajes de la cola, Color naranja */
static void prvQueueSendTaskRed( void *pvParameters );/* Implementaci�n de la funci�n de envi� de mensajes de la cola, Color rojo */
static void prvQueueSendTaskBlue( void *pvParameters );/* Implementaci�n de la funci�n de envi� de mensajes de la cola, Color azul */
static void prvQueueSendTaskUSART( void *pvParameters );/* Implementaci�n de la funci�n dpara recibir el mensaje por la UART */

/* Funci�n de devoluci�n del contador software */
static void vExampleTimerCallback( xTimerHandle xTimer );

/* Funci�n para la administraci�n del sem�foro */
static void prvEventSemaphoreTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* Cola para el envi� y recepci�n de datos */
static xQueueHandle xQueue = NULL;

/* Funci�n para el evento y tarea del sem�foro */
static xSemaphoreHandle xEventSemaphore = NULL;

/* Contadores utilizados */
static volatile uint32_t ulCountOfTimerCallbackExecutions = 0; 	/* Incrementa cada segundo */
static volatile uint32_t ulCountOfItemsReceivedOnQueue = 0; 	/* Incrementa cada vez que recibe el n�mero 100, como cada 200 ms llega un mensaje el contador incrementa en 5 cada segundo */
static volatile uint32_t ulCountOfReceivedSemaphores = 0;		/* Cuenta la administraci�n del semaforo cada 500 ms, osea que incrementa en dos cada segundo */

/*-----------------------------------------------------------*/
/* Funciones para inicializar la placa */
static void prvSetupHardware( void );
void start();													/* Funcion para iniciar */

/* Variables para manipular los puertos */
#define PORTA 			GPIOA 									/* Seleccion del puerto (PA) */
#define PUL_START 		GPIO_Pin_0								/* Seleccion del pin del puerto (PA0), pulsador User de la placa */
#define PUL_1 			GPIO_Pin_7								/* Pulsador 1 (PA1) */
#define PUL_2 			GPIO_Pin_5								/* Pulsador 2 (PA3) */
#define PUL_3 			GPIO_Pin_3								/* Pulsador 3 (PA5) */
#define PUL_4 			GPIO_Pin_1								/* Pulsador 4 (PA7) */

#define PORTB			GPIOB
#define	USART_TX		GPIO_Pin_10
#define	USART_RX		GPIO_Pin_11

#define PORTD 			GPIOD									/* Seleccion del puerto (PD) */
#define LED_GREENP 		GPIO_Pin_12								/* LED Verde */
#define LED_ORANGEP 	GPIO_Pin_13								/* LED Naranja */
#define LED_REDP 		GPIO_Pin_14								/* LED Rojo */
#define LED_BLUEP 		GPIO_Pin_15								/* LED Azul */
#define LED_PLA (LED_GREENP| LED_ORANGEP | LED_BLUEP | LED_REDP)/* Todos los LED */

/*-----------------------------------------------------------*/

int main(void)
{

xTimerHandle xExampleSoftwareTimer = NULL;

	prvSetupHardware();	/* Configuraci�n del sistema */
	start(); 			/* Funci�n para iniciar el Programa utilizando el pulsador User de la placa*/

	/* Crea la cola para el envi� y recepci�n de datos */
	xQueue = xQueueCreate( 	mainQUEUE_LENGTH,		/* N�mero de elementos que la cola puede contener (1) */
							sizeof( uint32_t ) );	/* Tama�o de cada elemento que contiene la cola */
	/* A�ade el registro, para el beneficio de la depuraci�n del kernel consciente. */
	vQueueAddToRegistry( xQueue, ( signed char * ) "MainQueue" );


	/* Crea el sem�foro usando la funci�n tick hook */
	vSemaphoreCreateBinary( xEventSemaphore );
	/* A�ade el registro, para el beneficio de la depuraci�n del kernel consciente. */
	vQueueAddToRegistry( xEventSemaphore, ( signed char * ) "xEventSemaphore" );


	/* Crea la cola para la recepci�n de datos */
	xTaskCreate( 	prvQueueReceiveTask,			/* Funci�n que implementa la tarea. */
					( signed char * ) "Rx", 		/* Nombre de texto para la tarea, s�lo para ayudar a la depuraci�n */
					configMINIMAL_STACK_SIZE, 		/* Tama�o (en palabras) de la pila que debe ser creado para la tarea. */
					NULL, 							/* Par�metro que puede pasar a la tarea */
					mainQUEUE_RECEIVE_TASK_PRIORITY,/* Prioridad asignada a la tarea */
					NULL );							/* Se utiliza para obtener un identificador de la tarea creada. */


	/* Crea la tarea para el envi� de datos */
	xTaskCreate( 	prvQueueSendTaskGreen,
						( signed char * ) "TX_Green",
						configMINIMAL_STACK_SIZE,
						NULL,
						mainQUEUE_SEND_TASK_PRIORITY,
						NULL );

	xTaskCreate( 	prvQueueSendTaskOrange,
						( signed char * ) "TX_Orange",
						configMINIMAL_STACK_SIZE,
						NULL,
						mainQUEUE_SEND_TASK_PRIORITY,
						NULL );

	xTaskCreate( 	prvQueueSendTaskRed,
						( signed char * ) "TX_Red",
						configMINIMAL_STACK_SIZE,
						NULL,
						mainQUEUE_SEND_TASK_PRIORITY,
						NULL );

	xTaskCreate( 	prvQueueSendTaskBlue,
						( signed char * ) "TX_Blue",
						configMINIMAL_STACK_SIZE,
						NULL,
						mainQUEUE_SEND_TASK_PRIORITY,
						NULL );

	xTaskCreate( 	prvQueueSendTaskUSART,
						( signed char * ) "USART_RX",
						configMINIMAL_STACK_SIZE,
						NULL,
						mainQUEUE_SEND_TASK_PRIORITY,
						NULL );

	/* Crear la tarea que se sincroniza con una interrupci�n utilizando el sem�foro xEventSemaphore */
	xTaskCreate( 	prvEventSemaphoreTask,
						( signed char * ) "Sem",
						configMINIMAL_STACK_SIZE,
						NULL,
						mainEVENT_SEMAPHORE_TASK_PRIORITY,
						NULL );

	/* Crea el timer software */
	xExampleSoftwareTimer = xTimerCreate( 	( const signed char * ) "LEDTimer", /* Nombre de texto para la depuraci�n */
								mainSOFTWARE_TIMER_PERIOD_MS,		/* Temporizador peri�dico, en esta caso 1000ms (1s). */
								pdTRUE,								/* Este es un temporizador peri�dico, por lo que se establece en xAutoReload pdTRUE */
								( void * ) 0,						/* El ID no se utiliza, por lo que se puede configurar para cualquier cosa. */
								vExampleTimerCallback				/* Funci�n de devoluci�n de llamada que cambia el LED apagado. */
							);

	/* Parametros de inicializaci�n del temporizador creado. */
	xTimerStart( xExampleSoftwareTimer, 0 );

	/* Iniciar las tareas y funcionamiento temporizador */
	vTaskStartScheduler();

	/* Si llega hasta aqui, significa que no hay sufuciente memoria para crear las tareas ociosas y/o temporizadores */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void vExampleTimerCallback( xTimerHandle xTimer )
{
	/* Temporizador autom�tico, despu�s de la carga ser� ejecutado peri�dicamente */
	ulCountOfTimerCallbackExecutions++;
}
/*-----------------------------------------------------------*/

static void prvQueueSendTaskGreen ( void *pvParameters )
{
const uint32_t ulValueToSend = 0x31; /* 1 - N�mero a enviar a la cola */

	for( ;; )
	{
		if (GPIO_ReadInputDataBit(PORTA,PUL_1)==SET)
			{
			/* Coloque esta tarea en el estado bloqueado hasta que es hora de correr de nuevo */
			vTaskDelay(mainQUEUE_SEND_PERIOD_MS );	/* 200 ms*/

			/* Enviar a la cola */
			xQueueSend( xQueue, 	/* Cabecera de la cola en la que el elemento debe ser publicado */
				&ulValueToSend,		/* Puntero sobre el objeto que se va a colocar en la cola.*/
				0 ); 				/* El 0 indica la cantidad m�xima de tiempo que la tarea debe
				 	 	 	 	 	 * bloquear la espera de espacio que est� disponible en la cola */
			}
	}
}

/*-----------------------------------------------------------*/

static void prvQueueSendTaskOrange ( void *pvParameters )
{
const uint32_t ulValueToSend = 0x32; /* 2 - N�mero a enviar a la cola */

	for( ;; )
	{
		if (GPIO_ReadInputDataBit(PORTA,PUL_2)==SET)
			{
			/* Coloque esta tarea en el estado bloqueado hasta que es hora de correr de nuevo */
			vTaskDelay(mainQUEUE_SEND_PERIOD_MS );	/* 200 ms*/

			/* Enviar a la cola */
			xQueueSend( xQueue, 	/* Cabecera de la cola en la que el elemento debe ser publicado */
				&ulValueToSend,		/* Puntero sobre el objeto que se va a colocar en la cola.*/
				0 ); 				/* El 0 indica la cantidad m�xima de tiempo que la tarea debe
				 	 	 	 	 	 * bloquear la espera de espacio que est� disponible en la cola */
			}
	}
}
/*-----------------------------------------------------------*/

static void prvQueueSendTaskRed ( void *pvParameters )
{
const uint32_t ulValueToSend = 0x33; /* 3 - N�mero a enviar a la cola */

	for( ;; )
	{
		if (GPIO_ReadInputDataBit(PORTA,PUL_3)==SET)
			{
			/* Coloque esta tarea en el estado bloqueado hasta que es hora de correr de nuevo */
			vTaskDelay(mainQUEUE_SEND_PERIOD_MS );	/* 200 ms*/

			/* Enviar a la cola */
			xQueueSend( xQueue, 	/* Cabecera de la cola en la que el elemento debe ser publicado */
				&ulValueToSend,		/* Puntero sobre el objeto que se va a colocar en la cola.*/
				0 ); 				/* El 0 indica la cantidad m�xima de tiempo que la tarea debe
				 	 	 	 	 	 * bloquear la espera de espacio que est� disponible en la cola */
			}
	}
}

/*-----------------------------------------------------------*/

static void prvQueueSendTaskBlue ( void *pvParameters )
{
const uint32_t ulValueToSend = 0x34; /* 4 - N�mero a enviar a la cola */

	for( ;; )
	{
		if (GPIO_ReadInputDataBit(PORTA,PUL_4)==SET)
			{
			/* Coloque esta tarea en el estado bloqueado hasta que es hora de correr de nuevo */
			vTaskDelay(mainQUEUE_SEND_PERIOD_MS );	/* 200 ms*/

			/* Enviar a la cola */
			xQueueSend( xQueue, 	/* Cabecera de la cola en la que el elemento debe ser publicado */
				&ulValueToSend,		/* Puntero sobre el objeto que se va a colocar en la cola.*/
				0 ); 				/* El 0 indica la cantidad m�xima de tiempo que la tarea debe
				 	 	 	 	 	 * bloquear la espera de espacio que est� disponible en la cola */
			}
	}
}

/*-----------------------------------------------------------*/

static void prvQueueSendTaskUSART ( void *pvParameters )
{
	uint16_t DataSend; /* Utilizado para enviar el numero recibido por USART a la cola */

	for( ;; )
	{
		if (USART_GetITStatus(USART3, USART_IT_RXNE) == RESET) /* Wait for Char */
			{
			DataSend = USART_ReceiveData(USART3); /* Collect Char */

			/* Enviar a la cola */
			xQueueSend( xQueue, 	/* Cabecera de la cola en la que el elemento debe ser publicado */
				&DataSend,			/* Puntero sobre el objeto que se va a colocar en la cola.*/
				0 ); 				/* El 0 indica la cantidad m�xima de tiempo que la tarea debe
				 	 	 	 	 	 * bloquear la espera de espacio que est� disponible en la cola */
			}
	}
}

/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
uint32_t ulReceivedValue;

	for( ;; )
	{
		/* Espere hasta que algo llega a la cola */
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		if( ulReceivedValue == 0x31 ) /* Recibe el No 1 */
			{
			/* Incrementa el contador */
			ulCountOfItemsReceivedOnQueue++;

			/*Cambia el estado del led */
			GPIO_ToggleBits(PORTD, LED_GREENP);
			}

		if( ulReceivedValue == 0x32 ) /* Recibe el No 2 */
			{
			/* Incrementa el contador */
			ulCountOfItemsReceivedOnQueue++;

			/*Cambia el estado del led */
			GPIO_ToggleBits(PORTD, LED_ORANGEP);
			}

		if( ulReceivedValue == 0x33 ) /* Recibe el No 3 */
			{
			/* Incrementa el contador */
			ulCountOfItemsReceivedOnQueue++;

			/*Cambia el estado del led */
			GPIO_ToggleBits(PORTD, LED_REDP);
			}

		if( ulReceivedValue == 0x34 ) /* Recibe el No 4 */
			{
			/* Incrementa el contador */
			ulCountOfItemsReceivedOnQueue++;

			/*Cambia el estado del led */
			GPIO_ToggleBits(PORTD, LED_BLUEP);
			}
	}
}

/*-----------------------------------------------------------*/

static void prvEventSemaphoreTask( void *pvParameters )
{
	for( ;; )
	{
		/* Solicita el sem�foro o bloquear hasta que el sem�foro se 'da - given' */
		xSemaphoreTake( xEventSemaphore, portMAX_DELAY );

		/* Cuenta el n�mero de veces que se recibi� el sem�foro */
		ulCountOfReceivedSemaphores++;
	}
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint32_t ulCount = 0;

	/* La funci�n RTOS tick hook es habilitada en setting configUSE_TICK_HOOK colocando
	1 en FreeRTOSConfig.h.

	"Give" Cede el sem�foro cada interrupci�n de 500 ms */
	ulCount++;
	if( ulCount >= 500UL )
	{
		/*  Esta funci�n se llama desde un contexto de interrupci�n.
			xHigherPriorityTaskWoken se inicializa a pdFALSE, y se ajustar� a
			pdTRUE por xSemaphoreGiveFromISR () si al dar el sem�foro desbloque� una
			tarea que tiene prioridad igual o m�s alta que la tarea interrumpida */

		xSemaphoreGiveFromISR( xEventSemaphore, 		/* Una Cabecera para el sem�foro liberado. Este es el identificador devuelto cuando se cre� el sem�foro.*/
						&xHigherPriorityTaskWoken );
		ulCount = 0UL;
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* malloc failed hook es habilitado colocando a 1 en configUSE_MALLOC_FAILED_HOOK en FreeRTOSConfig.h.

	Indica que no hay suficiente memoria disponible en la pila,  El tama�o de la pila FreeRTOS es fijado por la
	constante configTOTAL_HEAP_SIZE configurable en FreeRTOSConfig.h. */

	for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Comprobaci�n de desbordamiento de pila */
	for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void ) /* Consulta la cantidad de espacio de almacenamiento din�mico libre FreeRTOS */
{
volatile size_t xFreeStackSpace;

	/* La tarea ociosa se habilita estableciendo configUSE_IDLE_HOOK a 1 en FreeRTOSConfig.h */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* Por ahora, el n�cleo ha asignado todo el almacenamiento diamico libre, puede reducirse en
		 * configTOTAL_HEAP_SIZE en FreeRTOSConfig.h */
	}
}

/*-----------------------------------------------------------*/

void start()
{
	while (GPIO_ReadInputDataBit(PORTA,PUL_START)==Bit_RESET)	/* Espera hasta que se pulse User */
		{
		GPIO_SetBits(PORTD, LED_PLA);
		}
	GPIO_ResetBits(PORTD, LED_PLA);
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{

		  NVIC_SetPriorityGrouping( 0 );							/*Asegura que todos los bits de prioridad son asignados como bits de prioridad como preferencia. */
		  GPIO_InitTypeDef GPIO_InitStruct;							/* Definici�n de type */

		  /* Habilita los perifericos necesarios en el puerto D */
		  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

		  /* Habilita el clock para el USART3 */
		  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

		  /*-------------------------- Configurac�n GPIOA ----------------------------*/
		  /* Establece el pin de los pulsadores en el GPIO PA */
		  GPIO_InitStruct.GPIO_Pin = PUL_START|PUL_1|PUL_2|PUL_3|PUL_4;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		  GPIO_Init(PORTA, &GPIO_InitStruct);

		  /*-------------------------- Configurac�n GPIOB ----------------------------*/
		  GPIO_InitStruct.GPIO_Pin = USART_TX | USART_RX;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(PORTB, &GPIO_InitStruct);

		    /* Connect USART pins to AF */
		  GPIO_PinAFConfig(PORTB, GPIO_PinSource10, GPIO_AF_USART3);
		  GPIO_PinAFConfig(PORTB, GPIO_PinSource11, GPIO_AF_USART3);

		  USART_InitTypeDef USART_InitStructure;

		  /* USARTx configuration ------------------------------------------------------*/
		  /* USARTx configured as follow:
		        - BaudRate = 9600 baud
		        - Word Length = 8 Bits
		        - Two Stop Bit
		        - Odd parity
		        - Hardware flow control disabled (RTS and CTS signals)
		        - Receive and transmit enabled
		  */
		  USART_InitStructure.USART_BaudRate = 9600;
		  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		  USART_InitStructure.USART_StopBits = USART_StopBits_1;
		  USART_InitStructure.USART_Parity = USART_Parity_No;
		  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

		  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		  USART_Init(USART3, &USART_InitStructure);

		  USART_Cmd(USART3, ENABLE);

		  /*-------------------------- Configurac�n GPIOD ----------------------------*/
		  /* Establece el pin los LEDs en el GPIOD */
		  GPIO_InitStruct.GPIO_Pin = LED_PLA | LED_REDP;
		  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
		  /* PIO_Init(GPIOD, &GPIO_InitStruct); // Esta o la siguiente linea es valida */
		  GPIO_Init(PORTD, &GPIO_InitStruct);
		  GPIO_ResetBits(PORTD,LED_PLA);
}

/*-----------------------------------------------------------*/
