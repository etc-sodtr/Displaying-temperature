/*
    FreeRTOS V7.1.1 - Copyright (C) 2012 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?                                      *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, training, latest information,
    license and contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell
    the code with commercial support, indemnification, and middleware, under
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* TODO Add any manufacture supplied header files necessary for CMSIS functions
to be available here. */
#include "stm32f4xx.h"
#include"stm32f4xx_hal.h"
#include"stm32f4xx_hal_adc.h"
#include"stm32f4xx_hal_adc_ex.h"
#include"stm32f4xx_hal_cortex.h"
#include"stm32f4xx_hal_def.h"
#include"stm32f4xx_hal_dma.h"
#include"stm32f4xx_hal_dma2d.h"
#include"stm32f4xx_hal_dma_ex.h"
#include"stm32f4xx_hal_flash.h"
#include"stm32f4xx_hal_flash_ex.h"
#include"stm32f4xx_hal_flash_ramfunc.h"
#include"stm32f4xx_hal_gpio.h"
#include"stm32f4xx_hal_gpio_ex.h"
#include"stm32f4xx_hal_i2c.h"
#include"stm32f4xx_hal_i2c_ex.h"
#include"stm32f4xx_hal_ltdc.h"
#include"stm32f4xx_hal_ltdc_ex.h"
#include"stm32f4xx_hal_pwr.h"
#include"stm32f4xx_hal_pwr_ex.h"
#include"stm32f4xx_hal_rcc.h"
#include"stm32f4xx_hal_rcc_ex.h"
#include"stm32f4xx_hal_sdram.h"
#include"stm32f4xx_hal_spi.h"
#include"stm32f4xx_hal_sram.h"
#include"stm32f4xx_hal_tim.h"
#include"stm32f4xx_hal_tim_ex.h"
#include"stm32f4xx_hal_uart.h"
#include"stm32f4xx_ll_fmc.h"
#include"system_stm32f4xx.h"


/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_PERIOD_MS			( 200 / portTICK_RATE_MS )

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

/*-----------------------------------------------------------*/

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/*
 * The callback function assigned to the example software timer as described at
 * the top of this file.
 */
static void vExampleTimerCallback( xTimerHandle xTimer );

/*
 * The event semaphore task as described at the top of this file.
 */
static void prvEventSemaphoreTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static xSemaphoreHandle xEventSemaphore = NULL;

/* The counters used by the various examples.  The usage is described in the
 * comments at the top of this file.
 */
static volatile uint32_t ulCountOfTimerCallbackExecutions = 0;
static volatile uint32_t ulCountOfItemsReceivedOnQueue = 0;
static volatile uint32_t ulCountOfReceivedSemaphores = 0;

void SystemClock_Config(void)
{

	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 288;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 6;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
	SystemCoreClockUpdate();

	if (HAL_GetREVID() == 0x1001)
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}
ADC_HandleTypeDef g_AdcHandle;

void ConfigureADC()
{
	GPIO_InitTypeDef gpioInit;

	__GPIOC_CLK_ENABLE();
	__ADC1_CLK_ENABLE();

	gpioInit.Pin = GPIO_PIN_1;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &gpioInit);

	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	ADC_ChannelConfTypeDef adcChannel;

	g_AdcHandle.Instance = ADC1;

	g_AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	g_AdcHandle.Init.ScanConvMode = DISABLE;
	g_AdcHandle.Init.ContinuousConvMode = ENABLE;
	g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	g_AdcHandle.Init.NbrOfDiscConversion = 0;
	g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	g_AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	g_AdcHandle.Init.NbrOfConversion = 1;
	g_AdcHandle.Init.DMAContinuousRequests = ENABLE;
	g_AdcHandle.Init.EOCSelection = DISABLE;

	HAL_ADC_Init(&g_AdcHandle);

	adcChannel.Channel = ADC_CHANNEL_11;
	adcChannel.Rank = 1;
	adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcChannel.Offset = 0;

	if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
	{
		asm("bkpt 255");
	}
}
uint32_t g_ADCValue;
int g_MeasurementNumber;
/*-----------------------------------------------------------*/

int main(void)
{
	HAL_Init();
    SystemClock_Config();
    ConfigureADC();

    HAL_ADC_Start(&g_AdcHandle);
xTimerHandle xExampleSoftwareTimer = NULL;

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();


	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xQueue = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint32_t ) );	/* The size of each item the queue holds. */
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue, ( signed char * ) "MainQueue" );


	/* Create the semaphore used by the FreeRTOS tick hook function and the
	event semaphore task. */
	vSemaphoreCreateBinary( xEventSemaphore );
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xEventSemaphore, ( signed char * ) "xEventSemaphore" );


	/* Create the queue receive task as described in the comments at the top
	of this	file.  http://www.freertos.org/a00125.html */
	xTaskCreate( 	prvQueueReceiveTask,			/* The function that implements the task. */
					( signed char * ) "Rx", 		/* Text name for the task, just to help debugging. */
					configMINIMAL_STACK_SIZE, 		/* The size (in words) of the stack that should be created for the task. */
					NULL, 							/* A parameter that can be passed into the task.  Not used in this simple demo. */
					mainQUEUE_RECEIVE_TASK_PRIORITY,/* The priority to assign to the task.  tskIDLE_PRIORITY (which is 0) is the lowest priority.  configMAX_PRIORITIES - 1 is the highest priority. */
					NULL );							/* Used to obtain a handle to the created task.  Not used in this simple demo, so set to NULL. */


	/* Create the queue send task in exactly the same way.  Again, this is
	described in the comments at the top of the file. */
	xTaskCreate( 	prvQueueSendTask,
					( signed char * ) "TX",
					configMINIMAL_STACK_SIZE,
					NULL,
					mainQUEUE_SEND_TASK_PRIORITY,
					NULL );


	/* Create the task that is synchronised with an interrupt using the
	xEventSemaphore semaphore. */
	xTaskCreate( 	prvEventSemaphoreTask,
					( signed char * ) "Sem",
					configMINIMAL_STACK_SIZE,
					NULL,
					mainEVENT_SEMAPHORE_TASK_PRIORITY,
					NULL );


	/* Create the software timer as described in the comments at the top of
	this file.  http://www.freertos.org/FreeRTOS-timers-xTimerCreate.html. */
	xExampleSoftwareTimer = xTimerCreate( 	( const signed char * ) "LEDTimer", /* A text name, purely to help debugging. */
								mainSOFTWARE_TIMER_PERIOD_MS,		/* The timer period, in this case 1000ms (1s). */
								pdTRUE,								/* This is a periodic timer, so xAutoReload is set to pdTRUE. */
								( void * ) 0,						/* The ID is not used, so can be set to anything. */
								vExampleTimerCallback				/* The callback function that switches the LED off. */
							);

	/* Start the created timer.  A block time of zero is used as the timer
	command queue cannot possibly be full here (this is the first timer to
	be created, and it is not yet running).
	http://www.freertos.org/FreeRTOS-timers-xTimerStart.html */
	xTimerStart( xExampleSoftwareTimer, 0 );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details.  http://www.freertos.org/a00111.html */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void vExampleTimerCallback( xTimerHandle xTimer )
{
	/* The timer has expired.  Count the number of times this happens.  The
	timer that calls this function is an auto re-load timer, so it will
	execute periodically. http://www.freertos.org/RTOS-software-timer.html */
	ulCountOfTimerCallbackExecutions++;
}
/*-----------------------------------------------------------*/

static void prvQueueSendTask( void *pvParameters )
{
portTickType xNextWakeTime;
const uint32_t ulValueToSend = 100UL;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again.
		The block time is specified in ticks, the constant used converts ticks
		to ms.  While in the Blocked state this task will not consume any CPU
		time.  http://www.freertos.org/vtaskdelayuntil.html */
		vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_PERIOD_MS );

		/* Send to the queue - causing the queue receive task to unblock and
		increment its counter.  0 is used as the block time so the sending
		operation will not block - it shouldn't need to block as the queue
		should always be empty at this point in the code. */
		xQueueSend( xQueue, &ulValueToSend, 0 );
	}
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
uint32_t ulReceivedValue;

	for( ;; )
	{if (HAL_ADC_PollForConversion(&g_AdcHandle, 1000000) == HAL_OK)
	{
		g_ADCValue = HAL_ADC_GetValue(&g_AdcHandle);
		g_MeasurementNumber++;
	}
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h.  http://www.freertos.org/a00118.html */
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, increment the counter. */
		if( ulReceivedValue == 100UL )
		{
			/* Count the number of items that have been received correctly. */
			ulCountOfItemsReceivedOnQueue++;
		}
	}
}
/*-----------------------------------------------------------*/

static void prvEventSemaphoreTask( void *pvParameters )
{
	for( ;; )
	{
		/* Block until the semaphore is 'given'. */
		xSemaphoreTake( xEventSemaphore, portMAX_DELAY );

		/* Count the number of times the semaphore is received. */
		ulCountOfReceivedSemaphores++;
	}
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint32_t ulCount = 0;

	/* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
	1 in FreeRTOSConfig.h.

	"Give" the semaphore on every 500th tick interrupt. */
	ulCount++;
	if( ulCount >= 500UL )
	{
		/* This function is called from an interrupt context (the RTOS tick
		interrupt),	so only ISR safe API functions can be used (those that end
		in "FromISR()".

		xHigherPriorityTaskWoken was initialised to pdFALSE, and will be set to
		pdTRUE by xSemaphoreGiveFromISR() if giving the semaphore unblocked a
		task that has equal or higher priority than the interrupted task.
		http://www.freertos.org/a00124.html */
		xSemaphoreGiveFromISR( xEventSemaphore, &xHigherPriorityTaskWoken );
		ulCount = 0UL;
	}

	/* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
	normally be performed before leaving the interrupt (because during the
	execution of the interrupt a task of equal or higher priority than the
	running task was unblocked).  The syntax required to context switch from
	an interrupt is port dependent, so check the documentation of the port you
	are using.  http://www.freertos.org/a00090.html

	In this case, the function is running in the context of the tick interrupt,
	which will automatically check for the higher priority task to run anyway,
	so no further action is required. */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}


/*
 * Callback used by stm324xg_eval_i2c_ee.c.
 * Refer to stm324xg_eval_i2c_ee.h for more info.
 */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
  /* TODO, implement your code here */
  while (1)
  {
  }
}
