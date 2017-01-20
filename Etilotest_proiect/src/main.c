
/* Standard includes. */
#include <stdint.h>
#include "main.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "stm32f4xx.h"

#define display_task_PRIORITY		   ( 2 )
#define	adc_conversion_task_PRIORITY   ( 1 )

#define MESSAGE1   "Sensor heating   "
#define MESSAGE2   "      %d:%d      "
#define MESSAGE3   "   Etilotest     "
#define MESSAGE4   "     SODTR       "
#define MESSAGE5   "  ADC3 = %d      "
#define MESSAGE6   " ALC. DETECTED   "
#define MESSAGE7   "    NO ALC.      "

#define LINENUM            0x15
#define FONTSIZE         Font12x12
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
#define SENSOR_READY 120
#define ALC_DET 2500

#define senzor_READ_PERIOD	( 200 / portTICK_RATE_MS )

#define senzor_init_time   ( 1000 / portTICK_RATE_MS )

#define mainQUEUE_LENGTH			( 1 )

/*-----------------------------------------------------------*/

static void Display_Init(void);

static void Display_Countdown(uint32_t);

static void Display(uint32_t);

static void ADC_CH_Config(void);

static void prvSetupHardware( void );

static void display_ADC( void *pvParameters );

static void read_ADC( void *pvParameters );

static void enable_ADC_READ( xTimerHandle xTimer );

static xSemaphoreHandle ADC_Semaphore = NULL;

xTimerHandle senzor_initialization_Timer = NULL;

static xQueueHandle xQueue = NULL;

static volatile uint32_t count_second_INIT = 0;

/*-----------------------------------------------------------*/

int main(void)
{

    Display_Init();
    ADC_CH_Config();

	/* Configure the system ready */
	prvSetupHardware();

	xQueue = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint32_t ) );	/* The size of each item the queue holds.  */

    vSemaphoreCreateBinary( ADC_Semaphore );

    xSemaphoreTake( ADC_Semaphore, portMAX_DELAY );


	xTaskCreate(   	display_ADC,
			        ( signed char * )"Display value",
					configMINIMAL_STACK_SIZE,
					NULL,
					display_task_PRIORITY,
					NULL );

	xTaskCreate( 	read_ADC,
			        ( signed char * )"Read ADC",
					configMINIMAL_STACK_SIZE,
					NULL,
					adc_conversion_task_PRIORITY,
					NULL );


	senzor_initialization_Timer = xTimerCreate(	( const signed char * ) "1 second timer",
								senzor_init_time,
								pdTRUE,
								( void * ) 0,
								enable_ADC_READ
							);


	LCD_DisplayStringLine(LCD_LINE_5, (uint8_t*)MESSAGE1);

	xTimerStart( senzor_initialization_Timer, 0 );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	for( ;; );
}
/*-----------------------------------------------------------*/

static void enable_ADC_READ ( xTimerHandle xTimer )
{

	if(count_second_INIT < SENSOR_READY){

	count_second_INIT++;

	Display_Countdown(count_second_INIT);

	}
	else {

		xSemaphoreGive( ADC_Semaphore);

		//LCD_ClearLine(LCD_LINE_5);

		xTimerStop (senzor_initialization_Timer, 0);

	}
}
/*-----------------------------------------------------------*/


static void read_ADC( void *pvParameters )
{
portTickType xNextWakeTime;
uint32_t adc_read_value = 0UL;



	 xNextWakeTime = xTaskGetTickCount();
	 if(xSemaphoreTake( ADC_Semaphore, portMAX_DELAY )==pdTRUE){

	for( ;; )
	{

		vTaskDelayUntil( &xNextWakeTime, senzor_READ_PERIOD );

		ADC_SoftwareStartConv(ADC3);

		while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == RESET);

		adc_read_value =  ADC_GetConversionValue(ADC3);

		xQueueSend( xQueue, &adc_read_value, portMAX_DELAY );



	}
	}
}
/*-----------------------------------------------------------*/

static void display_ADC( void *pvParameters )
{
uint32_t ulReceivedValue = 0;

	for( ;; )
	{

		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

	    Display(ulReceivedValue);

	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.*/

	NVIC_SetPriorityGrouping( 0 );

}

static void Display_Countdown(uint32_t Seconds)
{

  uint8_t aTextBuffer[50];
  Seconds = (uint32_t)SENSOR_READY - Seconds;
  uint32_t s = Seconds%60;
  uint32_t m = Seconds/60;

  sprintf((char*)aTextBuffer, MESSAGE2, m,s);
  LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)aTextBuffer);

}

static void Display(uint32_t ADC_value)
{

  uint8_t aTextBuffer[50];

  if(ADC_value<(uint32_t)ALC_DET)
  {
	  LCD_SetBackColor(LCD_COLOR_GREEN);
	  LCD_DisplayStringLine(LCD_LINE_5, (uint8_t*)MESSAGE7);
  }
  else{
	  LCD_SetBackColor(LCD_COLOR_RED);
	  LCD_DisplayStringLine(LCD_LINE_5, (uint8_t*)MESSAGE6);
  }
  sprintf((char*)aTextBuffer, MESSAGE5, ADC_value);
  LCD_DisplayStringLine(LCD_LINE_6, (uint8_t*)aTextBuffer);
}


static void Display_Init(void)
{
  /* Initialize the LCD */
  LCD_Init();
  LCD_LayerInit();
  /* Eable the LTDC */
  LTDC_Cmd(ENABLE);

  /* Set LCD Background Layer  */
  LCD_SetLayer(LCD_BACKGROUND_LAYER);

  /* Clear the Background Layer */
  LCD_Clear(LCD_COLOR_WHITE);

  /* Configure the transparency for background */
  LCD_SetTransparency(0);

  /* Set LCD Foreground Layer  */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);

  /* Configure the transparency for foreground */
  LCD_SetTransparency(200);

  /* Clear the Foreground Layer */
  LCD_Clear(LCD_COLOR_WHITE);

  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(LCD_COLOR_BLUE);
  LCD_SetTextColor(LCD_COLOR_WHITE);

    /* Set the LCD Text size */
  LCD_SetFont(&FONTSIZE);

  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(LCD_COLOR_BLUE);
  LCD_SetTextColor(LCD_COLOR_WHITE);


  /* Set the LCD Text size */
  LCD_SetFont(&Font16x24);

  LCD_DisplayStringLine(LCD_LINE_0, (uint8_t*)MESSAGE3);
  LCD_DisplayStringLine(LCD_LINE_1, (uint8_t*)MESSAGE4);

  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(LCD_COLOR_WHITE);
  LCD_SetTextColor(LCD_COLOR_BLUE);
}


static void ADC_CH_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);



  /* Configure ADC3 Channel13 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel13 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, DISABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, DISABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}
