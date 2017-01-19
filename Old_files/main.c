/*******************************************************************************
*
* E M B E D D E D   W I Z A R D   P R O J E C T
*
*                                                Copyright (c) TARA Systems GmbH
*                                    written by Paul Banach and Manfred Schweyer
*
********************************************************************************
*
* This software is delivered "as is" and shows the usage of other software 
* components. It is provided as an example software which is intended to be 
* modified and extended according to particular requirements.
* 
* TARA Systems hereby disclaims all warranties and conditions with regard to the
* software, including all implied warranties and conditions of merchantability 
* and non-infringement of any third party IPR or other rights which may result 
* from the use or the inability to use the software.
* 
********************************************************************************
*
* DESCRIPTION:
*   This file implements a generic main() function for running Embedded Wizard
*   generated applications on the STM32Fx environment. The main()
*   function initializes the Runtime Environment, creates an instance of the
*   application class and drives the message translation and screen updates.
*
*   In order to keep the main() function independent from the particular GUI 
*   application, the application class and the screen size are taken from the
*   generated code. In this manner, it is not necessary to modify this file
*   when creating new GUI applications. Just set the attributes 'ScreenSize'
*   and 'ApplicationClass' of the profile in the Embedded Wizard IDE.
*
*   This program demonstrates how to integrate an application developed using
*   Chora and Mosaic class library on a STM32Fx target.
*
*******************************************************************************/
#ifdef __cplusplus
extern "C"
#endif
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"
#include <stdio.h>

#include "xprintf.h"
#include "tlsf.h"

#include "ewrte.h"
#include "ewgfx.h"
#include "ewextgfx.h"
#include "ewgfxdefs.h"
#include "Core.h"
#include "Graphics.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_adc.h>


#include "BoardConfig.h"

tlsf_t MemPool;

/* Defines for the pyhiscal dimension and color format of the LCD framebuffer */
#define FRAME_BUFFER_WIDTH    240
#define FRAME_BUFFER_HEIGHT   320

/* Calculated addresses for framebuffer(s) and memory manager */
#define FRAME_BUFFER_SIZE     FRAME_BUFFER_WIDTH * FRAME_BUFFER_HEIGHT * FRAME_BUFFER_DEPTH
#define FRAME_BUFFER_ADDR     (void*)(SDRAM_DEVICE_ADDR)

#ifdef EW_USE_DOUBLE_BUFFER
  #define DOUBLE_BUFFER_ADDR  (void*)(FRAME_BUFFER_ADDR + FRAME_BUFFER_SIZE)
  #define DOUBLE_BUFFER_SIZE  FRAME_BUFFER_SIZE
#else  
  #define DOUBLE_BUFFER_ADDR  (void*)(0)
  #define DOUBLE_BUFFER_SIZE  0
#endif

#define MEMORY_POOL_ADDR      (void*)((unsigned char *)FRAME_BUFFER_ADDR + FRAME_BUFFER_SIZE + DOUBLE_BUFFER_SIZE)
#define MEMORY_POOL_SIZE      0x800000 - FRAME_BUFFER_SIZE - DOUBLE_BUFFER_SIZE 

#undef USE_TERMINAL_INPUT


/*******************************************************************************
* FUNCTION:
*   TermGetCode
*
* DESCRIPTION:
*   The function TermGetCode reads the next EmWi key code from the console.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   Returns the next EmWi key code or CoreKeyCodeNoKey if no key code available.
*
*******************************************************************************/
#ifdef USE_TERMINAL_INPUT
static XEnum TermGetCode( void )
{
  switch ( UartGetCharacter())
  {
    case 0x65 : xputs("Key 'Exit' pressed\n");  return CoreKeyCodeExit;
    case 0x38 : xputs("Key 'Up' pressed\n");    return CoreKeyCodeUp;
    case 0x32 : xputs("Key 'Down' pressed\n");  return CoreKeyCodeDown;
    case 0x36 : xputs("Key 'Right' pressed\n"); return CoreKeyCodeRight;
    case 0x34 : xputs("Key 'Left' pressed\n");  return CoreKeyCodeLeft;
    case 0x35 : xputs("Key 'OK' pressed\n");    return CoreKeyCodeOk;
    case 0x6D : xputs("Key 'Menu' pressed\n");  return CoreKeyCodeMenu;
    case 0x70 : xputs("Key 'Power' pressed\n"); return CoreKeyCodePower;
  }
  return CoreKeyCodeNoKey;
}
#endif


/*******************************************************************************
* FUNCTION:
*   Update
*
* DESCRIPTION:
*   The function Update performs the screen update of the dirty area.
*
* ARGUMENTS:
*   aViewPort    - Viewport used for the screen update.
*   aApplication - Root object used for the screen update.
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
static void Update( XViewport* aViewport, CoreRoot aApplication )
{
  XBitmap*       bitmap     = EwBeginUpdate( aViewport );
  GraphicsCanvas canvas     = EwNewObject( GraphicsCanvas, 0 );
  XRect          updateRect = {{ 0, 0 }, { 0, 0 }};

  /* let's redraw the dirty area of the screen. Cover the returned bitmap 
     objects within a canvas, so Mosaic can draw to it. */
  if ( bitmap && canvas )
  {
    GraphicsCanvas__AttachBitmap( canvas, (XUInt32)bitmap );
    updateRect = CoreRoot__UpdateGE20( aApplication, canvas );
    GraphicsCanvas__DetachBitmap( canvas );
  }

  /* complete the update */
  if ( bitmap )
    EwEndUpdate( aViewport, updateRect );
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
		
	}
}
/*******************************************************************************
* FUNCTION:
*   main
*
* DESCRIPTION:
*   The main function of the whole application. The main function initializes all
*   necessary drivers and provides a for-ever loop to drive the EmWi application.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   Zero if successful.
*
*******************************************************************************/
 uint32_t g_ADCValue;
int main( void ) 
{
 
  CoreRoot   rootObject;
  XViewport* viewport;
  XEnum      cmd = CoreKeyCodeNoKey;

  int        touched = 0;
  XPoint     touchPos;

  HAL_Init();

  SystemClock_Config();
  ConfigureADC();
  BSP_SDRAM_Init();
  HAL_ADC_Start(&g_AdcHandle);

  /* Configure LEDs */
  BSP_LED_Init( LED4 );

  /* configure system tick counter */
  SystemTick_Config();

  /* initialize UART for debugging and connect xprintf module */
  InitUart();
  xdev_out( UartPutCharacter );

  /* initialize LCD */
  xputs( "Initialize LCD...                            " );
  LCD_Config( FRAME_BUFFER_WIDTH, FRAME_BUFFER_HEIGHT, FRAME_BUFFER_ADDR );
  xputs( "[OK]\n" );

  /* initialize touchscreen */
  Touch_Config( FRAME_BUFFER_WIDTH, FRAME_BUFFER_HEIGHT );

  /* initialize tlsf memory manager */
  /* please note, that the first part of SDRAM is reserved for framebuffer */
  xputs( "Initialize Memory Manager...                 " );
  MemPool = tlsf_create_with_pool( MEMORY_POOL_ADDR, MEMORY_POOL_SIZE );
  xputs( "[OK]\n" );
  
  EwPrint("MemoryPool at address 0x%08X size 0x%08X\n", MEMORY_POOL_ADDR, MEMORY_POOL_SIZE );

  /* initialize the Graphics Engine and Runtime Environment */
  xputs( "Initialize Graphics Engine...                " );
  if ( !EwInitGraphicsEngine( 0 ))
    return 2;
  xputs( "[OK]\n" );

  /* create the applications root object ... */
  xputs( "Create EmWi Root Object...                   " );
  rootObject = (CoreRoot)EwNewObjectIndirect( EwApplicationClass, 0 );
  EwLockObject( rootObject );
  CoreRoot__Initialize( rootObject, EwScreenSize );
  xputs( "[OK]\n" );

  /* create Embedded Wizard viewport object to provide uniform access to the
     framebuffer */
  xputs( "Create EmWi Viewport...                      " );
  viewport = EwInitViewport( EwScreenSize, EwNewRect( 0, 0, FRAME_BUFFER_WIDTH, FRAME_BUFFER_HEIGHT ), 
    0, 255, FRAME_BUFFER_ADDR, DOUBLE_BUFFER_ADDR, 0, 0 );  
  xputs( "[OK]\n" );

  /* start the EmWi main loop and process all user inputs, timers and signals */
  while( cmd != CoreKeyCodePower )
  {
    int timers  = 0;
    int signals = 0;
    int events  = 0;
    
	if (HAL_ADC_PollForConversion(&g_AdcHandle, 1000000) == HAL_OK)
			{
				g_ADCValue = HAL_ADC_GetValue(&g_AdcHandle);
				
			}
			
    /* receive keyboard inputs and provide the application with them */
    #ifdef USE_TERMINAL_INPUT
      cmd = TermGetCode();
      if ( cmd != CoreKeyCodeNoKey )
      {
        CoreKeyEvent event = EwNewObject( CoreKeyEvent, 0 );
        CoreKeyEvent__Initialize( event, cmd, 1 );
        CoreGroup__DispatchEvent( rootObject, (CoreEvent)event );
        events = 1;
      }
    #endif

    /* receive touch inputs and provide the application with them */
    if ( GetTouchPosition( &touchPos ))
    {
      /* begin of touch cycle */
      if ( touched == 0 )
        CoreRoot__DriveCursorHitting( rootObject, 1, 0, touchPos );

      /* movement during touch cycle */
      else if ( touched == 1 )
        CoreRoot__DriveCursorMovement( rootObject, touchPos );

      touched = 1;
      events  = 1;
    }
    /* end of touch cycle */
    else if ( touched == 1 )
    {
      CoreRoot__DriveCursorHitting( rootObject, 0, 0, touchPos );
      touched = 0;
      events  = 1;
    }

    /* process expired timers */
    timers = EwProcessTimers();

    /* process the pending signals */
    signals = EwProcessSignals();

    /* refresh the screen, if something has changed and draw its content */
    if ( timers || signals || events )
    {
      BSP_LED_On( LED4 );
      Update( viewport, rootObject );
      BSP_LED_Off( LED4 );

      /* after each processed message start the garbage collection */
      EwReclaimMemory();

      /* show the memory statistic */
      #ifdef EW_PRINT_MEMORY_USAGE
        EwPrintProfilerStatistic( 0 );
      #endif
    }
  }

  /* finished -> release unused resources and memory */
  xputs( "Shutting down EmWi application...            " );
  EwDoneViewport( viewport );
  EwUnlockObject( rootObject );
  EwReclaimMemory();

  /* ... and deinitialize the Graphics Engine */
  EwDoneGraphicsEngine();

  xputs( "[OK]\n" );
  xputs( "\n\n*************************************************\n" );

  tlsf_destroy( MemPool );

  /* deinitialize UART for debugging */
  DoneUart();

  return 0;
}  


/* msy */