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
*   This file contains the necessary initialization of the STM32Fx environment
*   used to execute Embedded Wizard generated UI applications on a dedicated
*   platform. The main() function calls the different initializations of
*   system clock, memory, LCD and UART.
*
*   This program demonstrates how to integrate an application developed using
*   Chora and Mosaic class library on a STM32Fx target.
*
*******************************************************************************/

#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_io.h"
#include "stm32f429i_discovery_ts.h"
#include "../Components/ili9341/ili9341.h"   

#include <stdio.h>

#include "xprintf.h"

#include "ewrte.h"
#include "ewgfx.h"
#include "ewextgfx.h"
#include "ewgfxdefs.h"

#include "BoardConfig.h"

#define UART_PORTID           USART1

#define LAYER_INDEX           0

unsigned int                  EmWiSystemTicks = 0;

static int                    TouchAreaWidth  = 0;
static int                    TouchAreaHeight = 0;

UART_HandleTypeDef            UART_Handle;

LTDC_HandleTypeDef            LtdcHandle;
LTDC_LayerCfgTypeDef          LayerConfig;

#ifdef EW_USE_DOUBLE_BUFFER
  static volatile unsigned long CurrentFramebuffer = 0;
  static volatile unsigned long PendingFramebuffer = 0;
#endif

static TUserButtonCallback    UserButtonCallback = NULL;


/*******************************************************************************
* FUNCTION:
*   SystemClock_Config
*
* DESCRIPTION:
*   System Clock Configuration
*     The system Clock is configured as follow : 
*     System Clock source            = PLL (HSE)
*     SYSCLK(Hz)                     = 168000000
*     HCLK(Hz)                       = 168000000
*     AHB Prescaler                  = 1
*     APB1 Prescaler                 = 4
*     APB2 Prescaler                 = 2
*     HSE Frequency(Hz)              = 8000000
*     PLL_M                          = 8
*     PLL_N                          = 336
*     PLL_P                          = 2
*     PLL_Q                          = 7
*     VDD(V)                         = 3.3
*     Main regulator output voltage  = Scale1 mode
*     Flash Latency(WS)              = 5
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void SystemClock_Config( void )
{
  RCC_ClkInitTypeDef        RCC_ClkInitStruct;
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;
  RCC_OscInitStruct.PLL.PLLN       = 360;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* LCD clock configuration */
  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 MHz */
  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 MHz */
  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/4 = 48 MHz */
  /* LTDC clock frequency = PLLLCDCLK / RCC_PLLSAIDIVR_8 = 48/8 = 6 MHz */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
  PeriphClkInitStruct.PLLSAIDivR     = RCC_PLLSAIDIVR_8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct); 
}


/*******************************************************************************
* FUNCTION:
*   LCD_Config
*
* DESCRIPTION:
*   Configure the LCD.
*
* ARGUMENTS:
*   aWidth   - Width of the framebuffer in pixel.
*   aHeight  - Height of the framebuffer in pixel.
*   aAddress - Startaddress of the framebuffer.
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void LCD_Config( int aWidth, int aHeight, void* aAddress )
{
  /* Initialization of ILI9341 component*/
  ili9341_Init();

  /* Polarity configuration */
  /* Initialize the horizontal synchronization polarity as active low */
  LtdcHandle.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  /* Initialize the vertical synchronization polarity as active low */
  LtdcHandle.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  /* Initialize the data enable polarity as active low */
  LtdcHandle.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  /* Initialize the pixel clock polarity as input pixel clock */
  LtdcHandle.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

  /* Timing configuration */
  /* Horizontal synchronization width = Hsync - 1 */
  LtdcHandle.Init.HorizontalSync = 9;
  /* Vertical synchronization height = Vsync - 1 */
  LtdcHandle.Init.VerticalSync = 1;
  /* Accumulated horizontal back porch = Hsync + HBP - 1 */
  LtdcHandle.Init.AccumulatedHBP = 29;
  /* Accumulated vertical back porch = Vsync + VBP - 1 */
  LtdcHandle.Init.AccumulatedVBP = 3;
  /* Accumulated active width = Hsync + HBP + Active Width - 1 */
  LtdcHandle.Init.AccumulatedActiveH = 323;
  /* Accumulated active height = Vsync + VBP + Active Heigh - 1 */
  LtdcHandle.Init.AccumulatedActiveW = 269;
  /* Total height = Vsync + VBP + Active Heigh + VFP - 1 */
  LtdcHandle.Init.TotalHeigh = 327;
  /* Total width = Hsync + HBP + Active Width + HFP - 1 */
  LtdcHandle.Init.TotalWidth = 279;

  /* Configure R,G,B component values for LCD background color */
  LtdcHandle.Init.Backcolor.Blue = 0;
  LtdcHandle.Init.Backcolor.Green = 0;
  LtdcHandle.Init.Backcolor.Red = 0;

  LtdcHandle.Instance = LTDC;

  /* Layer1 Configuration */
  LayerConfig.WindowX0 = 0;
  LayerConfig.WindowX1 = aWidth;
  LayerConfig.WindowY0 = 0;
  LayerConfig.WindowY1 = aHeight;
  
  /* Pixel Format configuration: translate framebuffer color format into LTDC mode */ 
  #if ( EW_FRAME_BUFFER_COLOR_FORMAT_RGBA8888 )
    LayerConfig.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  #elif ( EW_FRAME_BUFFER_COLOR_FORMAT_RGB888 )
    LayerConfig.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  #elif ( EW_FRAME_BUFFER_COLOR_FORMAT_RGBA4444 )
    LayerConfig.PixelFormat = LTDC_PIXEL_FORMAT_ARGB4444;
  #elif ( EW_FRAME_BUFFER_COLOR_FORMAT_RGB565 )
    LayerConfig.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  #elif ( EW_FRAME_BUFFER_COLOR_FORMAT_Index8 )
    LayerConfig.PixelFormat = LTDC_PIXEL_FORMAT_L8;
  #elif ( EW_FRAME_BUFFER_COLOR_FORMAT_LumA44 )
    LayerConfig.PixelFormat = LTDC_PIXEL_FORMAT_AL44;
  #endif  

  /* Start Address will be set by Graphics Engine */
  LayerConfig.FBStartAdress = (uint32_t)(aAddress);
  
  /* Alpha constant (255 totally opaque) */
  LayerConfig.Alpha = 255;
  
  /* Default Color configuration (configure A,R,G,B component values) */
  LayerConfig.Alpha0 = 0;
  LayerConfig.Backcolor.Blue = 0;
  LayerConfig.Backcolor.Green = 0;
  LayerConfig.Backcolor.Red = 0;
  
  /* Configure blending factors */
  LayerConfig.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  LayerConfig.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  
  /* Configure the number of lines and number of pixels per line */
  LayerConfig.ImageWidth = aWidth;
  LayerConfig.ImageHeight = aHeight;  

  /* Configure the LTDC */  
  if ( HAL_LTDC_Init( &LtdcHandle ) != HAL_OK )
    xputs( "Could not configure LTDC!\n" );

  /* Configure the Layer */
  if ( HAL_LTDC_ConfigLayer( &LtdcHandle, &LayerConfig, LAYER_INDEX ) != HAL_OK )
    xputs( "Could not configure layer!\n" );

  #if EW_USE_DOUBLE_BUFFER  

    /* Configure the vsync */
    if ( HAL_LTDC_ProgramLineEvent( &LtdcHandle, 0 ) != HAL_OK )
      xputs( "Could not configure interrupt for vsync!\n" );

  #endif
}


/*******************************************************************************
* FUNCTION:
*   SystemTick_Config
*
* DESCRIPTION:
*   Configure the system tick counter.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void SystemTick_Config( void )
{
  EmWiSystemTicks = 0;
  HAL_SYSTICK_Config( SystemCoreClock / 1000 );
}


/*******************************************************************************
* FUNCTION:
*   EmWiSetFramebufferAddress
*
* DESCRIPTION:
*   The function EmWiSetFramebufferAddress is called from the Graphics Engine
*   in order to change the currently active framebuffer address. If the display
*   is running in a double-buffering mode, the function is called after each
*   screen update. 
*   Changing the framebuffer address should be synchronized with V-sync.
*   In case of double-buffering, the function has to wait and return after 
*   the V-sync was detected.
*
* ARGUMENTS:
*   aAddress - New address of the framebuffer to be shown on the LCD.
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void EmWiSetFramebufferAddress( unsigned long aAddress )
{
#ifdef EW_USE_DOUBLE_BUFFER  

  /* set pending framebuffer address to be used on next V-sync */
  PendingFramebuffer = aAddress;
  
  /* now, the function has to wait until the pending back-buffer is visible -
     otherwise the currently visible frontbuffer may be overwritten with next 
     scene. */
  /* ==> ToDo: In case your system uses an operating system, please use a 
     while(...) sleep() statement, to avoid the high CPU usage */   
  while( CurrentFramebuffer != PendingFramebuffer )
    HAL_Delay( 1 ); 

#else
 
  /* change address within layer configuration structure */
  LayerConfig.FBStartAdress = (uint32_t)(aAddress);

  /* change layer configuration */
  if ( HAL_LTDC_ConfigLayer( &LtdcHandle, &LayerConfig, LAYER_INDEX ) != HAL_OK )
    xputs( "Could not change layer configuration!\n" );
    
#endif    
}


/*******************************************************************************
* FUNCTION:
*   HAL_LTDC_LineEventCallback
*
* DESCRIPTION:
*   Interrupt service routine for V-sync.
*
* ARGUMENTS:
*   aLtdc - Specifies the pins connected EXTI line
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
#if EW_USE_DOUBLE_BUFFER 
void HAL_LTDC_LineEventCallback( LTDC_HandleTypeDef* aLtdc )
{
  if ( CurrentFramebuffer != PendingFramebuffer )
  {  
    /* change address within layer configuration structure */
    LayerConfig.FBStartAdress = (uint32_t)(PendingFramebuffer);

    /* change layer configuration */
    if ( HAL_LTDC_ConfigLayer( &LtdcHandle, &LayerConfig, LAYER_INDEX ) != HAL_OK )
      xputs( "Could not change layer configuration!\n" );
      
    /* save new address */   
    CurrentFramebuffer = PendingFramebuffer;
  }    

  /* program next V-ync */
  HAL_LTDC_ProgramLineEvent( aLtdc, 0 );
}
#endif


/*******************************************************************************
* FUNCTION:
*   EmWiSetFramebufferClut
*
* DESCRIPTION:
*   The function EmWiSetFramebufferClut is called from the Graphics Engine
*   in order to update the hardware CLUT of the current framebuffer. 
*   The function is only called when the color format of the framebuffer is
*   Index8 or LumA44. 
*
* ARGUMENTS:
*   aClut - Pointer to a color lookup table with 256 enries.
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void EmWiSetFramebufferClut( unsigned long* aClut )
{
  if ( HAL_LTDC_ConfigCLUT( &LtdcHandle, aClut, 256, LAYER_INDEX ) != HAL_OK )
    xputs( "Could not configure layer CLUT!\n" );
  if ( HAL_LTDC_EnableCLUT( &LtdcHandle, LAYER_INDEX ) != HAL_OK )
    xputs( "Could not enable layer CLUT!\n" );
}


/*******************************************************************************
* FUNCTION:
*   EmWiSystemTickIncrement
*
* DESCRIPTION:
*   The function EmWiSystemTickIncrement increments the millisecond counter,
*   which is used by the Runtime Environmet (RTE) to trigger timer events.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void EmWiSystemTickIncrement( void )  
{
  EmWiSystemTicks++;
}


/*******************************************************************************
* FUNCTION:
*   UserButton_Config
*
* DESCRIPTION:
*   Configures the USER button.
*
* ARGUMENTS:
*   aUserButtonCallback - The USER button callback.
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void UserButton_Config( TUserButtonCallback aUserButtonCallback )
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init( GPIOA, &GPIO_InitStructure );

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority( EXTI0_IRQn, 2, 0 );
  HAL_NVIC_EnableIRQ( EXTI0_IRQn );

  UserButtonCallback = aUserButtonCallback;
}


/*******************************************************************************
* FUNCTION:
*   HAL_GPIO_EXTI_Callback
*
* DESCRIPTION:
*   The GPIO interrupt callback function.
*
* ARGUMENTS:
*   GPIO_Pin - the gpio pin.
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  if ( GPIO_Pin == KEY_BUTTON_PIN )
  {
    GPIO_PinState pinState;
    
    pinState = HAL_GPIO_ReadPin( KEY_BUTTON_GPIO_PORT, GPIO_Pin );
    
    if ( UserButtonCallback )
      UserButtonCallback( pinState );
  }
}


/*******************************************************************************
* FUNCTION:
*   Touch_Config
*
* DESCRIPTION:
*   Configure the touch driver.
*
* ARGUMENTS:
*   aWidth  - Width of the toucharea (framebuffer) in pixel.
*   aHeight - Height of the toucharea (framebuffer) in pixel.
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void Touch_Config( int aWidth, int aHeight )
{
  TouchAreaWidth  = aWidth;
  TouchAreaHeight = aHeight;
  
  BSP_TS_Init( aWidth, aHeight );
}


/*******************************************************************************
* FUNCTION:
*   GetTouchPosition
*
* DESCRIPTION:
*   The function GetTouchPosition reads the current touch position from the
*   touch driver and returns the current position and status. The orientation
*   of the touch positions is adjusted to match GUI coordinates.
*
* ARGUMENTS:
*   aPos - Pointer to XPoint structure to return the current position.
*
* RETURN VALUE:
*   Returns 1 if a touch event is detected, otherwise 0.
*
*******************************************************************************/
int GetTouchPosition( XPoint* aPos )
{
  static TS_StateTypeDef touchPadState;
  static XPoint          lastPos;
  static unsigned int    lastTime; 
  
  /* access touch driver to receive current touch status and position */
  BSP_TS_GetState( &touchPadState );
  if ( touchPadState.TouchDetected )
  {
    /* check for valid coordinates */
    if (( touchPadState.X > 0 ) && ( touchPadState.X < TouchAreaWidth - 1 ) &&
        ( touchPadState.Y > 0 ) && ( touchPadState.Y < TouchAreaHeight - 1 ))
    {    
      #if ( EW_SURFACE_ROTATION == 90 )
      
        aPos->X = touchPadState.Y;
        aPos->Y = TouchAreaWidth  - touchPadState.X;
      
      #elif ( EW_SURFACE_ROTATION == 270 )
      
        aPos->X = TouchAreaHeight - touchPadState.Y;
        aPos->Y = touchPadState.X;
      
      #elif ( EW_SURFACE_ROTATION == 180 )
      
        aPos->X = TouchAreaWidth  - touchPadState.X;
        aPos->Y = TouchAreaHeight - touchPadState.Y;
      
      #else  
      
        aPos->X = touchPadState.X;
        aPos->Y = touchPadState.Y;
      
      #endif  
      
      /* keep the position and time of last touch detection */
      lastPos.X = aPos->X;
      lastPos.Y = aPos->Y;
      lastTime  = EmWiSystemTicks;
      
      /* return valid touch event */
      return 1;
    }
  }
  
  /* in order to avoid that short undetected touch events will be interpreted 
     as an 'up' event - we return for some milliseconds the last detected event */
  if ( lastTime + 20 > EmWiSystemTicks )
  { 
    aPos->X = lastPos.X;
    aPos->Y = lastPos.Y;
    return 1;
  }
         
  /* return no touch event */
  return 0;
}


/*******************************************************************************
* FUNCTION:
*   InitUart
*
* DESCRIPTION:
*   The function InitUart initializes the STM32Fx UART interface used to print
*   error and trace messages from an Embedded Wizard GUI application.
*   This implementations uses the following configuration:
*   - BaudRate = 115200 baud  
*   - Word Length = 8 Bits
*   - One Stop Bit
*   - No parity
*   - Hardware flow control disabled (RTS and CTS signals)
*   - Receive and transmit enabled
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void InitUart()
{
  UART_Handle.Instance            = UART_PORTID;
  UART_Handle.Init.BaudRate       = 115200;
  UART_Handle.Init.WordLength     = UART_WORDLENGTH_8B;
  UART_Handle.Init.StopBits       = UART_STOPBITS_1;
  UART_Handle.Init.Parity         = UART_PARITY_NONE;
  UART_Handle.Init.Mode           = UART_MODE_TX_RX;
  UART_Handle.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  UART_Handle.Init.OverSampling   = UART_OVERSAMPLING_16;
  
  HAL_UART_Init( &UART_Handle );
}


/*******************************************************************************
* FUNCTION:
*   DoneUart
*
* DESCRIPTION:
*   The function DoneUart deinitializes the STM32Fx UART interface used to print
*   error and trace messages from an Embedded Wizard GUI application.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void DoneUart()
{
  HAL_UART_DeInit( &UART_Handle );
}


/*******************************************************************************
* FUNCTION:
*   UartPutCharacter
*
* DESCRIPTION:
*   The function UartPutCharacter sends the given character to the UART interface.
*
* ARGUMENTS:
*   aCharacter - The character to be send via the Uart interface.
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void UartPutCharacter( unsigned char aCharacter )
{
  HAL_UART_Transmit( &UART_Handle, (uint8_t*)&aCharacter, 1, 10 );
}


/*******************************************************************************
* FUNCTION:
*   UartGetCharacter
*
* DESCRIPTION:
*   The function UartGetCharacter retruns the current character from the UART.
*   If no character is available within the input buffer, zero is returned.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   Current character from UART input buffer or 0.
*
*******************************************************************************/
unsigned char UartGetCharacter()
{
  uint8_t ret;
  HAL_UART_Receive( &UART_Handle, &ret, 1, 10 );
  return ret;
}


/* msy */