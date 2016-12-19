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


/*******************************************************************************
* CALLBACK:
*   UserButtonCallback
*
* DESCRIPTION:
*   A function of this type has to be set with UserButton_Config() to get 
*   notified, everytime the USER button is pressed or released.
*
* ARGUMENTS:
*   aButtonPresssed
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
typedef void ( * TUserButtonCallback ) ( int aButtonPresssed );


/*******************************************************************************
* FUNCTION:
*   SystemClock_Config
*
* DESCRIPTION:
*   System Clock Configuration
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void SystemClock_Config( void );


/*******************************************************************************
* FUNCTION:
*   MPU_Config
*
* DESCRIPTION:
*   Configure the MPU attributes as Write Through for SRAM1/2.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
void MPU_Config( void );


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
void LCD_Config( int aWidth, int aHeight, void* aAddress );


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
void SystemTick_Config( void );


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
void UserButton_Config( TUserButtonCallback aUserButtonCallback );


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
void Touch_Config( int aWidth, int aHeight );


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
int GetTouchPosition( XPoint* aPos );


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
void InitUart
(
  void
);


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
void DoneUart
(
  void
);


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
void UartPutCharacter
(
  unsigned char aCharacter
);


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
unsigned char UartGetCharacter
(
  void
);


/* msy */