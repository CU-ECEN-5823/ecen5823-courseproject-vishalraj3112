/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

 *
 * Student edit:
 * @student    Vishal Raj, vira8255@colorado.edu
 *
 
 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <string.h>

#include "gpio.h"


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#define LED0_port         gpioPortF
#define LED0_pin          4
#define LED1_port         gpioPortF
#define LED1_pin          5
#define SI7021_port       gpioPortD
#define SI7021_pin        15
#define SI7021_SDA_port   gpioPortC
#define SI7021_SDA_pin    11
#define SI7021_SCL_port   gpioPortC
#define SI7021_SCL_pin    10
#define LCD_port          gpioPortD
#define LCD_EXTCOMIN_pin  13


// Set GPIO drive strengths and modes of operation
void gpioInit()
{

  // Student Edit:

	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);//!!Confirm this naming
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	//Init Si7021 GPIO
	GPIO_PinModeSet(SI7021_port, SI7021_pin, gpioModePushPull, true);

	//Init LCD ExtComin Pin
	GPIO_PinModeSet(LCD_port, LCD_EXTCOMIN_pin, gpioModePushPull, false);

  //Set PB0 as input - Input enabled. Filter if DOUT is set
  GPIO_PinModeSet(PB0_port, PB0_pin, gpioModeInput, true);

  //Enable GPIO interrupt for PB0
  GPIO_ExtIntConfig(PB0_port, PB0_pin, PB0_pin, true, true, true);

  //Set PB1 as input - Input enabled. Filter if DOUT is set
  GPIO_PinModeSet(PB1_port, PB1_pin, gpioModeInput, true);

  //Enable GPIO interrupt for PB1
  GPIO_ExtIntConfig(PB1_port, PB1_pin, PB1_pin, true, true, true);

} // gpioInit()


void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

void gpioSi7021Enable()
{
  GPIO_PinOutSet(SI7021_port,SI7021_pin);
}

void gpioSi7021Disable()
{
  GPIO_PinOutClear(SI7021_port,SI7021_pin);
}

void gpioI2CSDADisable()
{
  GPIO_PinModeSet(SI7021_SDA_port, SI7021_SDA_pin, gpioModeDisabled, 1);
}

void gpioI2CSCLDisable()
{
  GPIO_PinModeSet(SI7021_SCL_port, SI7021_SCL_pin, gpioModePushPull, true);
}

void gpioSetDisplayExtcomin(bool curr_state)
{
  if(curr_state == true){
      GPIO_PinOutSet(LCD_port, LCD_EXTCOMIN_pin);
  }else{
      GPIO_PinOutClear(LCD_port, LCD_EXTCOMIN_pin);
  }

}
