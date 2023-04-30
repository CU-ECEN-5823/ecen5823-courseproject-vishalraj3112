/***********************************************************************
 * @file      irq.c
 * @version   0.1
 * @brief     Function header/interface file.
 *
 * @author    Vishal Raj, vishal.raj@colorado.edu
 * @date      Jan 30, 2023
 *
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware
 * @instructor  David Sluiter
 *
 * @assignment Assignment 4 - Si7021 and Load Power Management P2
 * @due
 *
 * @resources Lecture Slides, EFR32xG13 Reference Manual
 *
 */

#include "em_letimer.h"
#include "irq.h"
#include "src/gpio.h"
#include "scheduler.h"
#include "log.h"
#include "app.h"
#include "i2c.h"

#define PERIOD_3_SEC      3000
#define GPIO_INT_PB0      (1 << PB0_pin)
#define GPIO_INT_PB1      (1 << PB1_pin)

#if LOWEST_ENERGY_MODE == 3
    #define CNT_PER_MS    1
#else
    #define CNT_PER_MS    8.192
#endif

static uint32_t counter_ms = 0;
static uint32_t pb0_val = 1; //Pulled-up
static uint32_t pb1_val = 1; //Pulled-up

// ---------------------------------------------------------------------
// Private ISR function
// This function is the LETIMER0 ISR, used to perform ISR actions
// @param None
// Returns None
// ---------------------------------------------------------------------
void LETIMER0_IRQHandler(void){

  //get the flags set
  uint32_t flags = LETIMER_IntGetEnabled(LETIMER0);

  //clear the flags set
  LETIMER_IntClear(LETIMER0, flags);

  //Every 3000ms
  if(flags & LETIMER_IF_UF){
      //Increment ms counter every 1ms
      counter_ms++;
      schedulerSetEventUF();
      schedulerSetEventUFConf();
  }

  if(flags & LETIMER_IF_COMP1){
      schedulerSetEventComp1();
      schedulerSetEventComp1Conf();
      /*Disable COMP1 Interrupt*/
      //LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
  }


}// LETIMER0_IRQHandler()

// ---------------------------------------------------------------------
// Private ISR function
// This function is the I2C0 ISR, used to perform ISR related actions
// @param None
// Returns None
// ---------------------------------------------------------------------
void I2C0_IRQHandler(void){

  I2C_TransferReturn_TypeDef transferStatus;

  transferStatus = I2C_Transfer(I2C0);

  if(transferStatus == i2cTransferDone){
      schedulerSetI2CTransferComp();
  }

  if(transferStatus < 0){
      LOG_ERROR("I2C_Transfer: Write error = %d\r\n", transferStatus);
  }

}// I2C0_IRQHandler()

// ---------------------------------------------------------------------
// Public function
// This function is called from log function to get the current time
// in ms since program startup.
// @param None
// Returns time since start-up in ms.
// ---------------------------------------------------------------------
uint32_t letimerMilliseconds(void){

  uint32_t ret_val = 0;

  ret_val = (counter_ms*PERIOD_3_SEC) + (LETIMER_CounterGet(LETIMER0)/CNT_PER_MS);

  return ret_val;
}// letimerMilliseconds()

// ---------------------------------------------------------------------
// Private ISR function
// This function is the GPIO PB0 ISR, triggered when PB0 is pressed.
// @param None
// Returns None
// ---------------------------------------------------------------------
void GPIO_EVEN_IRQHandler(){

  //Determine IRQ source
  uint32_t flags = GPIO_IntGetEnabled();

  pb0_val = GPIO_PinInGet(PB0_port, PB0_pin);

  //Clear the interrupt flag
  GPIO_IntClear(flags);

  if(flags & GPIO_INT_PB0) {

      schedulerSetEventPB0();

  }

}

// ---------------------------------------------------------------------
// Private ISR function
// This function is the GPIO PB1 ISR, triggered when PB1 is pressed.
// @param None
// Returns None
// ---------------------------------------------------------------------
void GPIO_ODD_IRQHandler(){

  //Determine IRQ source
  uint32_t flags = GPIO_IntGetEnabled();

  pb1_val = GPIO_PinInGet(PB1_port, PB1_pin);

  //Clear the interrupt flag
  GPIO_IntClear(flags);

  if(flags & GPIO_INT_PB1) {

      schedulerSetEventPB1();

  }

}

// ---------------------------------------------------------------------
// Public function
// This function is used to get the PB0 button current status
// @param None
// Returns true - button pressed, false - button released
// ---------------------------------------------------------------------
bool get_pbo_val(){

  if(pb0_val == 0){
      return true;
  }else if(pb0_val == 1){
      return false;
  }

  //to avoid warnings
  return false;
}

// ---------------------------------------------------------------------
// Public function
// This function is used to get the PB1 button current status
// @param None
// Returns true - button pressed, false - button released
// ---------------------------------------------------------------------
bool get_pb1_val(){

  if(pb1_val == 0){
      return true;
  }else if(pb1_val == 1){
      return false;
  }

  //to avoid warnings
  return false;
}

