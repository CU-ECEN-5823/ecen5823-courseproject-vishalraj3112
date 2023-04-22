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

#if LOWEST_ENERGY_MODE == 3
    #define CNT_PER_MS    1
#else
    #define CNT_PER_MS    8.192
#endif

static uint32_t counter_ms = 0;

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

