/***********************************************************************
 * @file      timers.c
 * @version   0.1
 * @brief     Function implementation file.
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
#include "timers.h"
#include "app.h"
#include "gpio.h"

#define INCLUDE_LOG_DEBUG   0
#include "src/log.h"

#define ACTUAL_CLK_FREQ        OSC_FREQ/PRESCALER_VAL   //8192 Hz, 1000Hz
//#define VAL_TO_LOAD_COMP1     (LETIMER_PERIOD_COMP1_MS*ACTUAL_CLK_FREQ)/1000
#define VAL_TO_LOAD_UF        (LETIMER_PERIOD_UF_MS*ACTUAL_CLK_FREQ)/1000

#if LOWEST_ENERGY_MODE == 3
    #define CNT_PER_US    1
    #define MIN_WAIT_US   ACTUAL_CLK_FREQ
#else
    #define CNT_PER_US    8.192
    #define MIN_WAIT_US   ACTUAL_CLK_FREQ
#endif

// ---------------------------------------------------------------------
// Public function
// This function is used to initialize the LETIMER0 and configure it.
// @param None
// Returns None
// ---------------------------------------------------------------------
void init_LETIMER0(){

  //Timer0 configuration:
  const LETIMER_Init_TypeDef letimerStruct = {
      false,              //dont load comp1 in comp2.
      true,               //load comp1 in cnt register.
      true,               //continue running clock when in debug.
      false,              //dont start counting when init completed.
      0,                  //set out0Pol to 0,
      0,                  //set out1Pol to 1,
      letimerUFOANone,    //Continue counting until stopped !!This feild position different from API docs!!
      letimerUFOANone,    //Underflow output 0 action.
      letimerRepeatFree,    //Underflow output 1 action.
      0
  };

  //Init the timer with the struct
  LETIMER_Init(LETIMER0, &letimerStruct);

  //Clear all LETIMER0 Interrupts
  LETIMER_IntClear(LETIMER0, 0xFFFFFFFF);

  //Enable UF and COMP1 bits in IEN register.
  uint32_t flags = LETIMER_IEN_UF;
  LETIMER_IntEnable(LETIMER0, flags);

  //Load the comp0 timer value
  LETIMER_CompareSet(LETIMER0, 0, VAL_TO_LOAD_UF);

  LETIMER0->CTRL |= LETIMER_CTRL_COMP0TOP;

  //Read the loaded COMP0 value to verify
  flags = LETIMER_CompareGet(LETIMER0, 0);

  //finally enable the LETIMER0
  LETIMER_Enable(LETIMER0, true);

}

// ---------------------------------------------------------------------
// Public function
// This function is used to create a blocking delay with minimum resolution
// of 1ms and max of 65535ms.
// @param us delay to wait
// Returns None
// ---------------------------------------------------------------------
void timerWaitUs_polled(uint32_t us_wait){

  uint32_t curr_tick = LETIMER_CounterGet(LETIMER0);
  uint32_t wait_ticks = (us_wait * CNT_PER_US) / 1000;
  uint16_t delay_tick;

  //range checking the input
  if(wait_ticks > UINT16_MAX || us_wait < MIN_WAIT_US){
    us_wait = UINT16_MAX;
    LOG_WARN("Provided delay is out of range!\r\n");
  }

  wait_ticks = (us_wait * CNT_PER_US) / 1000;

  //Normal case
  delay_tick = curr_tick - wait_ticks;
  //Wrap around case
  if(delay_tick > VAL_TO_LOAD_UF)
    delay_tick = VAL_TO_LOAD_UF - (wait_ticks - curr_tick);

  while(LETIMER_CounterGet(LETIMER0) != delay_tick){

  }
}

// ---------------------------------------------------------------------
// Public function
// This function is used to create a non- blocking Interrupt based delay
// with minimum resolution of 1ms and max of 65535ms, can work in
// all EM modes. Uses COMP1 interrupt to generate delay.
// @param us delay to wait
// Returns None
// ---------------------------------------------------------------------
void timerWaitUs_irq(uint32_t us_wait){

  uint32_t curr_tick = LETIMER_CounterGet(LETIMER0);
  uint32_t wait_ticks = (us_wait * CNT_PER_US) / 1000;
  uint16_t delay_tick;

  //range checking the input
  if(wait_ticks > UINT16_MAX || us_wait < MIN_WAIT_US){
    us_wait = UINT16_MAX;
    LOG_WARN("Provided delay is out of range!\r\n");
  }

  wait_ticks = (us_wait * CNT_PER_US) / 1000;

  //Normal case
  delay_tick = curr_tick - wait_ticks;
  //Wrap around case
  if(delay_tick > VAL_TO_LOAD_UF)
    delay_tick = VAL_TO_LOAD_UF - (wait_ticks - curr_tick);

  //Load the COMP1 value
  LETIMER_CompareSet(LETIMER0, 1, delay_tick);
  //Enable the COMP1 interrupt
  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);

}

// ---------------------------------------------------------------------
// Public function
// This function is used to unit test timerWaitUs.
// @param us delay to wait
// Returns None
// ---------------------------------------------------------------------
void timerWaitUs_test(){

  //Unit test for timerWaitUs_polled()
  timerWaitUs_polled(70000000);//over range value
  timerWaitUs_polled(999);//under range value
  gpioLed0SetOn();
  timerWaitUs_polled(500000);
  gpioLed0SetOff();
  timerWaitUs_polled(500000);
  gpioLed0SetOn();
  timerWaitUs_polled(500000);
  gpioLed0SetOff();
  timerWaitUs_polled(500000);

}
