/***********************************************************************
 * @file      oscillator.c
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
 * @assignment Assignment 2 - Managing Energy Modes
 * @due
 *
 * @resources Lecture Slides, EFR32xG13 Reference Manual
 *
 */

#include "src/oscillator.h"
#include "em_cmu.h"
#include "app.h"

// ---------------------------------------------------------------------
// Public function
// This function is used to initialize the clock management unit module.
// @param None
// Returns None
// ---------------------------------------------------------------------
void CMU_init(){

#if LOWEST_ENERGY_MODE == 3 //Only for EM3 ULFRCO required
  //Enable Oscillator
  CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
  //Configure to peripharel clock branch
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

#else
  //Enable Oscillator
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  //Configure to peripharel clock branch
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

#endif

  //Configure the prescalar(/4)
  CMU_ClockDivSet(cmuClock_LETIMER0 , PRESCALER_VAL);

  //Enable the clock to LETIMER0 and LFA
  CMU_ClockEnable(cmuClock_LETIMER0, true);
  //CMU_ClockEnable(cmuClock_LFA, true); //!!Check if this is required.

//  uint32_t freq;
//  freq = CMU_ClockFreqGet(cmuClock_LFA);
//  freq = CMU_ClockFreqGet(cmuClock_LETIMER0);

}

