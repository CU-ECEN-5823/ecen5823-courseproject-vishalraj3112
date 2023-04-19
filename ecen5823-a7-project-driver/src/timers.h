/***********************************************************************
 * @file      timers.h
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
 * @resources None
 *
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_


// Function prototypes
void init_LETIMER0();
void timerWaitUs_polled(uint32_t us_wait);
void timerWaitUs_irq(uint32_t us_wait);
void timerWaitUs_test();

#endif /* SRC_TIMERS_H_ */
