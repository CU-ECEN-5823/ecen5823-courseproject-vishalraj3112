/***********************************************************************
 * @file      scheduler.h
 * @version   0.1
 * @brief     Function header/interface file.
 *
 * @author    Vishal Raj, vishal.raj@colorado.edu
 * @date      Feb 7, 2023
 *
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware
 * @instructor  David Sluiter
 *
 * @assignment Assignment 4 - Si7021 and Load Power Management P2
 * @due
 *
 * @resources None
 *
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_
#include <stdint.h>
#include "sl_bt_api.h"

// Macros
//#define IDLE_EVENT                 0
//#define EVENT_LETIMER_UF           (1 << 0)
//#define EVENT_LETIMER_COMP1        (1 << 1)
//#define EVENT_I2C_TRANSFER_COMP    (1 << 2)

typedef enum {
  IDLE_EVENT,
  EVENT_LETIMER_UF,
  EVENT_LETIMER_COMP1,
  EVENT_I2C_TRANSFER_COMP,
  EVENT_CONNECTION_LOST
}events_t;

// Function prototypes
uint32_t getNextEvent();
void tempReadStateMachine();
void schedulerSetEventUF();
void schedulerSetEventComp1();
void schedulerSetI2CTransferComp();
void schedulerSetTempMeasureEvent();
void schedulerSetConnectionLostEvent();
void temperature_state_machine(sl_bt_msg_t *evt);
void discovery_state_machine(sl_bt_msg_t *evt);
void max_hub_read(sl_bt_msg_t *evt);
void max_hub_read_polled(sl_bt_msg_t *evt);

#endif /* SRC_SCHEDULER_H_ */
