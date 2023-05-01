/***********************************************************************
 * @file      scheduler.c
 * @version   0.1
 * @brief     Function implementation file.
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

#define INCLUDE_LOG_DEBUG     1

#include "scheduler.h"
#include "em_cmu.h"
#include "i2c.h"
#include "log.h"
#include "timers.h"
#include "gpio.h"
#include "ble.h"
#include "lcd.h"
#include "ble_device_type.h"
#include "irq.h"
#include "max_3266.h"

#if DEVICE_IS_BLE_SERVER

static uint32_t myEvents = 0;

typedef enum{
  stateDevInit,
  stateDevSetMode,
  stateDevSetDone
}State_DevMode_t;

  typedef enum {
    stateIdle_max,
    stateSensorCheck,
//    set_data_type,
//    set_thresh,
//    enable_sh,
//    enable_algo,
//    enable_maximFast,
    start_sh_read,
//    invalid_state
  }State_max_t;

  typedef enum uint32_t {
    stateIdle,
    stateSensorOn,
    stateI2CWrite,
    stateI2CWait,
    stateI2CRead,
    stateGetResult,
  }State_t;

  typedef enum{
    TRIGGER_MODE,
    CONTINUOUS_MODE,
    NO_MODE
  }DeviceMode_t;

  typedef enum{
    stateTrigInit,
    stateTrigReadTemp,
    stateTrigReadMax,
    stateTrigRestart
  }State_TrigMode_t;

  typedef enum{
    stateContInit,
    stateContReadTemp,
    stateContReadMax
  }State_ContMode_t;

static DeviceMode_t dev_mode = NO_MODE;

static void trig_mode_state_machine(sl_bt_msg_t *evt);
static void cont_mode_state_machine(sl_bt_msg_t *evt);

#else
  typedef enum uint32_t {
      discoverService,
      discoverChar,
      setIndication,
      getIndication,
      startScanning,
    }State_t;

  // Health Thermometer service UUID defined by Bluetooth SIG
  static const uint8_t thermo_service[2] = { 0x09, 0x18 };
  // Temperature Measurement characteristic UUID defined by Bluetooth SIG
  static const uint8_t thermo_char[2] = { 0x1c, 0x2a };

#endif
// ---------------------------------------------------------------------
// Public function
// This function is used to set the LETIMER0 UF event
// @param None
// Returns None
// ---------------------------------------------------------------------
void schedulerSetEventUF(){

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  //myEvents |= EVENT_LETIMER_UF;
  sl_bt_external_signal(EVENT_LETIMER_UF);

  CORE_EXIT_CRITICAL();
}

void schedulerSetEventUFConf(){

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  myEvents |= EVENT_LETIMER_UF_CONF;

  CORE_EXIT_CRITICAL();

}

void schedulerSetEventComp1Conf(){

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  myEvents |= EVENT_LETIMER_COMP1_CONF;

  CORE_EXIT_CRITICAL();
}

uint32_t getNextEvent(){

  CORE_DECLARE_IRQ_STATE;

  uint32_t retEvent;

  retEvent = myEvents;

  CORE_ENTER_CRITICAL();

  if(myEvents & EVENT_LETIMER_UF_CONF){

      retEvent = EVENT_LETIMER_UF_CONF;
      myEvents &= ~EVENT_LETIMER_UF_CONF;

  }else if(myEvents & EVENT_LETIMER_COMP1_CONF){

      retEvent = EVENT_LETIMER_COMP1_CONF;
      myEvents &= ~EVENT_LETIMER_COMP1_CONF;

  }

  CORE_EXIT_CRITICAL();

  return retEvent;
}

// ---------------------------------------------------------------------
// Public function
// This function is used to set the LETIMER0 Comp1 event
// @param None
// Returns None
// ---------------------------------------------------------------------
void schedulerSetEventComp1(){

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  //myEvents |= EVENT_LETIMER_COMP1;
  sl_bt_external_signal(EVENT_LETIMER_COMP1);

  CORE_EXIT_CRITICAL();
}

// ---------------------------------------------------------------------
// Public function
// This function is used to set the I2C0 transfer complete event
// @param None
// Returns None
// ---------------------------------------------------------------------
void schedulerSetI2CTransferComp(){

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  //myEvents |= EVENT_I2C_TRANSFER_COMP;
  sl_bt_external_signal(EVENT_I2C_TRANSFER_COMP);

  CORE_EXIT_CRITICAL();
}

// ---------------------------------------------------------------------
// Public function
// This function is used to set the PB0 button pressed/released event.
// @param None
// Returns None
// ---------------------------------------------------------------------
void  schedulerSetEventPB0(){

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  sl_bt_external_signal(EVENT_PB0);

  CORE_EXIT_CRITICAL();

}
// ---------------------------------------------------------------------
// Public function
// This function is used to set the PB1 button pressed/released event.
// @param None
// Returns None
// ---------------------------------------------------------------------
void  schedulerSetEventPB1(){

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  sl_bt_external_signal(EVENT_PB1);

  CORE_EXIT_CRITICAL();

}

// ---------------------------------------------------------------------
// Public function
// This function is used to set the I2C0 transfer complete event
// @param None
// Returns None
// ---------------------------------------------------------------------
void schedulerSetConnectionLostEvent(){

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();

  sl_bt_external_signal(EVENT_CONNECTION_LOST);

  CORE_EXIT_CRITICAL();

}

#if (DEVICE_IS_BLE_SERVER == 0)

// ---------------------------------------------------------------------
// Public function
// This function is used to run the client state machine
// @param BT stack msg context
// Returns None
// ---------------------------------------------------------------------
void discovery_state_machine(sl_bt_msg_t *evt){

  uint32_t ext_sig = 0;
  sl_status_t rc;

  ext_sig = SL_BT_MSG_ID(evt->header);

  State_t currentState;
  static State_t nextState = discoverService;

  /*Get ble data struct*/
  ble_data_struct_t* ble_params = get_ble_data_struct();

  /*Switch States*/
  currentState = nextState;

  switch(currentState){

    case discoverService:
      nextState = discoverService;
      if(ext_sig == sl_bt_evt_connection_opened_id){
          /*Discover HTM Service on client*/
          rc = sl_bt_gatt_discover_primary_services_by_uuid(ble_params->connectionHandle,
                                                            sizeof(thermo_service),
                                                            (const uint8_t*)thermo_service);
          if(rc != SL_STATUS_OK){
              LOG_ERROR("Bluetooth: Service discovery error = %d\r\n", (unsigned int) rc);
          }
          nextState = discoverChar;
      }
      break;

    case discoverChar:
      nextState = discoverChar;
      /*If current GATT command completed, discover HTM characteristic on client*/
      if(ext_sig == sl_bt_evt_gatt_procedure_completed_id){

          LOG_INFO("HTM Service discovered, now discovering HTM Characteristic..\r\n");

          rc = sl_bt_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection,
                                                           ble_params->serviceHandle,
                                                           sizeof(thermo_char),
                                                           (const uint8_t*)thermo_char);
          if(rc != SL_STATUS_OK){
              LOG_ERROR("Bluetooth: Characteristic discovery error = %d\r\n", (unsigned int) rc);
          }
          nextState = setIndication;
      }

      else if(ext_sig == sl_bt_evt_connection_closed_id){
          nextState = discoverService;
      }
      break;

    case setIndication:
      nextState = setIndication;
      /*If current GATT command completed, enable indications on client*/
      if(ext_sig == sl_bt_evt_gatt_procedure_completed_id){

          LOG_INFO("HTM Characteristic discovered, now enabling indication..\r\n");

          rc = sl_bt_gatt_set_characteristic_notification(evt->data.evt_gatt_procedure_completed.connection,
                                                          ble_params->characteristicHandle,
                                                          sl_bt_gatt_indication );
          if(rc != SL_STATUS_OK){
              LOG_ERROR("Bluetooth: Set characteristic indication error = %d\r\n", (unsigned int) rc);
          }
          nextState = getIndication;
      }
      else if(ext_sig == sl_bt_evt_connection_closed_id){
          nextState = discoverService;
      }
      break;

    case getIndication:
      nextState = getIndication;
      /*If current GATT command completed*/
      if(ext_sig == sl_bt_evt_gatt_procedure_completed_id){

          LOG_INFO("Indications enabled, now expecting temperature..\r\n");

          nextState = startScanning;
     }
     else if(ext_sig == sl_bt_evt_connection_closed_id){
         nextState = discoverService;
     }
     break;

    case startScanning:
      nextState = startScanning;
      /*If connection closed any time*/
      if(ext_sig == sl_bt_evt_connection_closed_id){
          nextState = discoverService;
      }
      break;

  }// switch

}

#elif (DEVICE_IS_BLE_SERVER == 1)

void set_device_mode(sl_bt_msg_t *evt){

  uint32_t ext_sig = 0;
  static bool init_mode = true;

  ble_data_struct_t* ble_params = get_ble_data_struct();
  bool bonded_state = ble_params->bonded;

  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
      ext_sig =  evt->data.evt_system_external_signal.extsignals;
  }else{
      return;
  }

  bool pb0_val = get_pbo_val();
  bool pb1_val = get_pb1_val();

  State_DevMode_t currentState;
  static State_DevMode_t nextState = stateDevInit;

  /*Switch States*/
  currentState = nextState;

  switch(currentState){

    case stateDevInit:

      nextState = stateDevInit;

      if(bonded_state == true){
       nextState = stateDevSetMode;
       displayPrintf(DISPLAY_ROW_ACTION, "PB0 for Trig Md");
       displayPrintf(DISPLAY_ROW_8, "PB1 for Cont md");
      }
      break;

    case stateDevSetMode:

      nextState = stateDevSetMode;

      if(pb0_val == true){
          dev_mode = TRIGGER_MODE;
          nextState = stateDevSetDone;
      }else if(pb1_val == true){
          dev_mode = CONTINUOUS_MODE;
          nextState = stateDevSetDone;
      }

      break;

    case stateDevSetDone:
      nextState = stateDevSetDone;
      //Do nothing
      break;

  }

}

void mode_state_machine(sl_bt_msg_t *evt){

  //Go ahead only, if bonding completed
  if(dev_mode == TRIGGER_MODE){
      trig_mode_state_machine(evt);
  }else if(dev_mode == CONTINUOUS_MODE){
      cont_mode_state_machine(evt);
  }

}

void trig_mode_state_machine(sl_bt_msg_t *evt){

  uint32_t ext_sig = 0;
  static cycle_count = 0;

  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
      ext_sig =  evt->data.evt_system_external_signal.extsignals;
  }else{
      return;
  }

  State_TrigMode_t currentState;
  static State_TrigMode_t nextState = stateTrigInit;

  /*Switch States*/
  currentState = nextState;

  switch(currentState){

    case stateTrigInit:
      //1. Clear old prints
      displayPrintf(DISPLAY_ROW_ACTION, "");
      displayPrintf(DISPLAY_ROW_8, "");
      displayPrintf(DISPLAY_ROW_11, "Current mode:Trigger");

      //2. Print Vital Params
      displayPrintf(DISPLAY_ROW_ACTION, "Body temp: --");
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "Pulse rate: --");
      displayPrintf(DISPLAY_ROW_8, "SpO2: --");

      nextState = stateTrigReadTemp;

      break;

    case stateTrigReadTemp:
      nextState = stateTrigReadTemp;

      bool done = temperature_state_machine(evt);

      if(done == true){
          nextState = stateTrigReadMax;
          cycle_count++;
      }
      break;

    case stateTrigReadMax:
      nextState = stateTrigReadMax;

      read_max_3266_single(evt);

      if(cycle_count == 5){
          nextState = stateTrigRestart;
          cycle_count = 0;
      }else{
          nextState = stateTrigReadTemp;
      }

      break;

    case stateTrigRestart:
      nextState = stateTrigRestart;

      displayPrintf(DISPLAY_ROW_10, "Press PB0 to restart");

      bool pb0_val = get_pbo_val();

      if(pb0_val == true){
          displayPrintf(DISPLAY_ROW_10, "");
          nextState = stateTrigInit;
      }
      break;
  }

}

void cont_mode_state_machine(sl_bt_msg_t *evt){

  uint32_t ext_sig = 0;

  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
      ext_sig =  evt->data.evt_system_external_signal.extsignals;
  }else{
      return;
  }

  State_ContMode_t currentState;
  static State_ContMode_t nextState = stateContInit;

  /*Switch States*/
  currentState = nextState;

  switch(currentState){

    case stateContInit:
      //1. Clear old prints
      displayPrintf(DISPLAY_ROW_ACTION, "");
      displayPrintf(DISPLAY_ROW_8, "");
      displayPrintf(DISPLAY_ROW_11, "Current mode:Conti");

      //2. Print Vital Params
      displayPrintf(DISPLAY_ROW_ACTION, "Body temp: --");
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "Pulse rate: --");
      displayPrintf(DISPLAY_ROW_8, "SpO2: --");

      nextState = stateContReadTemp;

      break;

    case stateContReadTemp:
      nextState = stateContReadTemp;

      bool done = temperature_state_machine(evt);

      if(done == true){
          nextState = stateContReadMax;
      }
      break;

    case stateContReadMax:
      nextState = stateContReadMax;

      read_max_3266_single(evt);

      nextState = stateContReadTemp;

      break;
  }

}

// ---------------------------------------------------------------------
// Public function
// This function is used to run the temperature read & state machine
// @param None
// Returns None
// ---------------------------------------------------------------------
bool temperature_state_machine(sl_bt_msg_t *evt){

  uint32_t ext_sig = 0;
  bool temp_process_done = false;

  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
      ext_sig =  evt->data.evt_system_external_signal.extsignals;
  }else{
      return false;
  }

  ble_data_struct_t* ble_params = get_ble_data_struct();

  State_t currentState;
  static State_t nextState = stateIdle;

  /*Stop taking temperature measurement if BLE connection is closed or
   * HTM indications are disabled.*/
  if(ble_params->connection_open == false || ble_params->ok_to_send_htm_connections == false){
      nextState = stateIdle;
      // Clear Temperature print from LCD in this case
      displayPrintf(DISPLAY_ROW_ACTION, " ");
      return true;//change this to true
  }

  /*Switch States*/
  currentState = nextState;

  switch(currentState){

    case stateIdle:
        if(ext_sig == EVENT_LETIMER_UF)
          nextState = stateSensorOn;
        break;

    case stateSensorOn:
        //1. Enable Si7021
        gpioSi7021Enable();
        //2. Wait for POR time
        timerWaitUs_irq(POR_STABILIZATION_TIME);
        nextState = stateI2CWrite;
        // Check for BLE connection lost
        if(ext_sig == EVENT_CONNECTION_LOST){
            I2CTeardown();
            nextState = stateIdle;
        }
        break;

   case stateI2CWrite:
       //3. Wait POR time
       if(ext_sig == EVENT_LETIMER_COMP1){
           //Add EM1 power requirement
           sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
           I2C_write();
           nextState = stateI2CWait;
       }
       // Check for BLE connection lost
       if(ext_sig == EVENT_CONNECTION_LOST){
           I2CTeardown();
           nextState = stateIdle;
       }
      break;

   case stateI2CWait:
     //4. Wait for I2C write to complete
     if(ext_sig == EVENT_I2C_TRANSFER_COMP){
         //Remove EM1 power requirement
         sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
         //Disable NVIC for I2C0
         NVIC_DisableIRQ(I2C0_IRQn);
         timerWaitUs_irq(SI7021_CONVERSION_TIME);
         nextState = stateI2CRead;
     }
     // Check for BLE connection lost
     if(ext_sig == EVENT_CONNECTION_LOST){
         I2CTeardown();
         nextState = stateIdle;
     }
     break;

   case stateI2CRead:
     //5. Wait for Conversion time
     if(ext_sig == EVENT_LETIMER_COMP1){
         //Add EM1 power requirement
         sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
         I2C_read();
         nextState = stateGetResult;
     }
     // Check for BLE connection lost
     if(ext_sig == EVENT_CONNECTION_LOST){
         I2CTeardown();
         nextState = stateIdle;
     }
     break;

   case stateGetResult:
     //6. Wait for I2C Read transfer to complete
     if(ext_sig == EVENT_I2C_TRANSFER_COMP){
         //Remove EM1 power requirement
         sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
         //Disable Si7021 - Turn Power Off
         //gpioSi7021Disable(); // Commenting for Display A6
         //Disable NVIC for I2C0
         NVIC_DisableIRQ(I2C0_IRQn);
         //Covert and print the result
         get_result();
         send_temp_ble();
         //read_max_3266_single(evt);
         nextState = stateIdle;
     }
     // Check for BLE connection lost
     if(ext_sig == EVENT_CONNECTION_LOST){
         I2CTeardown();
         nextState = stateIdle;
     }
     break;

  }// switch

  if(currentState == stateGetResult){
      temp_process_done = true;
  }else{
      temp_process_done = false;
  }

  return temp_process_done;

}
#endif
