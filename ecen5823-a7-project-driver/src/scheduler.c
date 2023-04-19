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

#if DEVICE_IS_BLE_SERVER

  typedef enum {
    stateIdle_max,
    stateSensorCheck,
    set_data_type,
    set_thresh,
    enable_sh,
    enable_algo,
    start_sh_read
  }State_max_t;

  typedef enum uint32_t {
    stateIdle,
    stateSensorOn,
    stateI2CWrite,
    stateI2CWait,
    stateI2CRead,
    stateGetResult,
  }State_t;
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

//void max_hub_read(sl_bt_msg_t *evt){
//
//  uint32_t ext_sig = 0;
//
//  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
//      ext_sig =  evt->data.evt_system_external_signal.extsignals;
//  }else{
//      return;
//  }
//
//  State_max_t currentState;
//  static State_max_t nextState = stateIdle_max;
//
//  /*Switch States*/
//  currentState = nextState;
//
//  switch(currentState){
//
//    case stateIdle_max:
//      nextState = stateIdle_max;
//      if(ext_sig == EVENT_LETIMER_UF)
//        nextState = stateSensorCheck;
//      break;
//
//    case stateSensorCheck:
//       nextState = stateSensorCheck;
//       get_sensor_hub_status();
//
//       if(ext_sig == EVENT_I2C_TRANSFER_COMP){
//
//         check_sh_status();
//         NVIC_DisableIRQ(I2C0_IRQn);
//
//         nextState = enable_sh;
//       }
//       break;
//
//    case enable_sh:
//      nextState = enable_sh;
//      sh_enable(0);
//
//      if(ext_sig == EVENT_I2C_TRANSFER_COMP){
//
//          check_sh_enable_status();
//          NVIC_DisableIRQ(I2C0_IRQn);
//
//          nextState = stateIdle_max;
//
//      }
//      break;
//  }
//
//
//}

void max_hub_read_polled(sl_bt_msg_t *evt){

  uint32_t ext_sig = 0;
  int status;
  static uint8_t init_state = 1;

  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
      ext_sig =  evt->data.evt_system_external_signal.extsignals;
  }else{
      return;
  }

  State_max_t currentState;
  static State_max_t nextState = stateIdle_max;

  /*Switch States*/
  currentState = nextState;

  switch(currentState){

    case stateIdle_max:
      nextState = stateIdle_max;
      if(ext_sig == EVENT_LETIMER_UF)
        nextState = stateSensorCheck;
      break;

    case stateSensorCheck:
     nextState = stateSensorCheck;

     status = get_sensor_hub_status();
     if(status == 0x00){
         LOG_INFO("No comm error in reading Sensor 1 from Sensor hub\r\n");
     }else{
         LOG_ERROR("Max Sensor Hub: Sensor 1 hub error = %d\r\n", (unsigned int) status);
     }

     if(init_state){
       nextState = set_data_type;
       init_state = 0;
     }else{
       nextState = start_sh_read;
     }

     break;

    /*1, 2, & 3 not working in a single state. Possibly because of less delay between commands.*/
    case set_data_type:
      nextState = set_data_type;

      //1. Set data type to Sensor and algorithm data
      status = sh_set_data_type(0x03);
      if(status == 0x00){
          LOG_INFO("Sensor data type set to sensor and algorithm data!\r\n");
      }else{
          LOG_ERROR("Max Sensor Hub: Sensor data type set error = 0x%X\r\n", (unsigned int) status);
      }

      nextState = set_thresh;
      break;

    case set_thresh:
      nextState = set_thresh;

      //2. Set max threshold of output FIFO to 15.
      status = sh_set_fifo_thresh(0x05);
      if(status == 0x00){
          LOG_INFO("Sensor threshold set to 15!\r\n");
      }else{
          LOG_ERROR("Max Sensor Hub: Sensor threshold set error = 0x%X\r\n", (unsigned int) status);
      }

      nextState = enable_sh;

      break;


    case enable_sh:
      nextState = enable_sh;

      //3. Enable MAX30101 sensor.
      status = sh_enable(0x03);
      if(status == 0x00){
          LOG_INFO("Sensor no. 0x03 enabled!\r\n");
      }else{
          LOG_ERROR("Max Sensor Hub: Sensor no. 0x03 enable error = 0x%X\r\n", (unsigned int) status);
      }

      nextState = enable_algo;
      break;

    case enable_algo:
      nextState = enable_algo;

      //4. Enable the WHRM/MaximFast 10.x algorithm
      status = sh_enable_algo(0x02);
      if(status == 0x00){
          LOG_INFO("Algo WHRM enabled!\r\n");
      }else{
          LOG_ERROR("Max Algo: Algo WHRM enable error = %d\r\n", (unsigned int) status);
      }

      nextState = start_sh_read;

      break;

    case start_sh_read:
      nextState = start_sh_read;

      //5. Get the number of sample in the FIFO
      int no_samples = 0;

      status = get_sh_no_samples(&no_samples);
      if(status == 0x00){
          LOG_INFO("No of samples in FIFO = %d\r\n",no_samples);
      }else{
          LOG_ERROR("Max Sensor Hub: No samples read error = %d\r\n", (unsigned int) status);
      }

      //6. Read the data stored in the FIFO.
      status = sh_read_output_fifo();
      if(status == 0x00){
          LOG_INFO("Output FIFO Data read!\r\n");
      }else{
          LOG_ERROR("Output Fifo Data read error = %d\r\n", (unsigned int) status);
      }

      //4. Dump the read FIFO data to terminal.
      dump_op_fifo_data();

      nextState= stateIdle_max;

      break;

  }

}

// ---------------------------------------------------------------------
// Public function
// This function is used to run the temperature read & state machine
// @param None
// Returns None
// ---------------------------------------------------------------------
void temperature_state_machine(sl_bt_msg_t *evt){

  uint32_t ext_sig = 0;

  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
      ext_sig =  evt->data.evt_system_external_signal.extsignals;
  }else{
      return;
  }

  ble_data_struct_t* ble_params = get_ble_data_struct();

  State_t currentState;
  static State_t nextState = stateIdle;

  /*Stop taking temperature measurement if BLE connection is closed or
   * HTM indications are disabled.*/
  if(ble_params->connection_open == false || ble_params->ok_to_send_htm_connections == false){
      nextState = stateIdle;
      // Clear Temperature print from LCD in this case
      displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
      return;
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
         nextState = stateIdle;
     }
     // Check for BLE connection lost
     if(ext_sig == EVENT_CONNECTION_LOST){
         I2CTeardown();
         nextState = stateIdle;
     }
     break;

  }// switch

}
#endif
