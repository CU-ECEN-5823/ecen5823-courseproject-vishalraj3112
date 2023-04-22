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

static uint32_t myEvents = 0;

  typedef enum {
    stateIdle_max,
    stateSensorCheck,
    set_data_type,
    set_thresh,
    enable_sh,
    enable_algo,
    enable_maximFast,
    start_sh_read,
    invalid_state
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

void init_max_3266(){

  int status;

  timerWaitUs_polled(1100000);//Wait for 1.1s at the start

  uint8_t device_mode;
  status = get_device_mode(&device_mode);
  if(status == 0x00){
      LOG_INFO("Device mode = 0x%X\r\n", device_mode);
  }else{
      LOG_ERROR("Device mode read error = %d\r\n", (unsigned int) status);
  }

  status = sh_set_data_type(0x02);
  if(status == 0x00){
      LOG_INFO("Sensor data type set to algorithm data!\r\n");
  }else{
      LOG_ERROR("Max Sensor Hub: Sensor data type set error = 0x%X\r\n", (unsigned int) status);
  }

  status = sh_set_fifo_thresh(0x0F);
  if(status == 0x00){
      LOG_INFO("Sensor threshold set to 15!\r\n");
  }else{
      LOG_ERROR("Max Sensor Hub: Sensor threshold set error = 0x%X\r\n", (unsigned int) status);
  }

  status = sh_enable_algo(0x00);
  if(status == 0x00){
      LOG_INFO("Algo AGC enabled!\r\n");
  }else{
      LOG_ERROR("Max Algo: Algo AGC enable error = %d\r\n", (unsigned int) status);
  }

  status = sh_enable(0x03);
  if(status == 0x00){
      LOG_INFO("Sensor no. 0x03 enabled!\r\n");
  }else{
      LOG_ERROR("Max Sensor Hub: Sensor no. 0x03 enable error = 0x%X\r\n", (unsigned int) status);
  }

  status = sh_enable_maxim_fast(0x01);
  if(status == 0x00){
      LOG_INFO("Sensor Maxim algo enabled!\r\n");
  }else{
      LOG_ERROR("Maxim fast algo set error = 0x%X\r\n", (unsigned int) status);
  }

  //5. Get the number of sample in the FIFO
  uint8_t no_samples = 0;

  status = get_sh_no_samples(&no_samples);
  if(status == 0x00){
      LOG_INFO("No of samples in FIFO = %d\r\n",no_samples);
  }else{
      LOG_ERROR("Max Sensor Hub: No samples read error = %d\r\n", (unsigned int) status);
  }

  timerWaitUs_polled(1000000);//Wait 1 sec

  LOG_INFO("Sensor Configured!\r\n");

  LOG_INFO("Loading sensor buffer data...\r\n");

  //Wait 4 secs after init
  timerWaitUs_polled(2000000);//Wait 2 secs
  timerWaitUs_polled(2000000);//Wait 2 secs

}

void read_max_32664(){

  uint32_t event;
  int status;

  event = getNextEvent();

  State_max_t currentState;
  static State_max_t nextState = stateIdle_max;

  /*Switch States*/
  currentState = nextState;

  switch(currentState){

    case stateIdle_max:

      nextState = stateIdle_max;

      timerWaitUs_irq(1000000);//1sec //250ms

      nextState = start_sh_read;
      break;


    case start_sh_read:
      nextState = start_sh_read;

      if(event == EVENT_LETIMER_COMP1_CONF){

          status = get_sensor_hub_status();
          if(status == 0x00){
             LOG_INFO("Sensor hub status ok!\r\n");
          }else{
             LOG_ERROR("Max Sensor Hub: Sensor 1 hub  status error = 0x%X\r\n", (unsigned int) status);
          }

          uint8_t no_samples = 0;
          status = get_fifo_no_samples(&no_samples);
          if(status == 0x00){
             LOG_INFO("Sensor hub Fifo number samples: %d!\r\n", no_samples);
          }else{
             LOG_ERROR("Max Sensor Hub: Sensor 1 hub  status error = %d\r\n", (unsigned int) status);
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


          nextState = stateIdle_max;
      }
      break;

  }

}


void max_hub_read_polled(sl_bt_msg_t *evt){

  uint32_t ext_sig = 0;
  int status;
  static uint8_t init_state = 1;

//  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
//      ext_sig =  evt->data.evt_system_external_signal.extsignals;
//  }else{
//      return;
//  }

  uint32_t event;

  event = getNextEvent();

  State_max_t currentState;
  static State_max_t nextState = stateIdle_max;

  /*Switch States*/
  currentState = nextState;

  /*Test section
  switch(currentState){

    case stateIdle_max:
      if(event == EVENT_LETIMER_UF_CONF){

          LOG_INFO("UF event occured!\r\n");
          timerWaitUs_irq(1000000);//1000ms
          gpioLed1SetOn();
          nextState = stateSensorCheck;

      }
      break;

    case stateSensorCheck:
      if(event == EVENT_LETIMER_COMP1_CONF){
          LOG_INFO("COMP1 event occured!\r\n");
          gpioLed1SetOff();
          nextState = stateIdle_max;
      }
      break;

  }
  Test section*/


  switch(currentState){

    case stateIdle_max:
      nextState = stateIdle_max;

      //If coming for the first time, or in second cycle after COMP1 delay.

        if(init_state){
          nextState = stateSensorCheck;
          init_state = 0;
        }else{
          nextState = start_sh_read;
        }

      break;

    case stateSensorCheck:
     nextState = stateSensorCheck;

     timerWaitUs_polled(1100000);//Wait for 1.1s at the start

     uint8_t device_mode;
     status = get_device_mode(&device_mode);
     if(status == 0x00){
         LOG_INFO("Device mode = 0x%X\r\n", device_mode);
     }else{
         LOG_ERROR("Device mode read error = %d\r\n", (unsigned int) status);
     }

     status = sh_set_data_type(0x02);
     if(status == 0x00){
         LOG_INFO("Sensor data type set to algorithm data!\r\n");
     }else{
         LOG_ERROR("Max Sensor Hub: Sensor data type set error = 0x%X\r\n", (unsigned int) status);
     }

     status = sh_set_fifo_thresh(0x01);
     if(status == 0x00){
         LOG_INFO("Sensor threshold set to 1!\r\n");
     }else{
         LOG_ERROR("Max Sensor Hub: Sensor threshold set error = 0x%X\r\n", (unsigned int) status);
     }

     status = sh_enable_algo(0x00);
     if(status == 0x00){
         LOG_INFO("Algo AGC enabled!\r\n");
     }else{
         LOG_ERROR("Max Algo: Algo AGC enable error = %d\r\n", (unsigned int) status);
     }

     status = sh_enable(0x03);
     if(status == 0x00){
         LOG_INFO("Sensor no. 0x03 enabled!\r\n");
     }else{
         LOG_ERROR("Max Sensor Hub: Sensor no. 0x03 enable error = 0x%X\r\n", (unsigned int) status);
     }

     status = sh_enable_maxim_fast(0x01);
     if(status == 0x00){
         LOG_INFO("Sensor Maxim algo enabled!\r\n");
     }else{
         LOG_ERROR("Maxim fast algo set error = 0x%X\r\n", (unsigned int) status);
     }

     //5. Get the number of sample in the FIFO
     uint8_t no_samples = 0;

     status = get_sh_no_samples(&no_samples);
     if(status == 0x00){
         LOG_INFO("No of samples in FIFO = %d\r\n",no_samples);
     }else{
         LOG_ERROR("Max Sensor Hub: No samples read error = %d\r\n", (unsigned int) status);
     }

     timerWaitUs_polled(1000000);//Wait 1 sec

     LOG_INFO("Sensor Configured!\r\n");

     LOG_INFO("Loading sensor buffer data...\r\n");

     nextState = set_data_type;

     break;

    case set_data_type:
      nextState = set_data_type;

      //1. Set data type to algorithm data
//      status = sh_set_data_type(0x02);
//      if(status == 0x00){
//          LOG_INFO("Sensor data type set to algorithm data!\r\n");
//      }else{
//          LOG_ERROR("Max Sensor Hub: Sensor data type set error = 0x%X\r\n", (unsigned int) status);
//      }

      nextState = set_thresh;
      break;

    case set_thresh:
      nextState = set_thresh;

      //2. Set max threshold of output FIFO to 15.
//      status = sh_set_fifo_thresh(0x01);
//      if(status == 0x00){
//          LOG_INFO("Sensor threshold set to 1!\r\n");
//      }else{
//          LOG_ERROR("Max Sensor Hub: Sensor threshold set error = 0x%X\r\n", (unsigned int) status);
//      }

      nextState = enable_algo;

      break;

    case enable_algo:
      nextState = enable_algo;

      //4. Enable the AGC algorithm
//      status = sh_enable_algo(0x00);
//      if(status == 0x00){
//          LOG_INFO("Algo AGC enabled!\r\n");
//      }else{
//          LOG_ERROR("Max Algo: Algo AGC enable error = %d\r\n", (unsigned int) status);
//      }

      nextState = enable_sh;

      break;

    case enable_sh:
      nextState = enable_sh;

      //3. Enable MAX30101 sensor.
//      status = sh_enable(0x03);
//      if(status == 0x00){
//          LOG_INFO("Sensor no. 0x03 enabled!\r\n");
//      }else{
//          LOG_ERROR("Max Sensor Hub: Sensor no. 0x03 enable error = 0x%X\r\n", (unsigned int) status);
//      }

      nextState = enable_maximFast;
      break;

    case enable_maximFast:
      nextState = enable_maximFast;

      //3. Enable Maxim fast algo.
//      status = sh_enable_maxim_fast(0x01);
//      if(status == 0x00){
//          LOG_INFO("Sensor Maxim algo enabled!\r\n");
//      }else{
//          LOG_ERROR("Maxim fast algo set error = 0x%X\r\n", (unsigned int) status);
//      }
//
//      //5. Get the number of sample in the FIFO
//      uint8_t no_samples = 0;
//
//      status = get_sh_no_samples(&no_samples);
//      if(status == 0x00){
//          LOG_INFO("No of samples in FIFO = %d\r\n",no_samples);
//      }else{
//          LOG_ERROR("Max Sensor Hub: No samples read error = %d\r\n", (unsigned int) status);
//      }
//
//      timerWaitUs_polled(1000000);//Wait 1 sec
//
//      LOG_INFO("Sensor Configured!\r\n");
//
//      LOG_INFO("Loading sensor buffer data...\r\n");

      //timerWaitUs_polled(4000000);//Wait 4 secs

      nextState = start_sh_read;

      break;

    case start_sh_read:

      nextState = start_sh_read;

      status = get_sensor_hub_status();
      if(status == 0x00){
         LOG_INFO("Sensor hub status ok!\r\n");
      }else{
         LOG_ERROR("Max Sensor Hub: Sensor 1 hub  status error = 0x%X\r\n", (unsigned int) status);
      }

      status = get_fifo_no_samples(&no_samples);
      if(status == 0x00){
         LOG_INFO("Sensor hub Fifo number samples: %d!\r\n", no_samples);
      }else{
         LOG_ERROR("Max Sensor Hub: Sensor 1 hub  status error = %d\r\n", (unsigned int) status);
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

      //timerWaitUs_polled(2500000);//Wait 2500ms before next cycle
      timerWaitUs_irq(250000);

      nextState = stateIdle_max;
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
