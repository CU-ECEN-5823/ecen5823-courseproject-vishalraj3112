/*
 * max_3266.c
 *
 *  Created on: Apr 30, 2023
 *      Author: vishal
 */

#define INCLUDE_LOG_DEBUG     1

#include "max_3266.h"
#include "i2c.h"
#include "log.h"
#include "timers.h"
#include "ble.h"

// ---------------------------------------------------------------------
// Public function
// This function is used to initialize the Max 3266 sensor hub for setting
// modes, enable sensor, set algorithms etc.
// @param None
// Returns None
// ---------------------------------------------------------------------
void init_max_3266(){

  int status;

  timerWaitUs_polled(1100000);//Wait for 1.1s at the start

  //Enter EM1 at start for I2C transfers
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);

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

  //Remove EM1 requirement
  sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

  timerWaitUs_polled(1000000);//Wait 1 sec

  LOG_INFO("Sensor Configured!\r\n");

  LOG_INFO("Loading sensor buffer data...\r\n");

  //Wait 4 secs after init
  timerWaitUs_polled(2000000);//Wait 2 secs
  timerWaitUs_polled(2000000);//Wait 2 secs

}

// ---------------------------------------------------------------------
// Public function
// This function is used to read the Max 3266 for heart rate and Spo2
// for single cycle and also send these values to client by sending
// command to Ble stack.
// @param None
// Returns None
// ---------------------------------------------------------------------
void read_max_3266_single(sl_bt_msg_t *evt){

  int status;
  uint32_t ext_sig = 0;

  if(SL_BT_MSG_ID(evt->header) ==  sl_bt_evt_system_external_signal_id){
      ext_sig =  evt->data.evt_system_external_signal.extsignals;
  }else{
      return;
  }

  ble_data_struct_t* ble_params = get_ble_data_struct();

  //Enter EM1 at start for I2C transfers
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);

  //1. Get sensor hub status
  status = get_sensor_hub_status();
  if(status == 0x00){
     LOG_INFO("Sensor hub status ok!\r\n");
  }else{
     LOG_ERROR("Max Sensor Hub: Sensor 1 hub  status error = 0x%X\r\n", (unsigned int) status);
  }

  //2. Get no of samples in the FIFO
  uint8_t no_samples = 0;
  status = get_fifo_no_samples(&no_samples);
  if(status == 0x00){
     LOG_INFO("Sensor hub Fifo number samples: %d!\r\n", no_samples);
  }else{
     LOG_ERROR("Max Sensor Hub: Sensor 1 hub  status error = %d\r\n", (unsigned int) status);
  }

  //3. Read the data stored in the FIFO.
  status = sh_read_output_fifo();
  if(status == 0x00){
      LOG_INFO("Output FIFO Data read!\r\n");
  }else{
      LOG_ERROR("Output Fifo Data read error = %d\r\n", (unsigned int) status);
  }

  //Remove EM1 requirement
  sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

  //4. Dump the read FIFO data to terminal.
  dump_op_fifo_data();

  //5. Run Finger status LED logic.
  led_finger_status();

  /*Stop sending heart rate if BLE connection is closed or
   * HR indications are disabled.*/
  if(ble_params->connection_open == true && ble_params->ok_to_send_hr_indications == true){
      //6. Send only heart rate to client
      send_heart_rate_ble();
  }

  /*Stop sending Spo2 if BLE connection is closed or
   * Spo2 indications are disabled.*/
  if(ble_params->connection_open == true && ble_params->ok_to_send_o2_indications == true){
    //7. Send SpO2 value to client
    send_spo2_ble();
  }

}
