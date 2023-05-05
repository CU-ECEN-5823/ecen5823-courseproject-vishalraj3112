/***********************************************************************
 * @file      i2c.c
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
 * @resources Lecture Slides and EFR32xG13 Reference Manual
 *
 */
#define INCLUDE_LOG_DEBUG     0

#include "i2c.h"
#include "gpio.h"
#include "src/log.h"
#include "sl_i2cspm.h"
#include "src/timers.h"
#include "lcd.h"

I2C_TransferSeq_TypeDef transferSequence;
uint8_t cmd_data;
static uint16_t read_data;
static uint32_t temp_degree_C;
static uint8_t op_fifo_buf[7] = {0};

uint32_t* get_temp_val(){

  return &temp_degree_C;

}

// ---------------------------------------------------------------------
// Private function
// This function is used to initialize the I2C0 and configure it.
// @param None
// Returns None
// ---------------------------------------------------------------------
void I2C0_init(){

  //uint32_t i2c_bus_freq;

  I2CSPM_Init_TypeDef i2c_config_struct = {
  .port = I2C0,
  .sclPort = gpioPortC,
  .sclPin = 10,
  .sdaPort = gpioPortC,
  .sdaPin = 11,
  .portLocationScl = 14,
  .portLocationSda = 16,
  .i2cRefFreq = 0,
  .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,
  .i2cClhr = i2cClockHLRStandard
  };

  //Initialize the I2C
  I2CSPM_Init(&i2c_config_struct);

  //Get the configured bus frequency
  //i2c_bus_freq = I2C_BusFreqGet(I2C0);

}// I2C0_init()

// ---------------------------------------------------------------------
// Private function
// This function is used to initialize the I2C0 and configure it for
// Max 3266 sensor hub.
// @param None
// Returns None
// ---------------------------------------------------------------------
void I2C0_init_max(){

  //uint32_t i2c_bus_freq;

  I2CSPM_Init_TypeDef i2c_config_struct = {
  .port = I2C0,
  .sclPort = gpioPortC,
  .sclPin = 10,
  .sdaPort = gpioPortC,
  .sdaPin = 11,
  .portLocationScl = 14,
  .portLocationSda = 16,
  .i2cRefFreq = 0,
  .i2cMaxFreq = 400000/*I2C_FREQ_STANDARD_MAX*/,
  .i2cClhr = i2cClockHLRStandard
  };

  //Initialize the I2C
  I2CSPM_Init(&i2c_config_struct);

  //Get the configured bus frequency
  //i2c_bus_freq = I2C_BusFreqGet(I2C0);

}// I2C0_init()

// ---------------------------------------------------------------------
// Public function
// This function is used to write, temp read command to Si7021.
// @param None
// Returns None
// ---------------------------------------------------------------------
void I2C_write(){

  I2C_TransferReturn_TypeDef transferStatus;

  I2C0_init();

  //Write - Read temp command
  cmd_data = MEASURE_TEMP_CMD;
  //Set device address and unset read/write bit
  transferSequence.addr = SI7021_DEVICE_ADDR << 1;
  //Set for write command
  transferSequence.flags = I2C_FLAG_WRITE;
  //Set the command to send and its size
  transferSequence.buf[0].data = &cmd_data;
  transferSequence.buf[0].len = sizeof(cmd_data);

  //Enable NVIC for I2C0
  NVIC_EnableIRQ(I2C0_IRQn);

  /*This is a non blocking call, generates an i2cTransferDone event*/
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if(transferStatus < 0){
      LOG_ERROR("I2C_TransferInit: Write error = %d\r\n", transferStatus);
  }

} //I2C_write()

// ---------------------------------------------------------------------
// Private function
// This function is used to perform I2C write command for Max 3266.
// @param slave_addr - Slave address to write the command.
// @param cmd_byte - Buffer of the command to write.
// @param cmd_len - Length of the buffer of write command.
// Returns None
// ---------------------------------------------------------------------
void I2C_Write_Max(uint8_t slave_addr, uint8_t* cmd_byte, uint16_t cmd_len){

  I2C_TransferReturn_TypeDef transferStatus;

  //uint8_t ByteSeq[] = {0x02, 0x00};

  I2C0_init_max();

  //Set device address and unset read/write bit
  transferSequence.addr = slave_addr;
  //Set for write command
  transferSequence.flags = I2C_FLAG_WRITE;
  //Set the command to send and its size
  transferSequence.buf[0].data = (uint8_t *) cmd_byte;//&cmd_byte;
  transferSequence.buf[0].len = cmd_len;

  //Enable NVIC for I2C0
  //NVIC_EnableIRQ(I2C0_IRQn);

  /*This is a blocking call, blocks until transfer is complete
   and ack received or any other error received*/
  transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
  if(transferStatus != i2cTransferDone){
      LOG_ERROR("I2CSPM_Transfer: Write error = %d\r\n", transferStatus);
  }

}

// ---------------------------------------------------------------------
// Public function
// This function is used to read temperature from Si7021.
// @param None
// Returns None
// ---------------------------------------------------------------------
void I2C_read(){

  I2C_TransferReturn_TypeDef transferStatus;

  I2C0_init();

  //Set device address and unset read/write bit
  transferSequence.addr = SI7021_DEVICE_ADDR << 1;
  //Set for read command
  transferSequence.flags = I2C_FLAG_READ;
  //Set the command to send and its size
  transferSequence.buf[0].data = (uint8_t *)  &read_data;
  transferSequence.buf[0].len = sizeof(read_data);

  //Enable NVIC for I2C0
  NVIC_EnableIRQ(I2C0_IRQn);

  /*This is a blocking call, blocks until transfer is complete
   and ack received or any other error received*/
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if(transferStatus < 0){
      LOG_ERROR("I2C_TransferInit: Read error = %d\r\n", transferStatus);
  }

} //I2C_read()

// ---------------------------------------------------------------------
// Private function
// This function is used to perform I2C write command for Max 3266.
// @param slave_addr - Slave address to write the command.
// @param cmd_byte - Buffer to store the read data.
// @param cmd_len - Length of the buffer of read data.
// Returns None
// ---------------------------------------------------------------------
void I2C_read_max(uint8_t slave_addr, uint8_t* rxbuf, uint16_t rxbuf_size){

  I2C_TransferReturn_TypeDef transferStatus;

  I2C0_init_max();

  //Set device address and unset read/write bit
  transferSequence.addr = slave_addr;
  //Set for Write Read command type
  transferSequence.flags = I2C_FLAG_READ;

  //Write 2 Command bytes to buffer0
  transferSequence.buf[0].data = (uint8_t *) rxbuf;
  transferSequence.buf[0].len = rxbuf_size;

  //Enable NVIC for I2C0
  //NVIC_EnableIRQ(I2C0_IRQn);

  /*This is a blocking call, blocks until transfer is complete
   and ack received or any other error received*/
  transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
  if(transferStatus != i2cTransferDone){
      LOG_ERROR("I2CSPM_Transfer: Read error = %d\r\n", transferStatus);
  }

}

// ---------------------------------------------------------------------
// Private function
// This function is used to read the data from Max 3266.
// @param cmd_bytes - buffer to the command to write.
// @param cmd_bytes_len - length of the buffer to write command.
// @param rxbuf - buffer to store the read data.
// @param rxbuf_size - length of the read data buffer.
// @param sleep_time - Delay to wait for response after command write in
//                     uS.
// Returns - command response status byte.
// ---------------------------------------------------------------------
int max_sh_read_cmd(uint8_t *cmd_bytes, uint16_t cmd_bytes_len,
                     uint8_t *rxbuf, uint16_t rxbuf_size,
                     uint16_t sleep_time){

  //1. First We need to do a multibyte I2C write of the command
  I2C_Write_Max(MAX32664_DEVICE_ADDR, cmd_bytes, cmd_bytes_len);

  //2. Then we need to wait for data ready time
  timerWaitUs_polled(sleep_time);

  //3. Lastly we need to do a multibyte I2C read of the data
  I2C_read_max(MAX32664_DEVICE_ADDR, rxbuf, rxbuf_size);

  return (int) rxbuf[0];
}

// ---------------------------------------------------------------------
// Private function
// This function is used to send data to Max 3266.
// @param txbuf - buffer to the command to write.
// @param txbuf_len - length of the buffer to write command.
// @param sleep_time - Delay to wait for response after command write in
//                     uS.
// Returns - command response status byte.
// ---------------------------------------------------------------------
int max_sh_write_cmd(uint8_t *txbuf,
                       int txbuf_len,
                         int sleep_time){

  char status_byte;

  //1. First do a I2C multi byte write
  I2C_Write_Max(MAX32664_DEVICE_ADDR, txbuf, txbuf_len);

  //2. Wait for specified delay
  timerWaitUs_polled(sleep_time);

  //3. Do I2C single byte read to read the written command status
  I2C_read_max(MAX32664_DEVICE_ADDR, &status_byte, 1);

  return (int) status_byte;
}

// ---------------------------------------------------------------------
// Public function
// This function is used to get the Max 3266 device mode
// - 0x00: Application operating mode.
// - 0x08: Bootloader operating mode.
// @param device_mode - reference to store the device mode.
// Returns - command response status byte.
// ---------------------------------------------------------------------
int get_device_mode(uint8_t* device_mode){

  uint8_t ByteSeq[] = {0x02, 0x00}, rxbuf[2] = {0};

  //makes no sense reading the status if its interrupt driven
  int status = max_sh_read_cmd(ByteSeq, sizeof(ByteSeq), &rxbuf[0], sizeof(rxbuf), DEFAULT_CMD_SLEEP_US);

  *device_mode = rxbuf[1];

  return status;
}

// ---------------------------------------------------------------------
// Public function
// This function is used to get the Max 3266 current sensor status.
// @param None
// Returns - command response status byte.
// ---------------------------------------------------------------------
int get_sensor_hub_status(){

  uint8_t ByteSeq[] = {0x00, 0x00}, rxbuf[2] = {0};

  //makes no sense reading the status if its interrupt driven
  max_sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq), &rxbuf[0], sizeof(rxbuf), DEFAULT_CMD_SLEEP_US);

  return (int) rxbuf[1];
}

// ---------------------------------------------------------------------
// Public function
// This function sets the data type required - Algo or sensor data.
// @param None
// Returns - command response status byte.
// ---------------------------------------------------------------------
int sh_set_data_type(uint8_t val){

  uint8_t ByteSeq[] = {0x10, 0x00, val};

  int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), DEFAULT_CMD_SLEEP_US);

  return status;

}

// ---------------------------------------------------------------------
// Public function
// Used to set the threshold for FIFO almost full indication from 0x01
// to 0xFF.
// @param None
// Returns - command response status byte.
// ---------------------------------------------------------------------
int sh_set_fifo_thresh(uint8_t thresh_val){

  uint8_t ByteSeq[] = {0x10, 0x01, thresh_val};

  int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), DEFAULT_CMD_SLEEP_US);

  return status;

}

// ---------------------------------------------------------------------
// Public function
// Used to enable the Max 3266 sensor hub.
// @param index to the specific sensor to enable MAXxxxx.
// Returns - command response status byte.
// ---------------------------------------------------------------------
int sh_enable(uint8_t index){

    uint8_t ByteSeq[] = {0x44, index, 0x01};

    int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), 3 * SENSOR_ENABLE_SLEEP_US);

    return status;
}

// ---------------------------------------------------------------------
// Public function
// Used to enable the Max 3266 sensor hub specific algorithm required.
// @param algo_idx to the specific algorithm to enable - AGC/AEC/WHRM.
// Returns - command response status byte.
// ---------------------------------------------------------------------
int sh_enable_algo(uint8_t algo_idx){

  uint8_t ByteSeq[] = {0x52, algo_idx, 0x01};

  int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), 3 * SENSOR_ENABLE_SLEEP_US);

  return status;
}

// ---------------------------------------------------------------------
// Public function
// Used to enable/disable the WHRM, MaximFast algorithm.
// @param mode 0: disable MaximFast algo.
//             1: enable MaximFast algo.
// Returns - command response status byte.
// ---------------------------------------------------------------------
int sh_enable_maxim_fast(uint8_t mode){

  uint8_t ByteSeq[] = {0x52, 0x02, mode};

  int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), 3 * SENSOR_ENABLE_SLEEP_US);//60ms

  return status;

}

// ---------------------------------------------------------------------
// Public function
// Used to get the current number of samples to average.
// @param no_samples reference to store the result of this command.
// Returns - command response status byte.
// ---------------------------------------------------------------------
int get_sh_no_samples(uint8_t * no_samples){

  uint8_t ByteSeq[] = {0x51, 0x00, 0x03}, rxbuf[2] = {0};

  //makes no sense reading the status if its interrupt driven
  int status = max_sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq), &rxbuf[0], sizeof(rxbuf), DEFAULT_CMD_SLEEP_US);

  *no_samples = rxbuf[1];

  return status;
}

// ---------------------------------------------------------------------
// Public function
// Used to get the current number of samples in the output FIFO.
// @param no_samples reference to store the result of this command.
// Returns - command response status byte.
// ---------------------------------------------------------------------
int get_fifo_no_samples(uint8_t* no_samples){

  uint8_t ByteSeq[] = {0x12, 0x00}, rxbuf[2] = {0};

  int status = max_sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq), &rxbuf[0], sizeof(rxbuf), DEFAULT_CMD_SLEEP_US);

  *no_samples = rxbuf[1];

  return status;

}

// ---------------------------------------------------------------------
// Public function
// Used to get the 6 bytes output sensor FIFO + 1 command status byte.
// @param None
// Returns - command response status byte.
// ---------------------------------------------------------------------
int sh_read_output_fifo(){

  uint8_t ByteSeq[] = {0x12, 0x01};

  int status = max_sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq), &op_fifo_buf[0], (sizeof(op_fifo_buf)), DEFAULT_CMD_SLEEP_US);

  return status;

}

// ---------------------------------------------------------------------
// Public function
// Function to process and dump the sensor values on terminal.
// @param None
// Returns - None
// ---------------------------------------------------------------------
void dump_op_fifo_data(){

  uint16_t heart_rate = 0;
  uint8_t confidence = 0;
  uint16_t Spo2 = 0;
  uint8_t status = 0;

  // Heart Rate formatting
  heart_rate = (uint16_t) op_fifo_buf[1];
  heart_rate= (heart_rate << 8);
  heart_rate |= (op_fifo_buf[2]);
  heart_rate /= 10;
  //LOG_INFO("Heart rate: %d\r\n", heart_rate);

  // Confidence formatting
  confidence = op_fifo_buf[3];
  LOG_INFO("Confidence: %d\r\n", confidence);

  //Blood oxygen level formatting
  Spo2 = (uint16_t) op_fifo_buf[4];
  Spo2 = (Spo2 << 8);
  Spo2 |= op_fifo_buf[5];
  Spo2 /= 10;
  //LOG_INFO("SpO2: %d\r\n", Spo2);

  //"Machine State" - has a finger been detected?
  status = op_fifo_buf[6];
  LOG_INFO("Status: %d\r\n", status);

}

// ---------------------------------------------------------------------
// Public function
// Function to indicate Max3266 finger status on LEDs.
// 0x00: No object detected.
// 0x01: Object detected.
// 0x02: Object other than finger detected.
// 0x03: Finger detected.
// @param None
// Returns - None
// ---------------------------------------------------------------------
void led_finger_status(){

  uint8_t status = op_fifo_buf[6];

  if(status == 0){
      gpioLed0SetOff();
      gpioLed1SetOff();
  }else if(status == 1){//object detected
      gpioLed1SetOn();
      gpioLed0SetOff();
  }else if(status == 2){//object other than finger detected
      gpioLed0SetOn();
      gpioLed1SetOff();
  }else if(status == 3){//finger detected
      gpioLed0SetOn();
      gpioLed1SetOn();
  }

}

// ---------------------------------------------------------------------
// Public function
// Function to get the processed heart rate value.
// @param None
// Returns - heart rate in BPM.
// ---------------------------------------------------------------------
uint16_t get_heart_rate_value(){

  uint16_t heart_rate = 0;

  // Heart Rate formatting
  heart_rate = (uint16_t) op_fifo_buf[1];
  heart_rate= (heart_rate << 8);
  heart_rate |= (op_fifo_buf[2]);
  heart_rate /= 10;
  LOG_INFO("Heart rate: %d\r\n", heart_rate);
  displayPrintf(DISPLAY_ROW_TEMPVALUE, "Pulse rate:%dbpm", heart_rate);

  return heart_rate;
}

// ---------------------------------------------------------------------
// Public function
// Function to get the processed Sp02 value.
// @param None
// Returns - Spo2 level in 0-100% range.
// ---------------------------------------------------------------------
uint16_t get_spo2_value(){

  uint16_t Spo2 = 0;

  //Blood oxygen level formatting
  Spo2 = (uint16_t) op_fifo_buf[4];
  Spo2 = (Spo2 << 8);
  Spo2 |= op_fifo_buf[5];
  Spo2 /= 10;

  LOG_INFO("SpO2: %d\r\n", Spo2);
  displayPrintf(DISPLAY_ROW_8, "SpO2:%d\%",Spo2);

  return Spo2;
}

// ---------------------------------------------------------------------
// Public function
// This function is used to convert the raw temperature from Si7021 to
// degree C and print the result on terminal.
// @param None
// Returns None
// ---------------------------------------------------------------------
void get_result(){

  read_data = ((read_data >> 8) & 0xFF) | ((read_data << 8) & 0xFF00); //Swap bytes
  temp_degree_C = ((175.72 * (read_data) / 65536) - 46.85);

  //LOG_INFO("Temp in C:%d\r\n", temp_degree_C);

  // Print temp on the LCD
  //displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temperature=%d", temp_degree_C);
  displayPrintf(DISPLAY_ROW_ACTION, "Body temp:%d", temp_degree_C);

} //get_result()

// ---------------------------------------------------------------------
// Public function
// This function is used to perform I2C teardown.
// @param None
// Returns None
// ---------------------------------------------------------------------
void I2CTeardown(){

  // Disable Si7021 - Turn Power Off
  //gpioSi7021Disable(); // Commenting for Display A6

  // Disable NVIC for I2C0
  NVIC_DisableIRQ(I2C0_IRQn);

  // reset and disable I2C clock module
  I2C_Reset(I2C0);
  I2C_Enable(I2C0, false);

  // disable SCL and SDA GPIOs
  gpioI2CSDADisable();
  gpioI2CSCLDisable();

  // Turn off I2C clock in CMU

}

