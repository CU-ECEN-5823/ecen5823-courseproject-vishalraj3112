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
#define INCLUDE_LOG_DEBUG     1

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

//This should be able to do a multi byte write
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

//Time in MS
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

int get_device_mode(uint8_t* device_mode){

  uint8_t ByteSeq[] = {0x02, 0x00}, rxbuf[2] = {0};

  //makes no sense reading the status if its interrupt driven
  int status = max_sh_read_cmd(ByteSeq, sizeof(ByteSeq), &rxbuf[0], sizeof(rxbuf), DEFAULT_CMD_SLEEP_US);

  *device_mode = rxbuf[1];

  return status;
}

int get_sensor_hub_status(){

  uint8_t ByteSeq[] = {0x00, 0x00}, rxbuf[2] = {0};

  //makes no sense reading the status if its interrupt driven
  max_sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq), &rxbuf[0], sizeof(rxbuf), DEFAULT_CMD_SLEEP_US);

  return (int) rxbuf[1];
}

int sh_set_data_type(uint8_t val){

  uint8_t ByteSeq[] = {0x10, 0x00, val};

  int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), DEFAULT_CMD_SLEEP_US);

  return status;

}

int sh_set_fifo_thresh(uint8_t thresh_val){

  uint8_t ByteSeq[] = {0x10, 0x01, thresh_val};

  int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), DEFAULT_CMD_SLEEP_US);

  return status;

}

int sh_enable(uint8_t index){

    uint8_t ByteSeq[] = {0x44, index, 0x01};

    int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), 3 * SENSOR_ENABLE_SLEEP_US);

    return status;
}

int sh_enable_algo(uint8_t algo_idx){

  uint8_t ByteSeq[] = {0x52, algo_idx, 0x01};

  int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), 3 * SENSOR_ENABLE_SLEEP_US);

  return status;
}

int sh_enable_maxim_fast(uint8_t mode){

  uint8_t ByteSeq[] = {0x52, 0x02, mode};

  int status = max_sh_write_cmd(&ByteSeq[0], sizeof(ByteSeq), 3 * SENSOR_ENABLE_SLEEP_US);//60ms

  return status;

}

int get_sh_no_samples(uint8_t * no_samples){

  uint8_t ByteSeq[] = {0x51, 0x00, 0x03}, rxbuf[2] = {0};

  //makes no sense reading the status if its interrupt driven
  int status = max_sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq), &rxbuf[0], sizeof(rxbuf), DEFAULT_CMD_SLEEP_US);

  *no_samples = rxbuf[1];

  return status;
}

int get_fifo_no_samples(uint8_t* no_samples){

  uint8_t ByteSeq[] = {0x12, 0x00}, rxbuf[2] = {0};

  int status = max_sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq), &rxbuf[0], sizeof(rxbuf), DEFAULT_CMD_SLEEP_US);

  *no_samples = rxbuf[1];

  return status;

}

int sh_read_output_fifo(){

  uint8_t ByteSeq[] = {0x12, 0x01};

  int status = max_sh_read_cmd(&ByteSeq[0], sizeof(ByteSeq), &op_fifo_buf[0], (sizeof(op_fifo_buf)), DEFAULT_CMD_SLEEP_US);

  return status;

}

//Test
void dump_op_fifo_data(){

  uint16_t heart_rate = 0;
  uint8_t confidence = 0;
  uint16_t Spo2 = 0;
  uint8_t status = 0;

//  for(int i=0; i<sizeof(op_fifo_buf); i++){
//      LOG_INFO("%d: %d\r\n", i, op_fifo_buf[i]);
//  }

  // Heart Rate formatting
  heart_rate= (((uint16_t) op_fifo_buf[0]) << 8);
  heart_rate |= (op_fifo_buf[1]);
  heart_rate /= 10;
  LOG_INFO("Heart rate: %d\r\n", heart_rate);

  // Confidence formatting
  confidence = op_fifo_buf[2];
  LOG_INFO("Confidence: %d\r\n", confidence);

  //Blood oxygen level formatting
  Spo2 = ((uint16_t)(op_fifo_buf[3]) << 8);
  Spo2 |= op_fifo_buf[4];
  Spo2 /= 10;
  LOG_INFO("SpO2: %d\r\n", Spo2);

  //"Machine State" - has a finger been detected?
  status = op_fifo_buf[5];
  LOG_INFO("Status: %d\r\n", status);

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

  LOG_INFO("Temp in C:%d\r\n", temp_degree_C);

  // Print temp on the LCD
  displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temperature=%d", temp_degree_C);

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

