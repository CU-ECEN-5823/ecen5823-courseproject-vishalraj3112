/***********************************************************************
 * @file      i2c.h
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

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include <stdint.h>
#include "em_i2c.h"

// macros
#define MEASURE_TEMP_CMD      0xF3
#define SI7021_DEVICE_ADDR    0x40

#define POR_STABILIZATION_TIME        80000
#define SI7021_CONVERSION_TIME        15000

#define MAX32664_DEVICE_ADDR    0xAA
#define SENSOR_ENABLE_SLEEP_US  20000
#define DEFAULT_CMD_SLEEP_US    600//6000

// Function prototypes
void I2C0_init();
void read_si7021_temp();
void I2C_write();
void I2C_read();
void get_result();
void I2CTeardown();
uint32_t* get_temp_val();

//Project Part
int get_device_mode(uint8_t* device_mode);
int get_sensor_hub_status();
int sh_enable(uint8_t index);
int sh_set_fifo_thresh(uint8_t thresh_val);
int sh_set_data_type(uint8_t val);
int sh_enable_algo(uint8_t algo_idx);
int get_sh_no_samples();
int sh_read_output_fifo();
void dump_op_fifo_data();
int sh_enable_maxim_fast(uint8_t mode);
int get_fifo_no_samples(uint8_t* no_samples);
uint16_t get_heart_rate_value();
uint16_t get_spo2_value();

#endif /* SRC_I2C_H_ */
