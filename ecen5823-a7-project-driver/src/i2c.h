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
#define DEFAULT_CMD_SLEEP_US    2000

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
int get_sh_version(uint8_t* sh_version);
int get_register_attributes(uint8_t* register_attr);
int read_all_max_reg(uint8_t* all_max_reg);
int read_single_max_reg(uint8_t reg_no, uint8_t* reg_val);

int get_sensor_hub_status();
int sh_enable(uint8_t index);
int sh_set_fifo_thresh(uint8_t thresh_val);
int sh_set_data_type(uint8_t val);
int sh_enable_algo(uint8_t algo_idx);
int get_sh_no_samples();
int sh_read_output_fifo();
void dump_op_fifo_data();

#endif /* SRC_I2C_H_ */
