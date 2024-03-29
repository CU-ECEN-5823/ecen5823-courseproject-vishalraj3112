/*
 * ble.h
 *
 *  Created on: Feb 19, 2023
 *      Author: visha
 */

#ifndef SRC_BLE_H_
#define SRC_BLE_H_
#include <stdbool.h>
#include "sl_bt_api.h"

#define UINT8_TO_BITSTREAM(p, n)        { *(p)++ = (uint8_t)(n); }
#define UINT32_TO_BITSTREAM(p, n)       { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
                                          *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }
#define UINT32_TO_FLOAT(m, e)           (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

// Function prototypes
void handle_ble_event(sl_bt_msg_t *evt);
void send_temp_ble(void);
void send_heart_rate_ble(void);
void send_spo2_ble(void);

// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
 // values that are common to servers and clients
 bd_addr myAddress;
 uint8_t connectionHandle;

 // values unique for server
 // The advertising set handle allocated from Bluetooth stack.
 uint8_t advertisingSetHandle;

 // Connection current state
 bool connection_open;               // true when connection is open
 bool ok_to_send_htm_connections;    // true when client enabled indications
 bool indication_in_flight;          // true when an indication is in flight
 bool ok_to_send_hr_indications;     // true when client enabled heart rate indications
 bool ok_to_send_o2_indications;     // true when client enabled Sp02 indications
 bool bonded;
 bool bonding_started;

 // values unique for client
 uint8_t myAddressType;
 uint8_t packetType;
 uint32_t serviceHandle;
 uint16_t characteristicHandle;

} ble_data_struct_t;

ble_data_struct_t*  get_ble_data_struct(void);

// ----CBFIFO related declarations----

// This is the number of entries in the queue. Please leave
// this value set to 16.
#define QUEUE_DEPTH      (16)
// Student edit:
//   define this to 1 if your design uses all array entries
//   define this to 0 if your design leaves 1 array entry empty
#define USE_ALL_ENTRIES  (1)

// Modern C (circa 2021 does it this way)
// This is referred to as an anonymous struct definition.
// This is the structure of 1 queue/buffer/FIFO entry.
// Please do not change this definition.
//typedef struct {
//  uint8_t       a;
//  uint16_t      b;
//} queue_struct_t;

typedef struct{

  uint8_t buffer[5];
  uint8_t bufferLength;
  uint16_t charType;

} queue_struct_t;

// Function prototypes. The autograder (i.e. the testbench) only uses these
// functions to test your design. Please do not change these definitions.
bool     write_queue (queue_struct_t q);
bool     read_queue (queue_struct_t *q);
uint32_t get_queue_depth (void);
void  display(void);

#endif /* SRC_BLE_H_ */
