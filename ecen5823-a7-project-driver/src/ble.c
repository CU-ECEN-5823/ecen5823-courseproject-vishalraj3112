/***********************************************************************
 * @file      scheduler.c
 * @version   0.1
 * @brief     Function implementation file.
 *
 * @author    Vishal Raj, vishal.raj@colorado.edu
 * @date      Feb 19, 2023
 *
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823-001: IoT Embedded Firmware
 * @instructor  David Sluiter
 *
 * @assignment Assignment
 * @due
 *
 * @resources Silicon Labs 'Bluetooth - Soc Thermometer' example project
 *            & https://docs.silabs.com/bluetooth/latest/.
 *
 */
#define INCLUDE_LOG_DEBUG     1

#include "ble.h"
#include "log.h"
#include "gatt_db.h"
#include "i2c.h"
#include "scheduler.h"
#include "lcd.h"
#include "ble_device_type.h"
#include <math.h>

// Server relate
#define ENABLE_BLE_LOGS                    0
#define MIN_ADVERTISE_INTERVAL             250
#define MIN_ADVERTISE_INTERVAL_VAL         (MIN_ADVERTISE_INTERVAL)/0.625
#define MAX_ADVERTISE_INTERVAL             250
#define MAX_ADVERTISE_INTERVAL_VAL         (MAX_ADVERTISE_INTERVAL)/0.625
#define MIN_CONNECTION_INTERVAL            75 //(60 * 1.25 = 75ms)
#define MIN_CONNECTION_INTERVAL_VAL        (MIN_CONNECTION_INTERVAL/1.25)
#define MAX_CONNECTION_INTERVAL            75 //(60 * 1.25 = 75ms)
#define MAX_CONNECTION_INTERVAL_VAL        (MAX_CONNECTION_INTERVAL/1.25)
#define MAX_LATENCY                        300   //(4 * 75 = 300ms)
#define MAX_LATENCY_VAL                    (MAX_LATENCY/MAX_CONNECTION_INTERVAL)
//#define MAX_TIMEOUT                        150 //(1 + 4) * 75 * 2 = 750, taking twice of that i.e 1500, but step size is 10ms
#define MAX_TIMEOUT_VAL                    ((1 + MAX_LATENCY_VAL) * (MAX_CONNECTION_INTERVAL * 2) + MAX_CONNECTION_INTERVAL) ///!!Check value with proff, suggested slightly larger

// Client related
#define SCAN_PASSIVE                        0
#define SCAN_INTERVAL                       50 //ms
#define SCAN_INTERVAL_VAL                   (SCAN_INTERVAL/0.625)
#define SCAN_WINDOW                         25 //ms
#define SCAN_WINDOW_VAL                     (SCAN_WINDOW/0.625)
#define MIN_CE_LENGTH                       0
#define MAX_CE_LENGTH                       4
#define REQ_CONNECTION_HANDLE               1

// BLE private data
static ble_data_struct_t ble_data;
int32_t FLOAT_TO_INT32(const uint8_t *value_start_little_endian);

// The advertising set handle allocated from Bluetooth stack.
#if (DEVICE_IS_BLE_SERVER == 1)
static uint8_t advertising_set_handle = 0xff;
#endif

// Global variable for cbfifo
// Declare memory for the queue/buffer/fifo, and the write and read pointers
queue_struct_t   my_queue[QUEUE_DEPTH]; // the queue - an array of structs
uint32_t         wptr = -1;//rear              // write pointer
uint32_t         rptr = -1;//front              // read pointer

static bool is_empty(void);

// ---------------------------------------------------------------------
// Public function
// This function is used to return the ble private data structure
// @param None
// Returns ble data structure
// ---------------------------------------------------------------------
ble_data_struct_t*  get_ble_data_struct(void){

    return &ble_data;
}

// ---------------------------------------------------------------------
// Public function
// This function is the bluetooth event handler
// @param Ble event context sl_bt_msg_t
// Returns None
// ---------------------------------------------------------------------
void handle_ble_event(sl_bt_msg_t *evt){

  sl_status_t rc;
  bd_addr address;
  uint8_t address_type;

#if (DEVICE_IS_BLE_SERVER == 0)
  uint8_t serverAddress[] = SERVER_BT_ADDRESS;
  uint8_t* char_value;
#endif

  // BT-Stack event handler
  switch (SL_BT_MSG_ID(evt->header)) {

#if (DEVICE_IS_BLE_SERVER == 0)

      /*This event is received when the device has started and
      the radio is ready.*/
      case sl_bt_evt_system_boot_id:

        #if ENABLE_BLE_LOGS
        // Print boot message.
        LOG_INFO("Bluetooth stack booted: v%d.%d.%d-b%d\n\r",
                     evt->data.evt_system_boot.major,
                     evt->data.evt_system_boot.minor,
                     evt->data.evt_system_boot.patch,
                     evt->data.evt_system_boot.build);
        #endif

        /*1. Set the scan mode on the specified PHY(s).*/
        rc = sl_bt_scanner_set_mode(sl_bt_gap_1m_phy, SCAN_PASSIVE);
        if(rc != SL_STATUS_OK){
            LOG_ERROR("Bluetooth: Scanner set mode error = %d\r\n", (unsigned int) rc);
        }

        /*2. Set the scanning timing parameters on the specified PHY(s).*/
        rc = sl_bt_scanner_set_timing(sl_bt_gap_1m_phy, SCAN_INTERVAL_VAL, SCAN_WINDOW_VAL);
        if(rc != SL_STATUS_OK){
            LOG_ERROR("Bluetooth: Scanner set timing error = %d\r\n", (unsigned int) rc);
        }

        /*3. Sets default connection parameters for all subsequent connections.*/
        rc = sl_bt_connection_set_default_parameters(
                MIN_CONNECTION_INTERVAL_VAL,
                MAX_CONNECTION_INTERVAL_VAL,
                MAX_LATENCY_VAL,
                MAX_TIMEOUT_VAL,
                MIN_CE_LENGTH,
                MAX_CE_LENGTH);

        if(rc != SL_STATUS_OK){
            LOG_ERROR("Bluetooth: Set connection default parameters error = %d\r\n", (unsigned int) rc);
        }

        /*4. Start GAP discovery procedure to scan for advertising devices.*/
        rc = sl_bt_scanner_start(sl_bt_gap_1m_phy , sl_bt_scanner_discover_generic);
        if(rc != SL_STATUS_OK){
            LOG_ERROR("Bluetooth: Scanner start error = %d\r\n", (unsigned int) rc);
        }

        #if ENABLE_BLE_LOGS
        LOG_INFO("Scanning started.\r\n");
        #endif

        /*5. Initialize the display.*/
        displayInit();

        /*6. Read the bluetooth identity address used by the device */
        rc = sl_bt_system_get_identity_address(&address, &address_type);
        if(rc != SL_STATUS_OK){
            LOG_ERROR("Bluetooth: Get Identity error = %d\r\n", (unsigned int) rc);
        }

        /*7. Add LCD prints*/
        displayPrintf(DISPLAY_ROW_NAME, BLE_DEVICE_TYPE_STRING);
        displayPrintf(DISPLAY_ROW_BTADDR, "%X:%X:%X:%X:%X:%X",address.addr[0], address.addr[1],
                        address.addr[2], address.addr[3], address.addr[4], address.addr[5]);
        displayPrintf(DISPLAY_ROW_ASSIGNMENT, ASSIGNMENT_NUMBER);
        displayPrintf(DISPLAY_ROW_CONNECTION, DISCOVERING_STRING);

        break;

      /* This event is generated when a new connection is established.*/
      case sl_bt_evt_connection_opened_id:

        #if ENABLE_BLE_LOGS
        LOG_INFO("New connection opened...\r\n");
        #endif

        /*1. Save connection Handle*/
        ble_data.connectionHandle = evt->data.evt_connection_opened.connection;

        /*2. Add LCD prints*/
        displayPrintf(DISPLAY_ROW_CONNECTION, CONNECTED_STRING);
        displayPrintf(DISPLAY_ROW_BTADDR2, "%X:%X:%X:%X:%X:%X",serverAddress[0], serverAddress[1],
                      serverAddress[2], serverAddress[3], serverAddress[4], serverAddress[5]);

        break;

      /*This event indicates that a connection was closed.*/
      case sl_bt_evt_connection_closed_id:

       #if ENABLE_BLE_LOGS
       LOG_INFO("Connection closed!\r\n");
       #endif

       /*1. Restart GAP discovery procedure to scan for advertising devices.*/
       rc = sl_bt_scanner_start(sl_bt_gap_1m_phy , sl_bt_scanner_discover_generic);
       if(rc != SL_STATUS_OK){
           LOG_ERROR("Bluetooth: Scanner start error = %d\r\n", (unsigned int) rc);
       }

       /*2. Add LCD prints*/
       displayPrintf(DISPLAY_ROW_CONNECTION, DISCOVERING_STRING);
       displayPrintf(DISPLAY_ROW_BTADDR2, " ");
       displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");

       break;

      /* Received for advertising or scan response packets generated by: sl_bt_scanner_start().*/
      case sl_bt_evt_scanner_scan_report_id:

        #if ENABLE_BLE_LOGS
        //LOG_INFO("Scan response received.\r\n");
        #endif

        /*1. Check for bd_addr, packet_type and address_type from the server*/
        ble_data.myAddress = evt->data.evt_scanner_scan_report.address;
        ble_data.myAddressType = evt->data.evt_scanner_scan_report.address_type;
        ble_data.packetType = evt->data.evt_scanner_scan_report.packet_type;

        if(memcmp((void *) serverAddress, (void *) ble_data.myAddress.addr , sizeof(ble_data.myAddress)) == 0){
//        if(serverAddress[0] == ble_data.myAddress.addr[0] && serverAddress[1] == ble_data.myAddress.addr[1] && serverAddress[2] == ble_data.myAddress.addr[2]
//                          && serverAddress[3] == ble_data.myAddress.addr[3] && serverAddress[4] == ble_data.myAddress.addr[4] && serverAddress[5] == ble_data.myAddress.addr[5]){

            /* Public address.*/
            if(ble_data.myAddressType == 0){

                if(ble_data.packetType == 0){

                    /*2. Stop Scanning*/
                    rc = sl_bt_scanner_stop();
                    if(rc != SL_STATUS_OK){
                        LOG_ERROR("Bluetooth: Scanner Stop error = %d\r\n", (unsigned int) rc);
                    }

                    /*3. Open connection with the server*/
                    rc = sl_bt_connection_open(ble_data.myAddress,
                                               ble_data.myAddressType,
                                               sl_bt_gap_phy_1m,
                                               (uint8_t *)REQ_CONNECTION_HANDLE);
                    if(rc != SL_STATUS_OK){
                        LOG_ERROR("Bluetooth: Connection open error = %d\r\n", (unsigned int) rc);
                    }else{
                        #if ENABLE_BLE_LOGS
                        LOG_INFO("Connection with server successful!\r\n");
                        #endif
                    }
                }
            }
        }


        break;

      /*We get this event when it’s ok to call the next GATT command.*/
      case sl_bt_evt_gatt_procedure_completed_id:

        #if ENABLE_BLE_LOGS
        LOG_INFO("GATT Command completed.\r\n");
        #endif

        break;

      /*This event is received when a GATT service in the remote GATT database was discovered. */
      case sl_bt_evt_gatt_service_id:

        #if ENABLE_BLE_LOGS
        LOG_INFO("GATT service in remote GATT database discovered.\r\n");
        #endif

        /*1. Save the GATT HTM service handle.*/
        ble_data.serviceHandle = evt->data.evt_gatt_service.service;

        break;

      /*This event is received when a GATT characteristic in the remote GATT database was discovered. */
      case sl_bt_evt_gatt_characteristic_id:

        #if ENABLE_BLE_LOGS
        LOG_INFO("GATT characteristic in remote GATT database discovered.\r\n");
        #endif

        /*1. Save the GATT HTM characteristic handle.*/
        ble_data.characteristicHandle = evt->data.evt_gatt_characteristic.characteristic;

        break;

      /*If an indication or notification has been enabled for a characteristic, this event is triggered
       * whenever an indication or notification is received from the remote GATT server */
      case sl_bt_evt_gatt_characteristic_value_id:

        #if ENABLE_BLE_LOGS
        LOG_INFO("Characteristic value received from remote.\r\n");
        #endif

        /*Is the characteristic handle and att_opcode we expect?*/
        if(evt->data.evt_gatt_characteristic_value.characteristic == gattdb_temperature_measurement &&
            evt->data.evt_gatt_characteristic_value.att_opcode == gatt_handle_value_indication ){

            /*1. Send indication confirmation*/
            rc = sl_bt_gatt_send_characteristic_confirmation(ble_data.connectionHandle);
            if(rc != SL_STATUS_OK){
                LOG_ERROR("Bluetooth: Sending characteristic confirmation error = %d\r\n", (unsigned int) rc);
            }

            /*2. Convert the temperature to be displayed on Client*/
            char_value = &(evt->data.evt_gatt_characteristic_value.value.data[0]);
            int32_t final_temp_val = FLOAT_TO_INT32(char_value);

            /*3. Add LCD prints*/
            displayPrintf(DISPLAY_ROW_CONNECTION, HANDLING_STRING);
            displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temperature=%d", final_temp_val);

        }

        break;

    /*Indicates that a soft timer has lapsed.*/
    case sl_bt_evt_system_soft_timer_id:

      /*Update the display @ 1Hz*/
      displayUpdate();

      break;

#elif (DEVICE_IS_BLE_SERVER == 1)
    /*This event is received when the device has started and
    the radio is ready.*/
    case sl_bt_evt_system_boot_id:

      #if ENABLE_BLE_LOGS
      // Print boot message.
      LOG_INFO("Bluetooth stack booted: v%d.%d.%d-b%d\n\r",
                   evt->data.evt_system_boot.major,
                   evt->data.evt_system_boot.minor,
                   evt->data.evt_system_boot.patch,
                   evt->data.evt_system_boot.build);
      #endif

      /* Update the connection state*/
      ble_data.connection_open = false;

      /* 1. Read the bluetooth identity address used by the device */
      rc = sl_bt_system_get_identity_address(&address, &address_type);
      if(rc != SL_STATUS_OK){
          LOG_ERROR("Bluetooth: Get Identity error = %d\r\n", (unsigned int) rc);
      }

      /* 2. Create bluetooth advertiser set */
      rc = sl_bt_advertiser_create_set(&advertising_set_handle);
      if(rc != SL_STATUS_OK){
          LOG_ERROR("Bluetooth: Create advertiser set error = %d\r\n", (unsigned int) rc);
      }

      /* 3. Set advertising interval to 100ms. */
      rc = sl_bt_advertiser_set_timing(
            advertising_set_handle, // advertising set handle
            MIN_ADVERTISE_INTERVAL_VAL, // min. adv. interval (milliseconds * 1.6)
            MAX_ADVERTISE_INTERVAL_VAL, // max. adv. interval (milliseconds * 1.6)
            0,   // adv. duration
            0);  // max. num. adv. events
      if(rc != SL_STATUS_OK){
          LOG_ERROR("Bluetooth: Set advertiser timing error = %d\r\n", (unsigned int) rc);
      }

      /* 4. Start advertising on the advertising set with the specified
       * discovery and connection modes.*/
      rc = sl_bt_advertiser_start(
            advertising_set_handle,
            sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
      if(rc != SL_STATUS_OK){
          LOG_ERROR("Bluetooth: Advertiser start error = %d\r\n", (unsigned int) rc);
      }

      /* 5. Initialize the display.*/
      displayInit();

      /* 6. Add LCD prints*/
      displayPrintf(DISPLAY_ROW_NAME, BLE_DEVICE_TYPE_STRING);
      displayPrintf(DISPLAY_ROW_BTADDR, "%X:%X:%X:%X:%X:%X",address.addr[0], address.addr[1],
                    address.addr[2], address.addr[3], address.addr[4], address.addr[5]);
      displayPrintf(DISPLAY_ROW_ASSIGNMENT, ASSIGNMENT_NUMBER);
      displayPrintf(DISPLAY_ROW_CONNECTION, ADVERTISING_STRING);

      #if ENABLE_BLE_LOGS
      LOG_INFO("Advertising started...\r\n");
      #endif

      break;

    /*Indication of a new connection opening.*/
    case sl_bt_evt_connection_opened_id:

      #if ENABLE_BLE_LOGS
      LOG_INFO("New connection opened...\r\n");
      #endif

      /*Save connection Handle*/
      ble_data.connectionHandle = evt->data.evt_connection_opened.connection;

      /*1. Stop the advertisement*/
      rc = sl_bt_advertiser_stop(advertising_set_handle);
      if(rc != SL_STATUS_OK){
          LOG_ERROR("Bluetooth: Advertiser stop error = %d\r\n", (unsigned int) rc);
      }

      /* Update the connection state*/
      ble_data.connection_open = true;

      /* Save the connection handle*/
      ble_data.advertisingSetHandle = advertising_set_handle;

      /*2. Set connection parameters*/
      rc = sl_bt_connection_set_parameters(ble_data.connectionHandle,
                                           MIN_CONNECTION_INTERVAL_VAL,
                                           MAX_CONNECTION_INTERVAL_VAL,
                                           MAX_LATENCY_VAL,
                                           MAX_TIMEOUT_VAL,
                                           0,
                                           0xFFFF);
      if(rc != SL_STATUS_OK){
          LOG_ERROR("Bluetooth: Connection set parameter = 0x%X\r\n", (unsigned int) rc);
      }

      /*3. Add LCD prints*/
      displayPrintf(DISPLAY_ROW_CONNECTION, CONNECTED_STRING);

      break;

    /*This event indicates that a connection was closed*/
    case sl_bt_evt_connection_closed_id:

      #if ENABLE_BLE_LOGS
      LOG_INFO("Connection closed!\r\n");
      #endif
      /*1. Set the connection lost event*/
      schedulerSetConnectionLostEvent();

      /*2. Set current connection open status & ok_to_send_indication to false*/
      ble_data.connection_open = false;
      ble_data.ok_to_send_htm_connections = false;
      ble_data.indication_in_flight = false;
      ble_data.ok_to_send_hr_indications = false;

      /*3. Start advertising on the advertising set with the specified
             * discovery and connection modes.*/
      rc = sl_bt_advertiser_start(
            advertising_set_handle,
            sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
      if(rc != SL_STATUS_OK){
          LOG_ERROR("Bluetooth: Advertiser start error = %d\r\n", (unsigned int) rc);
      }

      /*4. Add LCD prints.*/
      displayPrintf(DISPLAY_ROW_CONNECTION, ADVERTISING_STRING);

      #if ENABLE_BLE_LOGS
      LOG_INFO("Advertising started...\r\n");
      #endif

      break;

    /*Informational. Triggered whenever the connection parameters are changed
     * and at any time a connection is established*/
    case sl_bt_evt_connection_parameters_id:

      //sl_bt_evt_connection_parameters_t conn_params = evt->data.evt_connection_parameters;

      #if ENABLE_BLE_LOGS
      LOG_INFO("Connection parameters changed/connection established\r\n");


      LOG_INFO("Ble connection parameters:\n\r");
      LOG_INFO("HANDLE - %d\r\n", evt->data.evt_connection_parameters.connection);
      int interval = (int) (evt->data.evt_connection_parameters.interval*1.25);
      LOG_INFO("INTERVAL - %d\r\n", interval);
      LOG_INFO("LATENCY - %d\r\n", evt->data.evt_connection_parameters.latency);
      LOG_INFO("TIMEOUT - %d\r\n", evt->data.evt_connection_parameters.timeout*10);
      #endif

      break;

     /*This event indicates that sl_bt_external_signal(myEvent) was called and returns the myEvent
      * value in the event data structure: evt->data.evt_system_external_signal.extsignals*/
     case sl_bt_evt_system_external_signal_id:
       break;

     /*Indicates either:
      A local Client Characteristic Configuration descriptor (CCCD) was changed by the remote GATT client, or
      That a confirmation from the remote GATT Client was received upon a successful reception of the indication
      I.e. we sent an indication from our server to the client with sl_bt_gatt_server_send_indication()*/
     case sl_bt_evt_gatt_server_characteristic_status_id:

       #if ENABLE_BLE_LOGS
       LOG_INFO("Local CCCD changed or Indication confimation received\r\n");
       #endif

       /*For temperature measurement characteristic, see if client characteristic configuration changed*/
       if((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
           && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)){

           /*Check if indications were enabled from the client*/
           if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02){
               ble_data.ok_to_send_htm_connections = true;
           }

           /*Check if indications were disabled from the client*/
           if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x00){
               ble_data.ok_to_send_htm_connections = false;
           }

       }

       /*For heart rate measurement characteristic, see if client characteristic configuration changed*/
       if((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_heart_rate_state)
           && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)){

           /*Check if indications were enabled from the client*/
           if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02){
               ble_data.ok_to_send_hr_indications = true;
           }

           /*Check if indications were disabled from the client*/
           if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x00){
               ble_data.ok_to_send_hr_indications = false;
           }

       }

       /*A confirmation from the remote GATT Client was received upon a successful reception of the indication*/
       if((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
                  && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x02)){
           ble_data.indication_in_flight = false;
       }

       /*A confirmation from the remote GATT Client was received upon a successful reception of the indication*/
       if((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_heart_rate_state)
                  && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x02)){
           ble_data.indication_in_flight = false;
       }


       break;

     /*Possible event from calling sl_bt_gatt_server_send_indication() - i.e. we never received a confirmation
      *  for a previously transmitted indication.*/
     case sl_bt_evt_gatt_server_indication_timeout_id:

       #if ENABLE_BLE_LOGS
       LOG_INFO("Confirmation not received for a previously transmitted event\r\n");
       #endif

       /*Check if an indication is in flight, If yes then reset it since a timeout has occurred*/
       if(ble_data.indication_in_flight == true){
           ble_data.indication_in_flight = false;
       }

       break;

    /*Indicates that a soft timer has lapsed.*/
    case sl_bt_evt_system_soft_timer_id:

      /*1. Update the display @ 1Hz*/
      displayUpdate();

      /* 2. If no indication is in flight, then send the remaining commands
       * in the queue to client*/
      if(ble_data.connection_open == true &&
          ble_data.indication_in_flight == false &&
          is_empty() == false){

          queue_struct_t entry;

          if(read_queue(&entry) == false){//read success

            /*Send the indication since there is no pending GATT command.*/
            rc = sl_bt_gatt_server_send_indication(
                    ble_data.connectionHandle,
                    entry.charType, // handle from gatt_db.h
                    entry.bufferLength,
                    entry.buffer // pointer to where data is
                    );
            if (rc != SL_STATUS_OK) {
                LOG_ERROR("Error Sending client data from queue:%X\r\n", rc);
            } else {
               //Set indication_in_flight flag
                ble_data.indication_in_flight = true;
            }

          }else{
              LOG_ERROR("Error: Queue dequeue failed!\r\n");
          }

          //displayPrintf(DISPLAY_ROW_10, "QD = %d", get_queue_depth());

      }

      break;
#endif

  }// switch

}

// ---------------------------------------------------------------------
// Public function
// This function is used to write the Gatt DB and send the final
// temperature indication to the client.
// @param None
// Returns None
// ---------------------------------------------------------------------
void send_temp_ble(){

  uint32_t temperature_in_c, *temp;
  sl_status_t rc;

  uint8_t htm_temperature_buffer[5];
  uint8_t *p = htm_temperature_buffer;
  uint32_t htm_temperature_flt;
  uint8_t flag_byte = 0;

  temp = get_temp_val();
  temperature_in_c = *temp;

  // -------------------------------
  // Write our local GATT DB
  // -------------------------------
  rc = sl_bt_gatt_server_write_attribute_value(
         gattdb_temperature_measurement, // handle from gatt_db.h
         0, // offset
         4, // length
         (uint8_t *)&temperature_in_c // pointer to buffer where data is
         );
  if (rc != SL_STATUS_OK) {
      LOG_ERROR("Error writing GATT DB:%X\r\n", rc);
  }

  /*Perform conversions*/
  htm_temperature_flt = UINT32_TO_FLOAT(temperature_in_c*1000, -3);

  /*Convert temperature to bitstream and place it in the htm_temperature_buffer
   * along with flag byte.*/
  UINT8_TO_BITSTREAM(p, flag_byte);
  UINT32_TO_BITSTREAM(p, htm_temperature_flt);

  /*Send the data to client if the following conditions are met:
   * 1. Indication for the given characteristic have been enabled by the client.
   * 2. Connection for the current handle is open.
   * 3. There is no connection, which is already in flight.*/
  if(ble_data.ok_to_send_htm_connections == true
      && ble_data.connection_open == true
      ){

    if(ble_data.indication_in_flight == false){

      /*Send the indication since there is no pending GATT command.*/
      rc = sl_bt_gatt_server_send_indication(
              ble_data.connectionHandle,
              gattdb_temperature_measurement, // handle from gatt_db.h
              5,
              &htm_temperature_buffer[0] // in IEEE-11073 format
              );
      if (rc != SL_STATUS_OK) {
          LOG_ERROR("Error Sending client data:%X\r\n", rc);
      } else {
         //Set indication_in_flight flag
          ble_data.indication_in_flight = true;
      }

   }else{/*enqueue the current contents in queue*/

     queue_struct_t entry = {0};
     memcpy(entry.buffer, htm_temperature_buffer, sizeof(htm_temperature_buffer));
     entry.bufferLength = sizeof(htm_temperature_buffer);
     entry.charType = gattdb_temperature_measurement;

     if(write_queue(entry) == false){
         LOG_INFO("Event enqueued in queue, current Qdepth:%d\r\n", get_queue_depth());
         //displayPrintf(DISPLAY_ROW_10, "QD = %d", get_queue_depth()); // Remove this
     }else{
         LOG_ERROR("Error: Queue push failed, queue full!\r\n");
     }

   }

  }

}

void send_heart_rate_ble(){

  //1. Get the current heart rate value
  uint16_t heart_rate = get_heart_rate_value();
  sl_status_t rc;

  //2.
  // -------------------------------
  // Write our local GATT DB
  // -------------------------------
  rc = sl_bt_gatt_server_write_attribute_value(
         gattdb_heart_rate_state, // handle from gatt_db.h
         0, // offset
         2, // length
         (uint8_t *)&heart_rate // pointer to value
         );
  if (rc != SL_STATUS_OK) {
      LOG_ERROR("Error writing heart rate to GATT DB:%X\r\n", rc);
  }

  /*Send the data to client if the following conditions are met:
   * 1. Indication for the given characteristic have been enabled by the client.
   * 2. Connection for the current handle is open.
   * 3. There is no connection, which is already in flight.*/
  if(ble_data.ok_to_send_hr_indications == true &&
      ble_data.connection_open == true
      ){

      /*If no indication is in flight send immediately*/
      if(ble_data.indication_in_flight == false){

      rc = sl_bt_gatt_server_send_indication(
              ble_data.connectionHandle,
              gattdb_heart_rate_state, // handle from gatt_db.h
              2,
              (uint8_t *)&heart_rate
              );
      if (rc != SL_STATUS_OK) {
          LOG_ERROR("Error Sending heart rate to client:%X\r\n", rc);
      } else {
         //Set indication_in_flight flag
          ble_data.indication_in_flight = true;
      }
    }else{ /*enqueue the current contents in queue*/

      queue_struct_t entry = {0};
      memcpy(entry.buffer, &heart_rate, sizeof(heart_rate));
      entry.bufferLength = sizeof(heart_rate);
      entry.charType = gattdb_heart_rate_state;

      if(write_queue(entry) == false){
          LOG_INFO("Event enqueued in queue, current Qdepth:%d\r\n", get_queue_depth());
          //displayPrintf(DISPLAY_ROW_10, "QD = %d", get_queue_depth()); // Remove this
      }else{
          LOG_ERROR("Error: Queue push failed, queue full!\r\n");
      }

    }
  }

}

// -----------------------------------------------
// Private function, original from Dan Walkes. I fixed a sign extension bug.
// We'll need this for Client A7 assignment to convert health thermometer
// indications back to an integer. Convert IEEE-11073 32-bit float to signed integer.
// -----------------------------------------------
int32_t FLOAT_TO_INT32(const uint8_t *value_start_little_endian)
{
   uint8_t signByte = 0;
   int32_t mantissa;

   // input data format is:
   // [0] = flags byte
   // [3][2][1] = mantissa (2's complement)
   // [4] = exponent (2's complement)
   // BT value_start_little_endian[0] has the flags byte

   int8_t exponent = (int8_t)value_start_little_endian[4];
   // sign extend the mantissa value if the mantissa is negative

   if (value_start_little_endian[3] & 0x80) { // msb of [3] is the sign of the mantissa
       signByte = 0xFF;
   }

   mantissa = (int32_t) (value_start_little_endian[1] << 0) |
       (value_start_little_endian[2] << 8) |
       (value_start_little_endian[3] << 16) |
       (signByte << 24) ;

   // value = 10^exponent * mantissa, pow() returns a double type
   return (int32_t) (pow(10, exponent) * mantissa);

}

// ---------------------------------------------------------------------
// Private function used only by this .c file.
// Checks if the queue is currently empty
// Returns true if the queue is full, otherwise false
// ---------------------------------------------------------------------
bool is_empty(void){

  if(rptr == (uint32_t) -1)
    return true;

  return false;
}

// ---------------------------------------------------------------------
// Private function used only by this .c file.
// Checks if the queue is currently full for rptr > wptr & wptr > rptr.
// Returns true if the queue is full, false otherwise
// ---------------------------------------------------------------------
bool is_full(void){

  if(((rptr == 0) && (wptr == QUEUE_DEPTH - 1)) || (wptr == rptr - 1))
    return true;

  return false;
}

// ---------------------------------------------------------------------
// Public function used only by this .c file.
// Used to display the current contents of the queue.
// Returns nothing.
// ---------------------------------------------------------------------
void display(void){

  printf("Queue:\n");
    for(int i = 0; i < QUEUE_DEPTH; i++){
        //printf("%d %d\n",my_queue[i].a, my_queue[i].b);
    }
    printf("\n");
}

// ---------------------------------------------------------------------
// Private function used only by this .c file.
// Compute the next ptr value. Given a valid ptr value, compute the next valid
// value of the ptr and return it.
// Isolation of functionality: This defines "how" a pointer advances.
// ---------------------------------------------------------------------
uint32_t nextPtr(uint32_t ptr) {

  uint32_t ret_val = (ptr + 1) % QUEUE_DEPTH; //circularly increase index by one

  return ret_val;

} // nextPtr()

// ---------------------------------------------------------------------
// Public function
// This function writes an entry to the queue if the the queue is not full.
// Input parameter "a" should be written to queue_struct_t element "a"
// Input parameter "b" should be written to queue_struct_t element "b"
// Returns bool false if successful or true if writing to a full fifo.
// i.e. false means no error, true means an error occurred.
// ---------------------------------------------------------------------
bool write_queue (queue_struct_t q) {

  if(is_full()){
    LOG_INFO("Queue full!\n\r");
    return true;
  }else{

    //for first entry
    if(rptr == (uint32_t) -1)
      rptr = 0;

    //nextPtr logic
    wptr = nextPtr(wptr);

    //fill the entries
    my_queue[wptr] = q;

  }

  return false;//ok case

} // write_queue()

// ---------------------------------------------------------------------
// Public function
// This function reads an entry from the queue.
// Write the values of a and b from my_queue[rptr] to the memory addresses
// pointed at by *a and *b. In this implementation, we do it this way because
// standard C does not provide a mechanism for a C function to return multiple
// values, as is common in perl or python.
// Returns bool false if successful or true if reading from an empty fifo.
// i.e. false means no error, true means an error occurred.
// ---------------------------------------------------------------------
bool read_queue (queue_struct_t *q) {

  if(is_empty()){

    printf("Queue empty!\n");
    return true;

  }else{

    *q = my_queue[rptr];

    //for last element, reset read and write pointer
    if(rptr == wptr){
      wptr = -1;
      rptr = -1;
    }else{
      //nextPtr logic
      rptr = nextPtr(rptr);
    }
  }

  return false;

} // read_queue()

// ---------------------------------------------------------------------
// Public function
// This function returns the wptr, rptr, full and empty values, writing
// to memory using the pointer values passed in, same rationale as read_queue()
// ---------------------------------------------------------------------
void get_queue_status (uint32_t *_wptr, uint32_t *_rptr, bool *_full, bool *_empty) {

  *_wptr = wptr;
  *_rptr = rptr;

  *_full = is_full();
  *_empty = is_empty();

} // get_queue_status()

// ---------------------------------------------------------------------
// Public function
// Function that computes the number of written entries currently in the queue. If there
// are 3 entries in the queue, it should return 3. If the queue is empty it should
// return 0. If the queue is full it should return either QUEUE_DEPTH if
// USE_ALL_ENTRIES==1 otherwise returns QUEUE_DEPTH-1.
// ---------------------------------------------------------------------
uint32_t get_queue_depth() {

  //3 cases:
  //1. When wptr == -1, empty queue
  if(wptr == (uint32_t) -1){
    return 0;
  }else if(wptr >= rptr){//2. When wptr > rptr
    return(wptr - rptr + 1);
  }else if(rptr > wptr){//2. When rptr > wptr
    return (QUEUE_DEPTH - (rptr - wptr) + 1);
  }

  //Added to prevent warnings
  return 0;
} // get_queue_depth()

