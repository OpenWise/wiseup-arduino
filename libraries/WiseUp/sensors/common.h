/*
 * Author: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 */

#pragma once

#define YES     0x1
#define NO      0x0

typedef enum {
    DISCOVERY = 0,
    CONNECTED = 1
} device_status_t;

typedef struct sensor {
  uint8_t   address;
  uint8_t   pin;
  uint8_t   type;
  uint16_t  value;
  uint16_t	sensor_update_interval;
} sensor_t;

typedef struct device_context_info {
  sensor_t *        mapping_ptr;
  uint8_t           mapping_size;
  device_status_t   state;
  uint8_t           xor_sensors_value;
  uint8_t           is_sync;
  uint8_t			is_res_ack;
  uint8_t			res_ack_port;
  uint8_t           server_address[5];
  uint8_t           local_address[5];
  uint8_t           broadcast_address[5];
  uint32_t          discovery_timeout;
  uint32_t          discovery_interval;
  uint32_t          connected_read_sensors_interval;
  uint32_t          connected_read_sensors_as_keepalive_interval;
} device_context_t;

void
printBuffer (char* name, uint8_t* buff, int len) {
    Serial.print (name);
    for (int i = 0; i < len; i++) {
        Serial.print (buff[i], HEX);
    } Serial.println ();
}


