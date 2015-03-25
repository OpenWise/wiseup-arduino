#include <OneWire.h>
#include <DallasTemperature.h>
#include <nRFModule.h>
#include <SPInterface.h>
#include "wise_rfcomm.h"
#include "sensors/sensors.h"
#include "sensors/common.h"
#include "sensors/nrf24l01_receiver_handler.h"
#include "sensors/device_methods.h"

#define DISCOVERY_MODE_INTERVAL		                  120000
#define DISCOVERY_MODE_TIMEOUT                            5000
#define CONNECTED_MODE_READ_SENSORS_INTERVAL              1000
#define CONNECTED_MODE_READ_SENSORS_AS_KEEPALIVE_INTERVAL 20000

#define DIGITAL_RELAY_ADDR_1    1
#define DIGITAL_RELAY_PIN_1     5

uint8_t rx[32];
uint8_t tx[32];

spi_interface 		spi     		  = spi_interface();
NRF24L01    		nrf     		  = NRF24L01(&spi);
WiseRFComm*             net                       = NULL;

rfcomm_data*   		nrfRXPacket 		  = NULL;
rfcomm_data*   		nrfTXPacket 		  = NULL;
rfcomm_device_prot*     prot                      = NULL;
rfcomm_sensor_info*	next_sensor_info_slot 	  = NULL;

#define MAPPING_SIZE    1
sensor_t mapping[] = {
  { DIGITAL_RELAY_ADDR_1,  DIGITAL_RELAY_PIN_1,  RELAY_SENSOR_TYPE, 0 , 5}
};

device_context_t device_context = { mapping, MAPPING_SIZE, DISCOVERY, 0, 0, NO, 0,
                                    {0xFA, 0xFA, 0xFA, 0xFA, 0xFA}, 
                                    {0x05, 0x03, 0x03, 0x02, 0x02},
                                    {0xFA, 0xFA, 0xFA, 0xFA, 0xFA},
                                    millis(), millis(), millis(), millis() };

void
network_layer_broadcast_arrived_handler () {
  Serial.println ("(LIGHT)# Broadcast data [NULL]");
}

void
network_layer_data_arrived_handler () {
  nrf24l01_receiver_handler (nrfRXPacket, &device_context);
}

void
hardware_layer_data_arrived_handler () {
  net->parseRXRawData ();
}

void setup () {
  Serial.begin(57600);
  nrf.init (10, 9); // default csn = 7, ce = 8 (10, 9 on imall board)
  net = new WiseRFComm(&nrf, tx, rx, hardware_layer_data_arrived_handler);
  net->setSender (device_context.local_address);
  net->setDataHandler (network_layer_data_arrived_handler);
  net->setBroadcastHandler (network_layer_broadcast_arrived_handler);
  
  nrfRXPacket = (rfcomm_data *)net->rx_ptr;
  nrfTXPacket = (rfcomm_data *)net->tx_ptr;
  
  pinMode (device_context.mapping_ptr[DIGITAL_RELAY_ADDR_1 - 1].pin, OUTPUT);

  Serial.println("(LIGHT)# Initialized...");
}

void loop () {
  net->listenForIncoming ();
  
  // Change to DISCOVERY mode
  if (abs (millis () - device_context.discovery_interval) > DISCOVERY_MODE_INTERVAL) {
    Serial.println ("(LIGHT)# Change MODE (DISCOVERY)");
    device_context.state = DISCOVERY;
    device_context.discovery_interval = millis ();
  }
  
  // STATE MACHINE
  switch (device_context.state) {
    case DISCOVERY:
      if (abs (millis () - device_context.discovery_timeout) > DISCOVERY_MODE_TIMEOUT) {
        Serial.println ("(LIGHT)# DISCOVERY");
        device_context.state = DISCOVERY;
        memcpy (device_context.server_address, device_context.broadcast_address, 5);
        // Send DISCOVERY packet
        send_discovery (&device_context, net);
        device_context.discovery_timeout = millis ();
      }
    break;
    case CONNECTED:
      // Read sensors data each second
      if (abs (millis () - device_context.connected_read_sensors_interval) > CONNECTED_MODE_READ_SENSORS_INTERVAL) {
        Serial.println ("(LIGHT)# CONNECTED");
        device_context.mapping_ptr[DIGITAL_RELAY_ADDR_1 - 1].value = 
                  (uint16_t) read_digital_relay_info (device_context.mapping_ptr[DIGITAL_RELAY_ADDR_1 - 1].pin);

        // Check for changes
        device_context.is_sync = check_sensors_change (&device_context);
        device_context.connected_read_sensors_interval = millis ();
      }
      
      if (abs (millis () - device_context.connected_read_sensors_as_keepalive_interval) > CONNECTED_MODE_READ_SENSORS_AS_KEEPALIVE_INTERVAL) {
        send_sensors_data (&device_context, net);
        device_context.connected_read_sensors_as_keepalive_interval = millis ();
      }
      
      if (device_context.is_sync == YES) {
        // Send packet with sensors data
        send_sensors_data (&device_context, net);
        device_context.is_sync = NO;
        
        Serial.print ("(LIGHT)# Sensors : ");
        Serial.println ((uint8_t)device_context.mapping_ptr[DIGITAL_RELAY_ADDR_1 - 1].value);
      }
      
      if (device_context.is_res_ack == YES) {
        device_context.mapping_ptr[device_context.res_ack_port - 1].value =
          (uint16_t) read_digital_relay_info (device_context.mapping_ptr[device_context.res_ack_port - 1].pin);
        delay (100);
        send_sensors_data_with_ack (&device_context, net, device_context.res_ack_port);
        device_context.is_res_ack = NO;
        
        Serial.print ("(LIGHT)# Sensors : ");
        Serial.println ((uint8_t)device_context.mapping_ptr[DIGITAL_RELAY_ADDR_1 - 1].value);
      }
    break;
  }
}

