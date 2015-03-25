/*
 * Author: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 */

#pragma once

#include <OneWire.h>
#include <DallasTemperature.h>
#include <stdint.h>

float 
read_analog_temperature_info (uint8_t pin) {
  int     tempSum     = 0;
  float   temperature = 0;
  int     i;

  /* Read data from sensor 10 times */
  for (i = 0; i < 3; i++) {
    tempSum += analogRead (pin);
  }

  temperature = (tempSum * 0.48828125) / 3;
  return temperature;
}

float 
read_digital_temperature_info (DallasTemperature * sensors) {
  float   temperature = 0;  

  sensors->requestTemperatures();
  temperature = sensors->getTempCByIndex(0);

  return temperature;
}

float 
read_analog_luminance_info (uint8_t pin, uint8_t inverse) {
  int   lumSum     = 0;
  float luminance  = 0;
  int   i;

  /* Fill the sensor info fields */
  for (i = 0; i < 3; i++) {
    lumSum += analogRead (pin);
  }

  luminance = (lumSum * 0.48828125) / 3;
  luminance = (luminance / 512) * 100;
  
  if (inverse) {
	luminance = 100 - luminance;
  }

  return luminance;
}

uint8_t 
read_digital_pir_info (uint8_t pin) {
  return digitalRead (pin);
}
  
uint8_t 
read_digital_relay_info (uint8_t pin) {
  return digitalRead (pin);
}

uint8_t 
write_digital_relay_info (uint8_t pin, uint8_t data) {
  if (data == 1) {
    digitalWrite (pin, HIGH);
  } else {
    digitalWrite (pin, LOW);
  }
  
  return read_digital_relay_info (pin);
}

