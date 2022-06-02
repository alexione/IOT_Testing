
#pragma once

#include <../../../IOT_TestingCode/FreeRTOS.h>
#include "../../../IOT_TestingCode/lora_driver.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>


void SensorDataSharePackage_setPackage_lenght(uint8_t lenght);
void SensorDataSharePackage_setTemperature_value(uint16_t value);
void SensorDataSharePackage_setHumidity_value(uint16_t value);
void SensorDataSharePackage_setCo2_value(uint16_t value);
void SensorDataSharePackage_setServo(uint8_t servo_state);
lora_driver_payload_t SensorDataSharePackage_getLoraPayload(uint8_t port_No);
