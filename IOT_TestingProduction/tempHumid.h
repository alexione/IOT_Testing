#pragma once

#include <../../../IOT_TestingCode/FreeRTOS.h>
#include <stdint.h>
#include <stdio.h>
#include <../../../IOT_TestingCode/hih8120.h>
#include <../../../IOT_TestingCode/task.h>
#include <../../../IOT_TestingCode/semphr.h>

#include "Initializers.h"

void TempHumid_measureTask();
void TempHumid_getDataFromSensorTask();
void createTempHumidTask();
void TempHumid_init();
uint16_t get_temperature_data();
uint16_t get_humidity_data();