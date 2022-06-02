#pragma once

#include <../../../IOT_TestingCode/FreeRTOS.h>
#include <../../../IOT_TestingCode/task.h>
#include <../../../IOT_TestingCode/event_groups.h>
#include <../../../IOT_TestingCode/queue.h>
#include <stdio.h>
#include "Initializers.h"
#include "SensorDataSharePackage.h"

void applicationTaskRun();
void applicationTask(void* pvParameters);
uint16_t get_humidity_data();
uint16_t get_temperature_data();
uint16_t get_co2_data();
