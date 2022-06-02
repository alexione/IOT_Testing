#pragma once

#include <../../../IOT_TestingCode/FreeRTOS.h>
#include <stdio.h>
#include <../../../IOT_TestingCode/mh_z19.h>
#include <../../../IOT_TestingCode/event_groups.h>
#include "Initializers.h"


extern mh_z19_returnCode_t rc;

void Co2_measureTask();
void Co2_getDataFromSensorTask();
uint16_t get_co2_data();
void Co2_init();
void myCo2CallBack(uint16_t* ppm);
void createCo2Task();

