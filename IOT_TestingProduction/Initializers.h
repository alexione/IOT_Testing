#pragma once

#include <../../../IOT_TestingCode/FreeRTOS.h>
#include <../../../IOT_TestingCode/event_groups.h>
#include <../../../IOT_TestingCode/queue.h>
#include <../../../IOT_TestingCode/semphr.h>
#include "../../../IOT_TestingCode/message_buffer.h"
#include "../../../IOT_TestingCode/lora_driver.h"


#define temperature_and_humidity_bit (1 << 0)
#define co2_bit (1 << 1)


EventGroupHandle_t dataConfigurationGroup;
EventGroupHandle_t measureEventGroup;
MessageBufferHandle_t downlinkMessageBuffer;
MessageBufferHandle_t uplinkMessageBuffer;
SemaphoreHandle_t temperatureAndHumiditySemaphore;
SemaphoreHandle_t mutex;

void initializeDownlinkMessageBuffer();
void initializeEventGroup();
void initializeTemperatureAndHumiditySemaphore();
void initialize_mutex();