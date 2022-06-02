/*
* main.c
* Author : IHA
*
* Example main file including LoRaWAN setup
* Just for inspiration :)
*/

#include <stdio.h>
//#include <avr/io.h>
#include <../../../IOT_TestingCode/FreeRTOS.h>
#include <../../../IOT_TestingCode/task.h>
#include <../../../IOT_TestingCode/semphr.h>
//#include <stdio_driver.h>
#include <../../../IOT_TestingCode/serial.h>
#include <../../../IOT_TestingCode/lora_driver.h>
//#include <status_leds.h>
#include <../../../IOT_TestingCode/event_groups.h>
//#include "tempHumid.h"
#include "co2.h"
//#include "servo.h"
//#include "LoRaWANHandlerUplink.h"
//#include "LoRaWANDownlinkHandler.h"
#include "Application.h"
#include "Initializers.h"
//#include "Organization.h"


//define sensor data
//uint16_t temperature = 0;
//uint16_t humidity = 0;
//uint16_t ppm = 0;

//define servo threshold

//uint16_t minHumidity = 100;

extern MessageBufferHandle_t downlinkMessageBuffer;

// define semaphore handle
//SemaphoreHandle_t xTestSemaphore;

// Prototype for LoRaWAN handler
//void lora_handler_initialise(UBaseType_t lora_handler_task_priority);

void taskInitializeData()
{
	initializeTemperatureAndHumiditySemaphore();
	initializeEventGroup();
	initializeDownlinkMessageBuffer();
	initialize_mutex();
	lora_driver_initialise(ser_USART1, downlinkMessageBuffer);
}

/*-----------------------------------------------------------*/
void create_tasks(void)
{
	createTempHumidTask(1);
	createCo2Task();
	applicationTaskRun(3);
	lora_uplink_handler_create(4);
	lora_downlink_handler_create(5);
	servo_TaskRun(2);
}


/*-----------------------------------------------------------*/
void initialiseSystem()
{
	stdio_initialise(ser_USART0);
	taskInitializeData();
	create_tasks();

}

/*-----------------------------------------------------------*/
int main(void)
{
	initialiseSystem(); // Must be done as the very first thing!!
	printf("Program Started!!\n");
	vTaskStartScheduler(); // Initialize and run the freeRTOS scheduler. Execution should never return from here.

	/* Replace with your application code */
	while (1)
	{

	}
}

