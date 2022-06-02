#include "Initializers.h"


void initializeTemperatureAndHumiditySemaphore() {
	temperatureAndHumiditySemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(temperatureAndHumiditySemaphore);
}

void initialize_mutex() {
	mutex = xSemaphoreCreateMutex();
}

void initializeEventGroup() {
	dataConfigurationGroup = xEventGroupCreate();
	measureEventGroup = xEventGroupCreate();
}

void initializeDownlinkMessageBuffer() {

	uplinkMessageBuffer = xMessageBufferCreate(sizeof(lora_driver_payload_t) * 2);
	downlinkMessageBuffer = xMessageBufferCreate(sizeof(lora_driver_payload_t) * 2);
}