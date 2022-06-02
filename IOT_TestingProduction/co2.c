#include "co2.h"


mh_z19_returnCode_t rc;


uint16_t co2_data;

void Co2_init() {
/*
	mh_z19_initialise(ser_USART3);
	*/
}

void Co2_measureTask(void) {

	//rc = mh_z19_takeMeassuring();

	if (rc != MHZ19_OK)
	{
		printf("das ist kaput, scheisse");
	}
	printf("CO2 Task set  ");
}

/*void createCo2Task() {
	Co2_init();
	BaseType_t xTaskCreate(
		TaskFunction_t Co2_getDataFromSensorTask
		, "Get data from Sensor task "
		, configMINIMAL_STACK_SIZE
		, NULL
		, 1
		, NULL
	);
}*/

void Co2_getDataFromSensorTask() {

	while (1) {
		EventBits_t sensorDataBits = xEventGroupWaitBits(measureEventGroup, co2_bit, pdTRUE, pdTRUE, portMAX_DELAY);
		if ((sensorDataBits & co2_bit) == co2_bit)
			Co2_measureTask();
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}


/*void myCo2CallBack(uint16_t ppm) {

	co2_data = ppm;
	printf("CO2:  %i", co2_data);
	xEventGroupSetBits(dataConfigurationGroup, co2_bit);

}
*/

