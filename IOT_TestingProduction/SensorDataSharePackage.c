#include "SensorDataSharePackage.h"

uint16_t temperature_value;
uint16_t humidity_value;
uint16_t co2_value;
uint8_t dataSharePackage_lenght;
uint8_t servo_actuator;
lora_driver_payload_t uplink_payload;


void SensorDataSharePackage_setTemperature_value(uint16_t value) {
	temperature_value = value;
}

void SensorDataSharePackage_setHumidity_value(uint16_t value) {
	humidity_value = value;
}

void SensorDataSharePackage_setCo2_value(uint16_t value) {
	co2_value = value;
}

void SensorDataSharePackage_setServo(uint8_t servo_state) {
	servo_actuator = servo_state;
}

lora_driver_payload_t SensorDataSharePackage_getLoraPayload(uint8_t port_No) {


	printf("Temperature ---> %i", temperature_value);
	printf("Humidity ---> %i", humidity_value);
	printf("CO2 ---> %i", co2_value);

	uplink_payload.portNo = port_No;
	uplink_payload.len = 6;

	uplink_payload.bytes[0] = (uint8_t)(humidity_value >> 8);
	uplink_payload.bytes[1] = (uint8_t)(humidity_value & 0xFF);

	uplink_payload.bytes[2] = (uint8_t)(temperature_value >> 8);
	uplink_payload.bytes[3] = (uint8_t)(temperature_value & 0xFF);

	uplink_payload.bytes[4] = (uint8_t)(co2_value >> 8);
	uplink_payload.bytes[5] = (uint8_t)(co2_value & 0xFF);


	return uplink_payload;
}