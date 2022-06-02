
#ifndef LORA_DRIVER_H_
#define LORA_DRIVER_H_
#include <stdbool.h>
#include <../../../IOT_TestingCode/FreeRTOS.h>
#include <../../../IOT_TestingCode/message_buffer.h>

//#include <serial.h>

#define LORA_MAX_PAYLOAD_LENGTH	20 /* bytes - Must newer be changed!!!*/

typedef struct lora_driver_payload {
	uint8_t portNo; /**< Port_no the data is received on, or to transmit to [1..223]*/
	uint8_t len; /**< Length of the payload (no of bytes) - MAX 20 bytes is allowed in this implementation! */
	uint8_t bytes[LORA_MAX_PAYLOAD_LENGTH]; /**< Array to hold the payload to be send, or that has been received */
} lora_driver_payload_t;

typedef enum Lora_driver_returnCodes {
	LORA_OK	/**< Everything went well */
	, LORA_ERROR /**< An error occurred - the reason is not explained any further  */
	, LORA_KEYS_NOT_INIT /**< The necessary keys are not initialized */
	, LORA_NO_FREE_CH	/**< All channels are buzy */
	, LORA_SILENT /**< The module is in a Silent Immediately state */
	, LORA_BUSY /**< The MAC state of the module is not in an idle state */
	, LORA_MAC_PAUSED /**< The MAC is in PAUSED state and needs to be resumed back*/
	, LORA_DENIED /**< The join procedure was unsuccessful (the module attempted to join the
	network, but was rejected) */
	, LORA_ACCEPTED  /**< The join procedure was successful */
	, LORA_INVALID_PARAM  /**< One of the parameters given is wrong */
	, LORA_NOT_JOINED /**< The network is not joined */
	, LORA_INVALID_DATA_LEN /**< If application payload length is greater than the maximum
	application payload length corresponding to the current data rate */
	, LORA_FRAME_COUNTER_ERR_REJOIN_NEEDED /**< If the frame counter rolled over - a rejoin is needed */
	, LORA_MAC_TX_OK /**< If up link transmission was successful and no down link data was
	received back from the server */
	, LORA_MAC_RX /**< If there is a downlink message is received on an uplink transmission */
	, LORA_MAC_ERROR /**< If transmission was unsuccessful, ACK not received back from the
	server */
	, LORA_UNKNOWN /**< An unknown error occurred that is not identified by this driver */
} lora_driver_returnCode_t;

typedef enum lora_driver_joinModes {
	LORA_OTAA = 0  /**< Join the LoRaWAN network with Over The Air Activation (OTAA) */
	, LORA_ABP /**< Join the LoRaWAN network Activation By Personalization (ABP) */
} lora_driver_joinMode_t;

typedef enum lora_driver_adaptiveDataRateModes {
	LORA_OFF = 0 /**< Set ADR to ON */
	, LORA_ON /**< Set ADR to OFF */
} lora_driver_adaptiveDataRate_t;

/**
\ingroup lora_config
\brief Automatic Reply (AR) modes.
*/
typedef enum lora_driver_automaticReplyModes {
	LORA_AR_ON /**< Set AR to ON */
	, LORA_AR_OFF  /**< Set AR to OFF */
} lora_driver_automaticReplyMode_t;

/* ======================================================================================================================= */
/**
\ingroup lora_basic_function
\brief Get max payload size in bytes.

The maximum allowed number of bytes that must be sent in the payload!
*/
uint8_t lora_driver_getMaxPayloadSize(void);

//svoid lora_driver_initialise(serial_comPort_t comPort, MessageBufferHandle_t downlinkMessageBuffer);

lora_driver_returnCode_t lora_driver_setOtaaIdentity(char appEUI[17], char appKEY[33], char devEUI[17]);

lora_driver_returnCode_t lora_driver_configureToEu868(void);

char* lora_driver_mapReturnCodeToText(lora_driver_returnCode_t returnCode);

lora_driver_returnCode_t lora_driver_setAbpIdentity(char nwkSKEY[33], char appSKEY[33], char devADD[9]);

lora_driver_returnCode_t lora_driver_sendUploadMessage(bool confirmed, lora_driver_payload_t* payload);

lora_driver_returnCode_t lora_driver_setDeviceIdentifier(const char devEUI[17]);

lora_driver_returnCode_t lora_driver_setApplicationIdentifier(const char appEUI[17]);

lora_driver_returnCode_t lora_driver_setApplicationKey(const char appKey[33]);

lora_driver_returnCode_t lora_driver_setNetworkSessionKey(const char nwkSKey[33]);

lora_driver_returnCode_t lora_driver_setApplicationSessionKey(const char appSKey[33]);

lora_driver_returnCode_t lora_driver_setDeviceAddress(const char devAddr[9]);

lora_driver_returnCode_t lora_driver_setDataRate(uint8_t dr);

lora_driver_returnCode_t lora_driver_getDataRate(uint8_t* dr);

lora_driver_returnCode_t lora_driver_setAdaptiveDataRate(lora_driver_adaptiveDataRate_t state);

lora_driver_returnCode_t lora_driver_getAdaptiveDataRate(lora_driver_adaptiveDataRate_t* state);

lora_driver_returnCode_t lora_driver_setReceiveDelay(uint16_t rxDelay1);

lora_driver_returnCode_t lora_driver_setAutomaticReply(lora_driver_automaticReplyMode_t ar);

lora_driver_returnCode_t lora_driver_getAutomaticReply(lora_driver_automaticReplyMode_t* ar);

lora_driver_returnCode_t lora_driver_setLinkCheckInterval(uint16_t sec); // [0..65535]

lora_driver_returnCode_t lora_driver_getLinkCheckResult(uint8_t* no_gwys, uint8_t* margin);

lora_driver_returnCode_t lora_driver_setSpreadingFactor(uint8_t sf);

void lora_driver_resetRn2483(uint8_t state);

/* ======================================================================================================================= */
/**
\ingroup lora_advanced_function
\brief Flush the internal buffers in the driver.

*/
void lora_driver_flushBuffers(void);

/* ======================================================================================================================= */
/**
\ingroup lora_basic_function
\brief Get the RN2483 factory set devEUI.

This device ID is unique in time and space.

This hardware device ID is not automatically being used as the devEUI seen from the LoRaWAN. The later must be set using \ref lora_driver_setOtaaIdentity or \ref lora_driver_setDeviceIdentifier.

\param[out] hwDevEUI buffer where the hardware device ID will be returned.
\return lora_driver_returnCode
*/
lora_driver_returnCode_t lora_driver_getRn2483Hweui(char hwDevEUI[17]);

/* ======================================================================================================================= */
/**
\ingroup lora_basic_function
\brief Get the RN2483 modules supply voltage VDD.

\todo Implement lora_driver_rn2483GetVdd function!

\param[out] mv buffer where the VDD voltage will be returned [mv]
\return lora_driver_returnCode
*/
lora_driver_returnCode_t lora_driver_rn2483GetVdd(char mv[5]);

/* ======================================================================================================================= */
/**
\ingroup lora_basic_function
\brief Reset the RN2483 module.

Reboots the module and automatically restores the last saved parameters set in the module.
For a list of restored parameters see <a href="https://ww1.microchip.com/downloads/en/DeviceDoc/40001784B.pdf">RN2483 LoRa Technology Module
Command Reference User's Guide</a>

\return lora_driver_returnCode
*/
lora_driver_returnCode_t lora_driver_rn2483Reboot(void);

/* ======================================================================================================================= */
/**
\ingroup lora_basic_function
\brief Reset the RN2483 module.

Reboots the module and restores all parameters to factory settings.
\note I can't find a list of these default values.

\return lora_driver_returnCode
*/
lora_driver_returnCode_t lora_driver_rn2483FactoryReset(void);

/* ======================================================================================================================= */
/**
\ingroup lora_advanced_function
\brief Set the RN2384 module in sleep mode for a given periode.

This command puts the system to Sleep for the specified number of milliseconds. The
module can be forced to exit from Sleep by sending a break condition followed by a
0x55 character at the new baud rate. Note that the break condition needs to be long
enough not to be interpreted as a valid character at the current baud rate.

\todo Implement lora_driver_sleep function.

\note If the module is in sleep mode it will save battery power.

\param[in] ms The number of milliseconds to sleep [100-4294967296].
*/
lora_driver_returnCode_t lora_driver_sleep(uint32_t ms);

/* ======================================================================================================================= */
/**
\ingroup lora_basic_function
\brief Save the set parameters into the EEPROM of the RN2483 module.

For a list of restored parameters see <a href="https://ww1.microchip.com/downloads/en/DeviceDoc/40001784B.pdf">RN2483 LoRa Technology Module
Command Reference User's Guide</a>

\return lora_driver_returnCode
*/
lora_driver_returnCode_t lora_driver_saveMac(void);

/* ======================================================================================================================= */
/**
\ingroup lora_advanced_function
\brief Pause the MAC layer in the RN2483 module.

This must be done before any commands are send to the radio layer.

\return lora_driver_returnCode
*/
lora_driver_returnCode_t lora_driver_pauseMac(void);

/* ======================================================================================================================= */
/**
\ingroup lora_advanced_function
\brief Resume the MAC layer in the RN2483 module.

This must be done after a pause is finished.

\return lora_driver_returnCode
*/
lora_driver_returnCode_t lora_driver_resumeMac(void);

/**
\page lora_driver_quickstart Quick start guide for RN2483 based LoRa Driver

This is the quick start guide for the \ref lora_driver, with
step-by-step instructions on how to configure and use the driver in a
selection of use cases.

The use cases contain several code fragments. The code fragments in the
steps for setup can be copied into a custom initialization function, while
the steps for usage can be copied into, e.g., the main application function.

\section lora_driver_use_cases LoRa Driver use cases
- \ref lora_setup_use_case
- \subpage lora_setup_to_OTAA
- \subpage lora_setup_to_ABP
- \subpage lora_send_uplink_message
- \subpage lora_receive_downlink_message

\section lora_setup_use_case Initialise the driver
The following must be added to the project:
- \code
#include <lora_driver.h>
\endcode

Add to application initialization:
- Initialise the driver without down-link possibility:
\code
lora_driver_initialise(ser_USART1, NULL); // The parameter is the USART port the RN2483 module is connected to - in this case USART1 - here no message buffer for down-link messages are defined
\endcode
- Alternatively initialise the driver with down-link possibility:
\code
MessageBufferHandle_t downLinkMessageBufferHandle = xMessageBufferCreate(sizeof(lora_driver_payload_t)*2); // Here I make room for two downlink messages in the message buffer
lora_driver_initialise(ser_USART1, downLinkMessageBufferHandle); // The parameter is the USART port the RN2483 module is connected to - in this case USART1 - here no message buffer for down-link messages are defined
\endcode

Then the LoRaWAN transceiver needs to be hardware reset.
\note This must be done from a FreeRTOS task!!
\code
lora_driver_resetRn2483(1); // Activate reset line
vTaskDelay(2);
lora_driver_resetRn2483(0); // Release reset line
vTaskDelay(150); // Wait for tranceiver module to wake up after reset
lora_driver_flushBuffers(); // get rid of first version string from module after reset!
\endcode

Now you are ready to use the LoRaWAN module :)
*/

/**
\page lora_setup_to_OTAA OTAA setup steps
\note All the following code must be implemented in the initialisation part of a FreeRTOS task!
\note Nearly all calls to the driver will suspend the calling task while the driver waits for response from the RN2484 module.


\section lora_basic_use_case_setupOTAA_code Example code
In this use case, the driver is setup to Over The Air Activation (OTAA).

\section lora_setup_use_case_OTAA_setup_flow Workflow
-# Define the necessary app identification for OTAA join:
\code
// Parameters for OTAA join
#define LORA_appEUI "????????????????"
#define LORA_appKEY "????????????????????????????????"
\endcode

\note The parameters depends on the setup of the LoRaWAN network server and will be given to you.

-# Set the module to factory set defaults:
\code
if (lora_driver_rn2483FactoryReset() != LORA_OK)
{
	// Something went wrong
}
\endcode

-# Configure the module to use the EU868 frequency plan and settings:
\code
if (lora_driver_configureToEu868() != LORA_OK)
{
	// Something went wrong
}
\endcode

-# Get the RN2483 modules unique devEUI:
\code
static char devEui[17]; // It is static to avoid it to occupy stack space in the task
if (lora_driver_getRn2483Hweui(devEui) != LORA_OK)
{
	// Something went wrong
}
\endcode

-# Set the necessary LoRaWAN parameters for an OTAA join:
\code
if (lora_driver_setOtaaIdentity(LORA_appEUI,LORA_appKEY,devEui) != LORA_OK)
{
	// Something went wrong
}
\endcode

-# Save all set parameters to the RN2483 modules EEPROM (OPTIONAL STEP):
\note If this step is performed then it is no necessary to do the steps above more than once. These parameters will automatically be restored in the module on next reset or power on.

\code
if (lora_driver_saveMac() != LORA_OK)
{
	// Something went wrong
}

// All parameters are now saved in the module
\endcode

-# Join LoRaWAN parameters with OTAA:
\code
if (lora_driver_join(LORA_OTAA) == LORA_ACCEPTED)
{
	// You are now joined
}
 \endcode
*/

/**
 \page lora_setup_to_ABP ABP setup steps
 \note All the following code must be implemented in the initialisation part of a FreeRTOS task!
 \note Nearly all calls to the driver will suspend the calling task while the driver waits for response from the RN2484 module.

 \section lora_basic_use_case_setup_ABP_code Example code
 In this use case, the driver is setup to Activation by personalization (ABP).

 \section lora_setup_use_case_ABP_setup_flow Workflow
 -# Define the necessary app identification for ABP join:
 \code
// Parameters for ABP join
#define LORA_appAddr "????????"
#define LORA_nwkskey "????????????????????????????????"
#define LORA_appskey "????????????????????????????????"
 \endcode

 \note The parameters depends on the setup of the LoRaWAN network server and will be given to you.

 -# Set the module to factory set defaults:
 \code
 if (lora_driver_rn2483FactoryReset() != LORA_OK)
 {
	 // Something went wrong
 }
 \endcode

 -# Configure the module to use the EU868 frequency plan and settings:
 \code
 if (lora_driver_configureToEu868() != LORA_OK)
 {
	 // Something went wrong
 }
 \endcode

 -# Set the necessary LoRaWAN parameters for an ABP join:
 \code
 if (lora_driver_setAbpIdentity(LORA_nwkskey,LORA_appskey,LORA_appAddr) != LORA_OK)
 {
	 // Something went wrong
 }
 \endcode

 -# Save all set parameters to the RN2483 modules EEPROM (OPTIONAL STEP):
 \note If this step is performed then it is no necessary to do the steps above more than once. These parameters will automatically be restored in the module on next reset or power on.

 \code
 if (lora_driver_saveMac() != LORA_OK)
 {
	 // Something went wrong
 }

 // All parameters are now saved in the module
 \endcode

 -# Join LoRaWAN parameters with ABP:
 \code
 if (lora_driver_join(LORA_ABP) == LORA_ACCEPTED)
 {
	 // You are now joined
 }
 \endcode
*/

/**
\page lora_send_uplink_message Sent an uplink message

In this use case, an uplink message will be send.

\note The driver must be initialised \ref lora_setup_use_case and must be setup to OTAA \ref lora_setup_to_OTAA or ABP \ref lora_setup_to_OTAA.

In this example these two variables will be send in an uplink message
\par
\code
	uint16_t hum; // Humidity
	int16_t temp; // Temperature
\endcode

\section lora_send_uplink_message_setup Uplink Message Setup
The following must be added to a FreeRTOS task in the project:
-# Define a payload struct variable
\code
	lora_driver_payload_t uplinkPayload;
\endcode

-# Populate the payload struct with data
\code
	uplinkPayload.len = 4; // Length of the actual payload
	uplinkPayload.port_no = 1; // The LoRaWAN port no to sent the message to

	uplinkPayload.bytes[0] = hum >> 8;
	uplinkPayload.bytes[1] = hum & 0xFF;
	uplinkPayload.bytes[2] = temp >> 8;
	uplinkPayload.bytes[3] = temp & 0xFF;
 \endcode
-# Send the uplink message:
\code
	lora_driver_returnCode_t rc;

	if ((rc = lora_driver_sendUploadMessage(false, &_uplinkPayload)) == LORA_MAC_TX_OK )
	{
		// The uplink message is sent and there is no downlink message received
	}
	else if (rc == LORA_MAC_RX_OK)
	{
		// The uplink message is sent and a downlink message is received
	}
 \endcode
 */

 /**
\page lora_receive_downlink_message Receive a downlink message

In this use case, a downlink link message will be received.
A downlink message is received in a lora_driver_payload_t variable.

\note The driver must be initialised \ref lora_setup_use_case and must be setup to OTAA \ref lora_setup_to_OTAA or ABP \ref lora_setup_to_OTAA.
\note To be able to receive any downlink messages you may specify a FreeRTOS message buffer during the initialisation of the driver. In this message buffer the received messages will be delivered by the driver (\ref lora_setup_use_case).

In this example a downlink message with 4 bytes will be received from the LoRaWAN.
These 4 bytes will in this example represent a maximum humidity setting and a maximum temperature setting that we want to be able to recieve from the LoRaWAN.
\par
\code
	uint16_t maxHumSetting; // Max Humidity
	int16_t maxTempSetting; // Max Temperature
\endcode

\section lora_receive_downlink_message_setup Down-link Message Setup

The following must be added to a FreeRTOS tasks for(;;) loop in your application - typical you will have a separate task for handling downlink messages:
-# Define a payload struct variable
Create a payload variable to receive the down-link message in
\code
	lora_driver_payload_t downlinkPayload;
\endcode

-# Wait for a message to be received
\code
	// this code must be in the loop of a FreeRTOS task!
	xMessageBufferReceive(downLinkMessageBufferHandle, &downlinkPayload, sizeof(lora_driver_payload_t), portMAX_DELAY);
	printf("DOWN LINK: from port: %d with %d bytes received!", downlinkPayload.port_no, downlinkPayload.len); // Just for Debug
	if (4 == downlinkPayload.len) // Check that we have got the expected 4 bytes
	{
		// decode the payload into our variales
		maxHumSetting = (downlinkPayload.bytes[0] << 8) + downlinkPayload.bytes[1];
		maxTempSetting = (downlinkPayload.bytes[2] << 8) + downlinkPayload.bytes[3];
	}
 \endcode
*/
#endif /* LORA_DRIVER_H_ */