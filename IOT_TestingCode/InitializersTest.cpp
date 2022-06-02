#include "gtest/gtest.h"

#include "FreeRTOS_MOCK_FFF.h"
#include <stdint.h>


extern "C"
{
#include <../../../IOT_TestingProduction/Initializers.h>
#include "../../../IOT_TestingCode/FreeRTOS.h"
#include <../../../IOT_TestingCode/task.h>
#include "../../../IOT_TestingCode/semphr.h"
#include <../../../IOT_TestingCode/event_groups.h>
#include <../../../IOT_TestingCode/queue.h>
}



class InitializersTesting : public::testing::Test
{
protected:
	void SetUp() override
	{
		RESET_FAKE(xMessageBufferCreate);
		RESET_FAKE(xSemaphoreCreateBinary);
		RESET_FAKE(xSemaphoreGive);
		RESET_FAKE(xQueueCreate);
		RESET_FAKE(xEventGroupCreate);
		FFF_RESET_HISTORY();
	}

	void TearDown() override
	{}
};

TEST_F(InitializersTesting, Initialize_DownlinkMessageBuffer) {
	initializeDownlinkMessageBuffer();

	
	ASSERT_EQ(44, xMessageBufferCreate_fake.arg0_val);
}

TEST_F(InitializersTesting, Initialise_Semaphore) {
	initializeTemperatureAndHumiditySemaphore();

	ASSERT_EQ(1, xSemaphoreCreateBinary_fake.call_count);
	ASSERT_EQ(1, xSemaphoreGive_fake.call_count);

}
TEST_F(InitializersTesting, Initialise_mutex) {
	initialize_mutex();

	ASSERT_EQ(1, xSemaphoreCreateMutex_fake.call_count);
	
}