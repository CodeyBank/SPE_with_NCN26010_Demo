/*
 * T1S_FreeRTOS_OS.c
 *
 * Created on: May 6, 2022
 * Author: zbmgzd
 */

#include "T1S_OS.h"
#include "FreeRTOS_IP.h"
#include "event_groups.h"
#include "ncn26010.h"

/* Declare a variable to hold the handle of the created event group. */
EventGroupHandle_t xEventGroupHandle;

/* Declare a variable to hold the data associated with the created
event group. */
StaticEventGroup_t xCreatedEventGroup;

uint32_t OS_Init() {

	SystemCoreClockUpdate();

	static const uint8_t ucIPAddress[ 4 ] = DEFAULT_IP_ADDRESS_ARRAY;
	static const uint8_t ucNetMask[ 4 ] = { 255, 255, 255, 0 };
	static const uint8_t ucGatewayAddress[ 4 ] = { 192, 1, 1, 1 };
	static const uint8_t ucDNSServerAddress[ 4 ] = { 208, 67, 222, 222 };

	/* Initialise the RTOS's TCP/IP stack. */
	FreeRTOS_IPInit( ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, g_currentNodeMACAddr );

	/* Attempt to create the event group. */
	xEventGroupHandle = xEventGroupCreateStatic( &xCreatedEventGroup );

	return OK;
}

void OS_Start(){
	/* Start the RTOS scheduler. */
	vTaskStartScheduler();
}

uint32_t OS_WaitUntilISR() {
	EventBits_t uxBits;

	/* Wait a maximum of 100ms for either bit 0 or bit 4 to be set within
	  the event group.  Clear the bits before exiting. */
	uxBits = xEventGroupWaitBits(
			xEventGroupHandle,   /* The event group being tested. */
			0x1, /* The bits within the event group to wait for. */
			pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
			pdFALSE,       /* Don't wait for both bits, either bit will do. */
			-1);/* Wait a maximum of 100ms for either bit to be set. */

	if(uxBits != 0x01) {
		return UNKNOWN_ERROR;
	}
	else
		return OK;
}

static ISR_t dataReadyCallback = NULL;
void OS_Callback(uint32_t tick) {

	BaseType_t xHigherPriorityTaskWoken, xResult;

	/* Set bit 0 and bit 4 in xEventGroup. */
	xResult = xEventGroupSetBitsFromISR(
			xEventGroupHandle,   /* The event group being updated. */
			0x1, /* The bits being set. */
			&xHigherPriorityTaskWoken );

	if (dataReadyCallback != NULL) {
		dataReadyCallback(tick);
	}
}

uint32_t OS_SetDataReadyCallback(ISR_t callback) {
	dataReadyCallback = callback;
	SetDataReadyISR(OS_Callback);
	return OK;
}
