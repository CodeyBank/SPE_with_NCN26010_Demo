/*
 * FreeRTOS+TCP V2.3.2
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

/* RSL10 includes. */
#include <NCN26010.h>
//#include <printf.h>
#define PRINTF(...) (0)

/* If ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES is set to 1, then the Ethernet
 * driver will filter incoming packets and only pass the stack those packets it
 * considers need processing. */
#if ( ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES == 0 )
#define ipCONSIDER_FRAME_FOR_PROCESSING( pucEthernetBuffer )    eProcessBuffer
#else
#define ipCONSIDER_FRAME_FOR_PROCESSING( pucEthernetBuffer )    eConsiderFrameForProcessing( ( pucEthernetBuffer ) )
#endif

#define STACK_SIZE 2048
StaticTask_t xTaskBuffer;
StackType_t xStack[ STACK_SIZE ]__attribute__ ((aligned(4)));

void vTaskNCN26010Poll( void * pvParameters )
{
	/* Check if new data has been received on the line Polling */
	configASSERT( ( uint32_t ) pvParameters == 1UL );

	NCN26010_Loop();
}

BaseType_t xNetworkInterfaceInitialise(void) {

	TaskHandle_t xHandle = NULL;

	// create UDP server task
	vTaskCreateUDPServer(STACK_SIZE, configMAX_PRIORITIES);

	/* Create the task without using any dynamic memory allocation. */
	xHandle = xTaskCreateStatic(
			vTaskNCN26010Poll,       /* Function that implements the task. */
			"NCN26010_Poll",          /* Text name for the task. */
			STACK_SIZE,      /* Number of indexes in the xStack array. */
			( void * ) 1,    /* Parameter passed into the task. */
			( configMAX_PRIORITIES - 3 ),/* Priority at which the task is created. */
			xStack,          /* Array to use as the task's stack. */
			&xTaskBuffer );  /* Variable to hold the task's data structure. */

	if(xHandle != NULL) return pdTRUE;
	else return pdFALSE;
}

BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxNetworkBuffer,
		BaseType_t xReleaseAfterSend )
{
	T1S_Transmit(pxNetworkBuffer->pucEthernetBuffer, pxNetworkBuffer->xDataLength);
	PRINTF("\nDataChunk sent\n");
	PRINTF("Data Length = %d\n", pxNetworkBuffer->xDataLength);
	PRINTF("CONTENT = \n");
	for (int i = 0; i<pxNetworkBuffer->xDataLength; i++)
	{
		PRINTF("%02x",pxNetworkBuffer->pucEthernetBuffer[i] );
	}

	if (xReleaseAfterSend != pdFALSE)
	{
		vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
	}
	memset(pxNetworkBuffer->pucEthernetBuffer, 0,  pxNetworkBuffer->xDataLength);
	return pdTRUE;
}

#define BUFFER_SIZE 				(ipTOTAL_ETHERNET_FRAME_SIZE+ipBUFFER_PADDING)
#define BUFFER_SIZE_ROUNDED_UP		((BUFFER_SIZE+7)) & ~0x07UL
static uint8_t ucBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS] [BUFFER_SIZE_ROUNDED_UP];

void vNetworkInterfaceAllocateRAMToBuffers(NetworkBufferDescriptor_t pxNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS])
{
	for(int i = 0; i<ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS; i++)
	{
		pxNetworkBuffers[i].pucEthernetBuffer = &(ucBuffers[i][ipBUFFER_PADDING]);
		*((uint32_t*) &ucBuffers[i][0]) = (uint32_t) &(pxNetworkBuffers[i]);
	}
}

BaseType_t xGetPhyLinkStatus( void )
{
	/* FIX ME. */
	return pdFALSE;
}
