/*
 * T1S_FreeRTOS_TCP-IP.c
 *
 *  Created on: May 2, 2022
 *      Author: FG4XXP
 */

#include "T1S_TCP-IP.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "ncn26010.h"
//#include <printf.h>
#define PRINTF(...) (0)

uint8_t g_currentNodeMACAddr[SIZE_OF_MAC_ADDR] = DEFAULT_MAC_ADDRESS;

uint32_t TCPIP_Init() {
	return OK;
}

uint32_t TCPIP_Receive(uint8_t* buffer, uint16_t number_of_bytes_received) {

	IPStackEvent_t xRxEvent;
	size_t xDataBytesReceived;
	NetworkBufferDescriptor_t *pxBufferDescriptor = NULL;
	pxBufferDescriptor = pxGetNetworkBufferWithDescriptor(ipTOTAL_ETHERNET_FRAME_SIZE,0);

	pxBufferDescriptor->pucEthernetBuffer = buffer;

	xDataBytesReceived = number_of_bytes_received;

	if (xDataBytesReceived > 0)
	{
		pxBufferDescriptor->xDataLength = xDataBytesReceived;
		PRINTF("\nNetwork Buffer: \n");
		for (int i = 0; i<xDataBytesReceived; i++)
		{
			PRINTF("%02x", pxBufferDescriptor->pucEthernetBuffer[i]);
			if(i%64 == 0 && i > 1)
				PRINTF("\n");
		}

		if(eConsiderFrameForProcessing(pxBufferDescriptor->pucEthernetBuffer) == eProcessBuffer)
		{
			xRxEvent.eEventType = eNetworkRxEvent;
			xRxEvent.pvData = (void*) pxBufferDescriptor;
			/* Send the data to the TCP/IP stack. */
			PRINTF("\nData sent to IP Stack\n");
			if (xSendEventStructToIPTask(&xRxEvent, 0) == pdFALSE) {
				vReleaseNetworkBufferAndDescriptor(pxBufferDescriptor);
			}
		}
		else
		{
			vReleaseNetworkBufferAndDescriptor(pxBufferDescriptor );
		}
	}

	return OK;
}

uint32_t TCPIP_PollCallback(uint8_t transmitCreditsAvailable_TXC){
	return OK;
}
