#ifndef T1S_TCPIP_H
#define T1S_TCPIP_H
/***************************************************************************//**
* @mainpage :	10Base-T1S MACPHY - NCN26010
***************************************************************************//**
* @file 	T1S_TCP-IP.h
* @brief 	Defines function prototypes for interfacing with the TCP/IP layers of the network stack.
* @author 	Arndt Schuebel, Kyle Storey.
* $Rev:	$
* $Date:	$
******************************************************************************
* @copyright (c) 2021 ON Semiconductor. All rights reserved.
* ON Semiconductor is supplying this software for use with ON Semiconductor
* ethernet products only.
*
* THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
* ON SEMICONDUCTOR SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
* INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
* @details
*/

#include <stdint.h>
#include "ncn26010.h"

/*************************************************************************************************
 *   Error Codes
 ************************************************************************************************/
#define OK	                             	(uint32_t) 0
#define PEND		                 	(uint32_t) 1
#define TCPIP_UNKNOWN_ERROR                 	(uint32_t) 2
//TODO fill in other error codes

/*************************************************************************************************
 *   Prototypes
 ************************************************************************************************/
/*!
    \brief Initialization Callback for TCP/IP Code.
	\return OK if successfully initalized, TCPIP_UNKNOWN_ERROR otherwise.
*/
uint32_t TCPIP_Init();
/*!
    \brief Transfers data in a buffer into the TCP/IP Stack
    \param buffer A pointer to where the data to be received is stored
    \param number_of_bytes_received The number of bytes in the buffer to transfer to the TCP/IP stack
	\return OK if successfully transfered into the TCP/IP stack, TCPIP_UNKNOWN_ERROR otherwise.
*/
uint32_t TCPIP_Receive(uint8_t* buffer, uint16_t number_of_bytes_received);
/*!
    \brief Callback called frequently to allow for the TCPIP code to poll the TCP/IP stack to see if data is available.
    \param transmitCreditsAvailable_TXC The number of available chunks that will fit in NCN26010 buffer.
	\return OK if no erros occurred, TCPIP_UNKNOWN_ERROR otherwise.
*/
uint32_t TCPIP_PollCallback(uint8_t transmitCreditsAvailable_TXC);
/*!
    \brief Configures the NCN26010 to accept the given MAC address. Must be called before \a TCPIP_Init.
    \param address The MAC address
	\return OK if no erros occurred, TCPIP_UNKNOWN_ERROR otherwise.
*/
uint32_t setMACAddress(uint32_t address); //calls RegWrite in NCN26010
/*!
    \brief Configures the TCP/IP stack to claim the given IP address. Must be called before \a TCPIP_Init.
    \param address The IP address
	\return OK if no erros occurred, TCPIP_UNKNOWN_ERROR otherwise.
*/
uint32_t setIPAddress(uint32_t address); //

#endif /* T1S_TCPIP_H */
