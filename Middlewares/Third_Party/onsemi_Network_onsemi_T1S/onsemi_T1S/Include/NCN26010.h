#ifndef NCN26010_H
#define NCN26010_H
/***************************************************************************//**
* @mainpage :    10Base-T1S MACPHY - NCN26010
***************************************************************************//**
* @file     NCN26010.h
* @brief    Support functions to simplify utilization of NCN26010 Features.
* @author   Arndt Schuebel, Kyle Storey.
* $Rev:    $
* $Date:    $
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

#include <stdbool.h>
#include <stdint.h>

/*************************************************************************************************
*  Configuration
*************************************************************************************************/
#define REMOTE_CONFIGURATION_ENABLED			1
#define PLCA_ENABLED							(bool) false
#define PLCA_ROLE								(bool) false //Not a leader unless ID 0
#define PLCA_ID_INIT							(uint8_t) 0 //LEADER
#define PLCA_NODE_COUNT							(uint8_t) 4 //4 nodes
#define ENI_ENABLED								(bool) false //Enhanced Noise Immunity
#define POLLING									0
#define USE_INTERNAL_MAC_ADDRESS				1 //If true read the MAC stored in the NCN26010 otherwise use DEFAULT_MAC_ADDRESS
#define MAC_PROMISCUOUS_MODE					0
#define DEFAULT_MAC_ADDRESS {0x60, 0xC0, 0xBF, 0x01, 0x01, 0x01}
#define DEFAULT_IP_ADDRESS_ARRAY {192, 1, 1, 1}
#define DEFAULT_IP_ADDRESS 0x010101c0	//matches 192.1.1.1

/*************************************************************************************************
*  Symbolic constants                                                                            *
*************************************************************************************************/

#define MAX_PAYLOAD_BYTE            (uint8_t) 64 // ToDo This is configurable so need to change based on configuration
#define EACH_REG_SIZE                (uint8_t) 4
#define MAX_REG_DATA_ONECHUNK        (uint8_t) (MAX_PAYLOAD_BYTE/EACH_REG_SIZE)
#define MAX_DATA_DWORD_ONECHUNK     MAX_REG_DATA_ONECHUNK
#define HEADER_FOOTER_SIZE			(uint8_t)4
#define STATUS_OK(status)			(uint8_t)(((status) & 0xFFFFFFF7) == 0x00000000)
#define SIZE_OF_MAC_ADDR			(uint8_t)6
#define ENABLE_MAC_FILTER			(uint32_t)1 << 31

/*************************************************************************************************
*  Global Configuration                                                                          *
*************************************************************************************************/

extern uint8_t g_currentNodeMACAddr[SIZE_OF_MAC_ADDR];

/**************************************************************************************************
 *       Structures
 *************************************************************************************************/

typedef struct
{
    uint8_t type;
    uint8_t MMS;
    uint16_t address;
    uint32_t data;
} stRemoteConfigurationCommand_t;

/*************************************************************************************************
 *   Error Codes
 ************************************************************************************************/

#define OK                             (uint32_t) 0
#define UNKNOWN_ERROR                  (uint32_t) 1
#define PHYSICAL_COLLISSION_ERROR      2
#define PLCA_RECOVERY_ERROR            3
#define REMOTE_JABBER_ERROR            4
#define LOCAL_JABBER_ERROR             5
#define RX_BUFFER_OVERFLOW_ERROR       6
#define TX_BUFFER_OVERFLOW_ERROR       7
#define TX_BUFFER_UNDERFLOW_ERROR      8
#define REGISTER_READ_ERROR            9
#define INIT_ERROR		       10
#define SIZE_OF_MAC_ADDR			(uint8_t)6
//TODO FILL IN ALL POSSIBLE ERRORS

/*************************************************************************************************
 *   Prototypes
 ************************************************************************************************/
/*!
    \brief Saves configuration options for Physical Layer Colision Avoidance. Must be called before \a NCN26010_Init.
    \param id The PLCA id, which determinses the order in the round robin. If 0 then this node is the PLCA leader.
	\param max_id The maximum PLCA id, ignored if id != 0, determines the number of nodes on the same segment.
	\param enabled If true the part will be configured in PLCA mode. Otherwise \a id and \a max_id are igonored.
	\return OK after internal varialbes have been set.
*/
uint32_t T1S_ConfigurePLCA(uint8_t id, uint8_t max_id, bool enabled);
/*!
    \brief Configures the NCN26010 as defined in \file NCN26010.h, Configures Interrupt Routines and initializes SPI, TCP/IP, and (if applicable) OS specific code.
	\return OK if successfully initalized of all module, INIT_ERROR otherwise.
*/
uint32_t NCN26010_Init(); // Configures as defined in this header. Returns 0 on success otherwise an error code
/*!
    \brief Endless loop to retrieve data from NCN26010 to the TCP/IP stack and vice versa. Calls \a OS_WaitUntilISR between each iteration to support event driven realtime operation.
	\return OK only after \a NCN26010_Exit is called.
*/
uint32_t NCN26010_Loop();
/*!
    \brief Assenbles \a txBuffer into chunks and transfers then to the NCN26010 to be trasmitted over T1S.
	\param txBuffer A pointer to where the data to be transfered is stored.
	\param num_bytes_to_transmit The number of bytes in txBuffer to transmit.
	\return OK if data transfered to NCN26010 successfully. An error code indicating what went wront otherwise.
*/
uint32_t T1S_Transmit(uint8_t* txBuffer, uint16_t num_bytes_to_transmit); //create a ethernet frame and transfer the given data. Return 0 on success otherwise an error code connected with the problem
/*!
    \brief Reads a register on the NCN26010 over SPI.
	\param MMS The memory map selector of the register to be read.
	\param address The address of the register to be read.
	\return The value of the requested register.
*/
uint32_t T1S_RegRead(uint8_t MMS, uint16_t address); //return the value in the given address
/*!
    \brief Writes a register on the NCN26010 over SPI.
	\param MMS The memory map selector of the register to be written.
	\param address The address of the register to be written.
	\return The value written as echoed by the NCN26010 over SPI
*/
uint32_t T1S_RegWrite(uint8_t MMS, uint16_t address, uint32_t data); //write the given value in the given address
/*!
    \brief Reads or writes registers of a remote NCN26010. May be disbled by defining REMOTE_CONFIGURATION_ENABLED to be false.
	\param commands An array of \a stRemoteConfigurationCommand_t to send to the remote device.
	\param address The address of the register to be written.
	\return The value written as echoed by the NCN26010 over SPI
*/
uint32_t T1S_SendRemoteConfiguration(uint8_t targetMAC[SIZE_OF_MAC_ADDR], stRemoteConfigurationCommand_t* commands, uint32_t numCommands);
/*!
    \brief Reads and clears the NCN26010 status register (MMS 0, 0x8).
	\return OK if the status is now STATUS_OK. Otherwise the value of the status register.
*/
uint32_t T1S_ClearStatus();
/*!
    \brief Sets a flag to cause \a NCN26010_Loop to exit after the next iteration.
	\return OK.
*/
uint32_t NCN26010_Exit();

#endif /* NCN26010_H */
