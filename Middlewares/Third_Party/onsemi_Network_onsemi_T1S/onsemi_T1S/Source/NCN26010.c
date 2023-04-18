/***************************************************************************//**
* @mainpage :    10Base-T1S MACPHY - NCN26010
***************************************************************************//**
* @file     : NCN26010.c
* @brief     : Support functions to simplify utilization of NCN26010 Features
* @author     : Manjunath H M, Arndt Schuebel, Kyle Storey
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
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "NCN26010.h"
#include "T1S_Hardware.h"
#include "T1S_TCP-IP.h"
#include "T1S_OS.h"

/*************************************************************************************************
*  Symbolic constants                                                                            *
*************************************************************************************************/
#define MAX_CONTROL_CMD_LEN			(uint8_t)(0x7F)
#define HEADER_FOOTER_SIZE			(uint8_t)4
#define MAX_REG_DATA_ONECONTROLCMD	(uint8_t)(MAX_CONTROL_CMD_LEN * EACH_REG_SIZE)
#define MESSAGETYPE_SEND			(uint8_t)0
#define MESSAGETYPE_RESPONSE		(uint8_t)1
#define CONFIGTYPE_REGREAD			(uint8_t)0
#define CONFIGTYPE_REGWRITE			(uint8_t)1
#define ETHERNET_HEADER_SIZE		(uint8_t)14
#define ETHERNET_TYPE_MACPHYCONFIG	(uint16_t)0x0909
#define MAX_RCA_VALUE				(uint8_t)31 // max 5 bit value as RCA is 5bit value
#define CONFIG_FILE_MAXLINE_LEN		(uint8_t)50
#define FCS_POLYNOMIAL 				(uint32_t)0xEDB88320
#define SIZE_OF_FCS					(uint8_t)4
#define PLCA_REG_ENABLED			(uint32_t)0b1 << 15
#define PLCA_REG_DISABLED			(uint32_t)0

#define PRINTF(fmt, ...)

/*************************************************************************************************
*  Type definitions Enums and Unions                                                             *
*************************************************************************************************/
typedef enum
{
	DNC_COMMANDTYPE_CONTROL = 0,
	DNC_COMMANDTYPE_DATA
} DNC_COMMANDTYPE;

typedef enum
{
	REG_ADDR_INCREMENT_ENABLE = 0,
	REG_ADDR_INCREMENT_DISABLE
} REG_ADDR_INCREMENT;

typedef enum
{
	REG_COMMAND_TYPE_READ = 0,
	REG_COMMAND_TYPE_WRITE
} REG_COMMAND_TYPE;

typedef union
{
	uint8_t controlHeaderArray[HEADER_FOOTER_SIZE];
	uint32_t controlFrameHead;
	struct stHeadFootBits
	{
		uint32_t P		: 1;
		uint32_t LEN  	: 7;
		uint32_t ADDR 	: 16;
		uint32_t MMS	: 4;
		uint32_t AID	: 1;
		uint32_t WNR	: 1;
		uint32_t HDRB	: 1;
		uint32_t DNC	: 1;
	}stVarHeadFootBits;
} uCommandHeaderFooter_t;

typedef union
{
	uint32_t dataFrameHeadFoot;
	uint8_t dataFrameHeaderBuffer[HEADER_FOOTER_SIZE];
	struct stTxHeadBits
	{
		uint32_t P		: 1;
		uint32_t RSVD3	: 5;
		uint32_t TS  	: 2;
		uint32_t EBO	: 6;
		uint32_t EV  	: 1;
		uint32_t RSVD2	: 1;
		uint32_t SWO	: 4;
		uint32_t SV  	: 1;
		uint32_t DV 	: 1;
		uint32_t VS		: 2;
		uint32_t RSVD1	: 5;
		uint32_t NORX	: 1;
		uint32_t SEQ	: 1;
		uint32_t DNC	: 1;
	}stVarTxHeadBits;

	struct stRxFooterBits
	{
		uint32_t P		: 1;
		uint32_t TXC	: 5;
		uint32_t RTPS  	: 1;
		uint32_t RTSA	: 1;
		uint32_t EBO  	: 6;
		uint32_t EV		: 1;
		uint32_t FD		: 1;
		uint32_t SWO  	: 4;
		uint32_t SV 	: 1;
		uint32_t DV		: 1;
		uint32_t VS		: 2;
		uint32_t RCA	: 5; //00110
		uint32_t SYNC	: 1; //0
		uint32_t HDRB	: 1; //0
		uint32_t EXST	: 1; //0
	}stVarRxFooterBits;
} uDataHeaderFooter_t;

/*************************************************************************************************
 *   	Structures
 *************************************************************************************************/
typedef struct
{
	uint8_t memoryMap;
	uint8_t length;
	uint16_t address;
	uint32_t databuffer[MAX_CONTROL_CMD_LEN];
}stControlCmdReg_t;

typedef struct
{
	uDataHeaderFooter_t receivedFooter;
	uint64_t timestamp;
}stDataReceive_t;

typedef struct
{
	uint8_t startValid;
	uint8_t startWordOffset;
	uint8_t endValid;
	uint8_t endValidOffset;
	//uint32_t databuffer[MAX_DATA_DWORD_ONECHUNK];
	uint8_t transmitCredits;
	uint8_t receiveChunkAvailable;
}stDataTransfer_t;

typedef struct
{
	uint16_t totalBytesToTransfer;
	stDataTransfer_t stVarEachChunkTransfer;
}stBulkDataTransfer_t;

typedef struct
{
	uint8_t destMACAddr[6];
	uint8_t srcMACAddr[6];
	uint16_t ethernetType;
} stEthernetFrame_t;

typedef union
{
	uint8_t receiveBuffer[MAX_RCA_VALUE * (MAX_PAYLOAD_BYTE+HEADER_FOOTER_SIZE)];
	uint8_t receiveBufferInChunks[MAX_RCA_VALUE][(MAX_PAYLOAD_BYTE+HEADER_FOOTER_SIZE)];
} uTCPIPReceiveBuffer_t;

typedef union
{
	uint32_t statusRegister0;
	struct stVarStatusReg0
	{
		uint32_t TXPE	: 1;
		uint32_t TXBOE 	: 1;
		uint32_t TXBUE 	: 1;
		uint32_t RXBOE	: 1;
		uint32_t LOFE	: 1;
		uint32_t HDRE	: 1;
		uint32_t RESETC	: 1;
		uint32_t PHYINT : 1;
		uint32_t TTSCAA : 1;
		uint32_t TTSCAB : 1;
		uint32_t TTSCAC : 1;
		uint32_t TXFCSE : 1;
		uint32_t CDPE	: 1;
		uint32_t RSVD	: 19;
	}stVarStatusReg0;
} uStatusReg0_t;

typedef struct
{
    uint64_t* data;
    uint16_t length;
    uint16_t head;
    uint16_t tail;
}timestampFIFO_t;

/*************************************************************************************************
*  		Global Variables                                                                 *
*************************************************************************************************/
uint8_t g_topoMACAddrReceived[SIZE_OF_MAC_ADDR] = {0};
uint8_t g_maxPayloadSize;
uint8_t g_transmitCreditsAvailable_TXC = 0;
const uint8_t g_broadCastMACAddr[SIZE_OF_MAC_ADDR]= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


/*************************************************************************************************
*  		Local Variables                                                                  *
*************************************************************************************************/
static uint8_t receiveMACPHYBuff[3000];
static uint8_t receiveChunksAvailable_RCA;
static bool exitFlag = false;
static bool g_isFCSEnabled = true;
static bool PLCA_MODE = PLCA_ENABLED;
static uint8_t PLCA_ID = 0 ; // used when asked to run in PLCA mode using -plca command line option
static uint8_t PLCA_MAX_NODE = 2;
static uint16_t receiveDataIndex;
static uint32_t dataReady = 1;
static uint32_t errorOccurred = 0;

/*************************************************************************************************
 *   Prototypes
 ************************************************************************************************/
//PROTOTYPE FOR HELPER FUNCTIONS NOT DEFINED IN API
uint32_t ConfigureMACPHY(void);
void CheckIfFCSEnabled(void);
uint32_t FCS_Calculator(const uint8_t * p_data, uint16_t dataSize);
bool GetParity(uint32_t valueToCalculateParity);
void ConvertEndianness(uint32_t valueToConvert, uint32_t *convertedValue);
bool T1S_WriteMACPHYReg(stControlCmdReg_t *p_regData);
bool T1S_ReadMACPHYReg(stControlCmdReg_t *p_regInfoInput,  stControlCmdReg_t *p_readRegData);
uint32_t T1S_ClearStatus(void);
uint32_t T1S_Receive(stDataReceive_t* p_dataReceiveInfo, uint16_t num_chunks_to_collect);
uint32_t CheckRCABuffAndReceiveData(bool readTCPIPDataState);
void T1S_RemoteRegWrite(uint8_t *mac_addr, uint8_t mms, uint16_t reg_address, uint32_t regValueToWrite);
void T1S_RemoteRegRead(uint8_t *mac_addr, uint8_t mms, uint16_t reg_address);
void T1S_InterpretEthernetTypeFrame(uint8_t *receiveMACPHYBuff, stEthernetFrame_t stVarEthernetFrame);

/*************************************************************************************************
*  Default OS Functions for Bare Metal                                                           *
*************************************************************************************************/

__attribute__ ((weak)) uint32_t OS_WaitUntilISR() {
	//default to bare metal call to hardware code
	return SleepUntilISR();
}

__attribute__ ((weak)) uint32_t OS_SetDataReadyCallback(ISR_t callback) {
	//default to bare metal call to hardware code
	return SetDataReadyISR(callback);
}

__attribute__ ((weak)) uint32_t OS_Init() {
	return OK;
}

__attribute__ ((weak)) void OS_Start() {
	NCN26010_Loop();
}

/*************************************************************************************************
*  Functions                                                                                     *
*************************************************************************************************/
#if !POLLING
//static int MACPHYCalls = 0;
void DataReadyCallback(uint32_t tick)
{
	dataReady = 1;
	if (errorOccurred > 0) {
		errorOccurred -= 1;
	}
}
#endif

uint32_t NCN26010_Init(void)
{
	if (SPI_Init() != OK) {
		return INIT_ERROR;
	}
	if (ConfigureMACPHY() != OK) {
		return UNKNOWN_ERROR;
	}
	#if !POLLING // This interrupt is not enabled as we moved onto polling based data reception
	if (OS_Init() != OK) {
		return INIT_ERROR;
	}
	if (OK != (OS_SetDataReadyCallback(DataReadyCallback))) //!< Enables interrupt to check if any data to be read and assigns its ISR callback
	{
		// ToDo: Handle Interrupt setup failed
		perror ("Error enabling interrupt\n");
	}
	#endif
	// ToDo Set TXFCSVE bit in CONFIG0 register if FCS and Frame padding need to be enabled

	// ToDo before doing any configuration setting, need to check if that option is enabled or not in MACPHY


	if (TCPIP_Init() != OK)
	{
		return INIT_ERROR;
	}

	return OK;
}

uint32_t get_internal_MAC_address(uint8_t* MAC_address) {

	//Retrieve PHY ID Register
	uint32_t PHYID = T1S_RegRead(0, 1);

	uint16_t PHYID2 = PHYID >> 16;
	uint16_t PHYID3 = (uint16_t)PHYID & 0x0000FFFF;
	printf("PHYID2 : %04x PHYID3: %04X ", PHYID2, PHYID3);
	// work on Byte 0
	uint8_t temp = (uint8_t)(PHYID2 >> 8) & 0xFC;
	// printf("\ntemp %x\n", temp);
	MAC_address[0] = ((temp >> 2) & 0x01) << 7;
	MAC_address[0] |= (temp >> 3 & 0x01) << 6;
	MAC_address[0] |= (temp >> 4 & 0x01) << 5;
	MAC_address[0] |= (temp >> 5 & 0x01) << 4;
	MAC_address[0] |= (temp >> 6 & 0x01) << 3;;
	MAC_address[0] |= (temp >> 7 & 0x01) << 2;
	// get Byte 1
	temp = (uint8_t)(PHYID2 >> 2) & 0x00FF;
	MAC_address[1] = (temp & 0x01) << 7;
	MAC_address[1] |= (temp >> 1 & 0x01) << 6;
	MAC_address[1] |= (temp >> 2 & 0x01) << 5;
	MAC_address[1] |= (temp >> 3 & 0x01) << 4;
	MAC_address[1] |= (temp >> 4 & 0x01) << 3;
	MAC_address[1] |= (temp >> 5 & 0x01) << 2;
	MAC_address[1] |= (temp >> 6 & 0x01) << 1;
	MAC_address[1] |= (temp >> 7 & 0x01);

	temp = (uint8_t)(PHYID3 >> 10) & 0xFF;
	MAC_address[2] = (temp & 0x01) << 7;
	MAC_address[2] |= (temp >> 1 & 0x01) << 6;
	MAC_address[2] |= (temp >> 2 & 0x01) << 5;
	MAC_address[2] |= (temp >> 3 & 0x01) << 4;
	MAC_address[2] |= (temp >> 4 & 0x01) << 3;
	MAC_address[2] |= (temp >> 5 & 0x01) << 2;
	temp = (uint8_t)(PHYID2 & 0x03);
	//printf ("temp : %d\n",temp);
	MAC_address[2] |= (temp & 0x01) << 1;
	MAC_address[2] |= (temp >> 1 & 0x01);


	uint32_t MACID0 = T1S_RegRead(12, 0x1002);
	uint32_t MACID1 = T1S_RegRead(12, 0x1003);
	MAC_address[3] = MACID0 & 0xFF00;
	MAC_address[4] = MACID0 & 0x00FF;
	MAC_address[5] = MACID1 & 0x00FF;

	//extract the Model Number and Chip Revision
	uint8_t ModelNumber = (uint8_t)(PHYID >> 4 )& 0x3F;
	uint8_t ChipRev = (uint8_t)(PHYID & 0x0000000F);
	return OK;
}

uint32_t ConfigureMACPHY()
{
	uint32_t regValue;
	if (PLCA_MODE) // User requested PLCA mode
	{
		if (PLCA_ID == 0)
		{
			regValue = PLCA_ID | (PLCA_MAX_NODE << 8);
		}
		else
		{
			regValue = PLCA_ID;
		}
		T1S_RegWrite(4, 0xCA02, regValue);
		T1S_RegWrite(4, 0xCA01,  PLCA_REG_ENABLED);
	}
	else  // Disable PLCA if not enabled during command line argument
	{
		T1S_RegWrite(4, 0xCA01,  PLCA_REG_DISABLED);
	}


	// Writes CONFIG0 register from MMS 0
	// Use 64 byte chunk
	// New frames always start at new assertion of Chip select
	// New chunk always starts at Byte offset zero
	// Set Sync bit to confirm configuration
	T1S_RegWrite(0, 0x0004, 0x000BC06);
	// Reads STATUS0 register from MMS 0
	if (T1S_ClearStatus() != OK) {
		return UNKNOWN_ERROR;
	}
	// set configure DIO LEDs to show TX and RX
	T1S_RegWrite(12, 0x0012, 0x00008D8F);

	// Write PHY Link up register
	T1S_RegWrite(0, 0xFF00, 0x00001000);

	if(USE_INTERNAL_MAC_ADDRESS) {
		get_internal_MAC_address(g_currentNodeMACAddr);
	}
	if (MAC_PROMISCUOUS_MODE) {
		// Enable TX and RX, set MAC to be promiscuous
		T1S_RegWrite(1, 0x0000, 0x00000003);
	}
	else {
		T1S_RegWrite(1, 0x0020, 0xFFFFFFFF); //look at all bits of the destination
		T1S_RegWrite(1, 0x0021, 0x0000FFFF);
		T1S_RegWrite(1, 0x0011, (g_currentNodeMACAddr[0] & 0xFF) << 8 | (g_currentNodeMACAddr[1] & 0xFF) | ENABLE_MAC_FILTER);
		T1S_RegWrite(1, 0x0010, (g_currentNodeMACAddr[2] & 0xFF) << 24 | (g_currentNodeMACAddr[3] & 0xFF) << 16 | (g_currentNodeMACAddr[4] & 0xFF) << 8 | (g_currentNodeMACAddr[5] & 0xFF));
		//filter for our MAC address (and broadcasts/multicasts)
		T1S_RegWrite(1, 0x0000, 0x00010003);
	}

	return OK;

}

void CheckIfFCSEnabled(void)
{
	uint32_t regValue;
	regValue = T1S_RegRead(1, 0x0000);
	if (regValue & 0x00000100)
	{
		PRINTF("MMS 1 FCS Auto Append has been enabled by setting bit 8 High\n");
		g_isFCSEnabled = false;//true;
	}
	else
	{
		PRINTF("MMS 1 FCS Auto Append has been disabled by setting Bit 8 Low\n");
		g_isFCSEnabled = true;// false;
	}
}




uint32_t FCS_Calculator(const uint8_t * p_data, uint16_t dataSize)
{
	uint32_t fcs = 0xFFFFFFFF;
    uint16_t buff_index;
	uint8_t data_bits;

    for (buff_index = 0; buff_index < dataSize; ++buff_index)
	{
        uint32_t val = (fcs ^ ((uint32_t) p_data[buff_index])) & 0xFF;

        for (data_bits = 0; data_bits < 8; ++data_bits)
		{
            if (val & 1)
			{
                val = (val >> 1) ^ FCS_POLYNOMIAL;
			}
            else
			{
                val >>= 1;
			}
        }

        fcs = val ^ (fcs >> 8);
    }

    return ~fcs;
}

bool GetParity(uint32_t valueToCalculateParity)
{
	valueToCalculateParity ^= valueToCalculateParity >> 1;
	valueToCalculateParity ^= valueToCalculateParity >> 2;
	valueToCalculateParity = ((valueToCalculateParity & 0x11111111U) * 0x11111111U);
	return ((valueToCalculateParity >> 28) & 1);
}

void ConvertEndianness(uint32_t valueToConvert, uint32_t *convertedValue)
{
  uint8_t position = 0;
  uint8_t variableSize = (uint8_t)(sizeof(valueToConvert));
  uint8_t tempVar = 0;
  uint8_t convertedBytes[(sizeof(valueToConvert))] = {0};

  bcopy((char *)&valueToConvert, convertedBytes, variableSize);      /** cast and copy an uint32_t to a uint8_t array **/
  position = variableSize - (uint8_t)1;
  for (uint8_t byteIndex = 0; byteIndex < (variableSize/2); byteIndex++)  /** swap bytes in this uint8_t array **/
  {
      tempVar = (uint8_t)convertedBytes[byteIndex];
      convertedBytes[byteIndex] = convertedBytes[position];
      convertedBytes[position--] = tempVar;
  }
  bcopy(convertedBytes, (uint8_t *)convertedValue, variableSize);      /* copy the swapped convertedBytes to an uint32_t */
}

bool T1S_WriteMACPHYReg(stControlCmdReg_t *p_regData)
{
	uint8_t bufferIndex = 0;
	uint8_t multiplicationFactor = 1;
	uint8_t numberOfbytesToSend = 0;
	uint8_t numberOfRegisterTosend = 0;
	//uint8_t echoedAddOffset = 0;
	bool writeStatus = true;
	const uint8_t igoredEchoedBytes = 4;
	uint8_t txBuffer[MAX_PAYLOAD_BYTE + HEADER_FOOTER_SIZE]__attribute__ ((aligned(4))) = {0};
	uint8_t rxBuffer[MAX_PAYLOAD_BYTE + HEADER_FOOTER_SIZE]__attribute__ ((aligned(4))) = {0};
	uint32_t bigEndianHeader = 0;
	uCommandHeaderFooter_t commandHeader;
	uCommandHeaderFooter_t commandHeaderEchoed;

	commandHeader.controlFrameHead = commandHeaderEchoed.controlFrameHead = 0;

	commandHeader.stVarHeadFootBits.DNC = DNC_COMMANDTYPE_CONTROL;
	commandHeader.stVarHeadFootBits.HDRB = 0;
	commandHeader.stVarHeadFootBits.WNR = REG_COMMAND_TYPE_WRITE; // Write into register
	if (p_regData->length != 0) // ToDo : Check if this option is supported and add that check
	{
		commandHeader.stVarHeadFootBits.AID = REG_ADDR_INCREMENT_ENABLE;	// Write register continously from given address
	}
	else
	{
		commandHeader.stVarHeadFootBits.AID = REG_ADDR_INCREMENT_DISABLE;	// Write into same register
	}
	commandHeader.stVarHeadFootBits.MMS = (uint32_t)(p_regData->memoryMap & 0x0F);
	commandHeader.stVarHeadFootBits.ADDR = (uint32_t)p_regData->address;
	commandHeader.stVarHeadFootBits.LEN = (uint32_t)(p_regData->length & 0x7F);
	commandHeader.stVarHeadFootBits.P = 0;
	commandHeader.stVarHeadFootBits.P = (!GetParity(commandHeader.controlFrameHead));

	for (int8_t headerByteCount = 3; headerByteCount >= 0; headerByteCount--)
	{
		txBuffer[bufferIndex++] = commandHeader.controlHeaderArray[headerByteCount];
	}

	#if 0
	if () // ToDo Add check if Protected mode in configuration(PROTE in CONFIG0) enabled, so that if protected then number of bytes in payload will be x2
	{
		multiplicationFactor = 2;
	}
	#endif

	numberOfRegisterTosend = (uint8_t)(multiplicationFactor * ((uint8_t)commandHeader.stVarHeadFootBits.LEN + (uint8_t)1));

	for (uint8_t controlRegIndex = 0; controlRegIndex < numberOfRegisterTosend; controlRegIndex++)
	{
		for (int8_t regByteIndex = 3; regByteIndex >= 0; regByteIndex--)
		{
			txBuffer[bufferIndex++] = (uint8_t)((p_regData->databuffer[controlRegIndex] >> (8 * regByteIndex)) & 0xFF); // Converting into big endian as MACPHY is big endian
		}
		// ToDo if PROTE in CONFIG0 enabled then add 1's complement in between each register data(4bytes)
		#if 0
		for (int8_t regByteIndex = 3; regByteIndex >= 0; regByteIndex--)
		{
			if (PROTE | CONFIG0) // Use correct CONFIG0 byte taken from MACPHY
			{
				txBuffer[bufferIndex++] = (uint8_t) ~((p_regData->databuffer[controlRegIndex] >> (8 * regByteIndex))) & 0xFF); // Converting into big endian as MACPHY is big endian
			}
		}
		#endif
	}

	numberOfbytesToSend = (uint8_t)(bufferIndex + HEADER_FOOTER_SIZE); // Added extra 4 bytes because last 4 bytes of payload will be ignored by MACPHY

	SPI_Transfer((uint8_t *)&rxBuffer[0], (uint8_t *)&txBuffer[0], numberOfbytesToSend);

	//echoedAddOffset = (numberOfbytesToSend - igoredEchoedBytes) - 1; // -1 at last to get correct index value to use(as bufferIndex was incremented at last)

	// ToDo Need to check CDPE bit in STATUS0 register is set, which indicates writing into register failed and update writeStatus to false

	memmove((uint8_t *)&commandHeaderEchoed.controlFrameHead, &rxBuffer[igoredEchoedBytes], HEADER_FOOTER_SIZE);
	ConvertEndianness(commandHeaderEchoed.controlFrameHead, &bigEndianHeader);
	commandHeaderEchoed.controlFrameHead = bigEndianHeader;

	#if 0 // Error Handling
	if (commandHeaderEchoed.controlFrameHead != commandHeader.controlFrameHead)
	{
		// ToDo Handle Error when sent and echoed header doesn't match, need to check for respective bits to validate what went wrong
		writeStatus = false;
	}
	else
	{
		if (0 != (memcmp(&rxBuffer[igoredEchoedBytes+HEADER_FOOTER_SIZE], &txBuffer[HEADER_FOOTER_SIZE], (bufferIndex - (igoredEchoedBytes+HEADER_FOOTER_SIZE)))))
		{
			// ToDo Error handling if echoed data doesn't match with transmitted register data(leaving header info and ignored bytes)
			writeStatus = false;
		}
	}
	#endif

	if (commandHeader.stVarHeadFootBits.MMS == 0)
	{
		bool executionStatus;
		stControlCmdReg_t stVarReadRegInfoInput;
		stControlCmdReg_t stVarReadRegData;

		// Reads CONFIG0 register from MMS 0
		stVarReadRegInfoInput.memoryMap = 0;
		stVarReadRegInfoInput.length = 0;
		stVarReadRegInfoInput.address = 0x0004;
		memset(&stVarReadRegInfoInput.databuffer[0], 0, MAX_REG_DATA_ONECHUNK);
		executionStatus = T1S_ReadMACPHYReg(&stVarReadRegInfoInput, &stVarReadRegData);
		if (executionStatus == false)
		{
			// ToDo Action to be taken if reading register fails
			PRINTF("Reading CONFIG0 reg failed after writing (inside WriteReg)\n");
		}
		else
		{
			uint8_t payloadSizeConfiguredValue;
			payloadSizeConfiguredValue = (stVarReadRegData.databuffer[0] & 0x00000007);

			switch (payloadSizeConfiguredValue)
			{
				case 3:
					g_maxPayloadSize = 8;
					break;

				case 4:
					g_maxPayloadSize = 16;
					break;

				case 5:
					g_maxPayloadSize = 32;
					break;

				case 6:
				default:
					g_maxPayloadSize = 64;
					break;
			}
			PRINTF("CONFIG0 reg value is 0x%08x in WriteReg function\n", stVarReadRegData.databuffer[0]);
		}
	}
	else if (commandHeader.stVarHeadFootBits.MMS == 1)
	{
		CheckIfFCSEnabled();
	}

	return writeStatus;
}

bool T1S_ReadMACPHYReg(stControlCmdReg_t *p_regInfoInput,  stControlCmdReg_t *p_readRegData)
{
	uint8_t bufferIndex = 0;
	const uint8_t igoredEchoedBytes = 4;
	bool readStatus = true;
	static uint8_t txBuffer[MAX_REG_DATA_ONECONTROLCMD + HEADER_FOOTER_SIZE + EACH_REG_SIZE]__attribute__ ((aligned(4))) = {0};
	static uint8_t rxBuffer[MAX_REG_DATA_ONECONTROLCMD + HEADER_FOOTER_SIZE + EACH_REG_SIZE]__attribute__ ((aligned(4))) = {0};
	uint16_t numberOfbytesToSend = 0;
	uint32_t bigEndianHeader = 0;
	uCommandHeaderFooter_t commandHeader;
	uCommandHeaderFooter_t commandHeaderEchoed;

	commandHeader.controlFrameHead = commandHeaderEchoed.controlFrameHead = 0;

	commandHeader.stVarHeadFootBits.DNC = DNC_COMMANDTYPE_CONTROL;
	commandHeader.stVarHeadFootBits.HDRB = 0;
	commandHeader.stVarHeadFootBits.WNR = REG_COMMAND_TYPE_READ; // Read from register
	if (p_regInfoInput->length != 0)
	{
		commandHeader.stVarHeadFootBits.AID = REG_ADDR_INCREMENT_ENABLE;	// Read register continously from given address
	}
	else
	{
		commandHeader.stVarHeadFootBits.AID = REG_ADDR_INCREMENT_DISABLE;	// Read from same register
	}

	commandHeader.stVarHeadFootBits.MMS = (uint32_t)p_regInfoInput->memoryMap;
	commandHeader.stVarHeadFootBits.ADDR = (uint32_t)p_regInfoInput->address;
	commandHeader.stVarHeadFootBits.LEN = (uint32_t)(p_regInfoInput->length & 0x7F);
	commandHeader.stVarHeadFootBits.P = 0;
	commandHeader.stVarHeadFootBits.P = (!GetParity(commandHeader.controlFrameHead));

	for (int8_t headerByteCount = 3; headerByteCount >= 0; headerByteCount--)
	{
		txBuffer[bufferIndex++] = commandHeader.controlHeaderArray[headerByteCount];
	}

	numberOfbytesToSend = (uint16_t)(bufferIndex + ((commandHeader.stVarHeadFootBits.LEN + 1) * EACH_REG_SIZE) + igoredEchoedBytes); // Added extra 4 bytes because first 4 bytes during reception shall be ignored

	SPI_Transfer((uint8_t *)&rxBuffer[0], (uint8_t *)&txBuffer[0], numberOfbytesToSend);

	memmove((uint8_t *)&commandHeaderEchoed.controlFrameHead, &rxBuffer[igoredEchoedBytes], HEADER_FOOTER_SIZE);

	ConvertEndianness(commandHeaderEchoed.controlFrameHead, &bigEndianHeader);
	commandHeaderEchoed.controlFrameHead = bigEndianHeader;

	if (commandHeaderEchoed.stVarHeadFootBits.HDRB != 1) // if MACPHY received header with parity error then it will be 1
	{
		uint32_t endiannessConvertedValue = 0;
		if (commandHeader.stVarHeadFootBits.LEN == 0)
		{
			memmove((uint8_t *)&p_readRegData->databuffer[0], &(rxBuffer[igoredEchoedBytes+HEADER_FOOTER_SIZE]), EACH_REG_SIZE);
			ConvertEndianness(p_readRegData->databuffer[0], &endiannessConvertedValue);
			p_readRegData->databuffer[0] = endiannessConvertedValue;
		}
		else
		{
			for (uint8_t regCount = 0; regCount <= commandHeader.stVarHeadFootBits.LEN; regCount++)
			{
				memmove((uint8_t *)&p_readRegData->databuffer[regCount], &rxBuffer[igoredEchoedBytes+HEADER_FOOTER_SIZE+(EACH_REG_SIZE * regCount)], EACH_REG_SIZE);
				ConvertEndianness(p_readRegData->databuffer[regCount], &endiannessConvertedValue);
				p_readRegData->databuffer[regCount] = endiannessConvertedValue;
			}
		}
	}
	else
	{
		// ToDo Error handling if MACPHY received with header parity error
		PRINTF("Parity Error READMACPHYReg header value : 0x%08x\n", commandHeaderEchoed.controlFrameHead );
		readStatus = false;
	}

	return readStatus;
}

uint32_t T1S_RegRead(uint8_t MMS, uint16_t Address)
{
	bool executionStatus;
	stControlCmdReg_t stVarReadRegInfoInput;
	stControlCmdReg_t stVarReadRegData;
	stVarReadRegInfoInput.memoryMap = MMS;
	stVarReadRegInfoInput.length = 0;
	stVarReadRegInfoInput.address = Address;

	executionStatus = T1S_ReadMACPHYReg(&stVarReadRegInfoInput, &stVarReadRegData);
	if (executionStatus == true)
	{
		return stVarReadRegData.databuffer[0];
	}
	else
	{
		PRINTF("ERROR: register Read failed to read %d.%4x", MMS, Address);
		return 0xDEAD;
	}
}

uint32_t T1S_RegWrite(uint8_t MMS, uint16_t Address, uint32_t data)
{
	stControlCmdReg_t stVarWriteRegInput;
	stVarWriteRegInput.memoryMap = MMS;
	stVarWriteRegInput.length = 0;
	stVarWriteRegInput.address = Address;
	stVarWriteRegInput.databuffer[0] = data;
	bool executionStatus = T1S_WriteMACPHYReg(&stVarWriteRegInput);
	if (executionStatus == false)
	{
		// ToDo Action to be taken if reading register fails
		PRINTF("Writing into STATUS0 reg failed while clearing error\n");
		return 0;
	}
	else
	{
		return stVarWriteRegInput.databuffer[0];
	}
}

uint32_t T1S_ClearStatus(void)
{
	uint32_t statusReg;

	statusReg = T1S_RegRead(0, 0x0008);
	T1S_RegWrite(0, 0x0008, statusReg); //write back status to clear errors
	statusReg = T1S_RegRead(0, 0x0008);
	if (STATUS_OK(statusReg)) {
		return OK;
	}
	else {
		return UNKNOWN_ERROR;
	}
}

uint32_t T1S_Transmit(uint8_t* p_tcpipDataBuffer, uint16_t num_bytes_to_transmit)
{
	uint8_t txBuffer[2108]__attribute__ ((aligned(4))) = {0}; //! Total required chunks is 31(31 x 64) but kept one extra chunk bytes added extra to buffer so total 32x64
	uint8_t rxBuffer[2108]__attribute__ ((aligned(4))) = {0};

	uint16_t sentChunkCount = 0;
	uint16_t numberOfChunksToSend = 0;
	uint16_t numberOfBytesToSend = 0;
	uint16_t bytesToTransmit = num_bytes_to_transmit;
	uint32_t bigEndianRxFooter = 0;
	static uDataHeaderFooter_t dataTransferHeader;
	uDataHeaderFooter_t datatransferRxFooter;

	if (g_isFCSEnabled)
	{
		uint32_t fcsCalculated = 0;

		if (bytesToTransmit < (MAX_PAYLOAD_BYTE - HEADER_FOOTER_SIZE))
		{
			uint8_t frameCountDiff = (MAX_PAYLOAD_BYTE - HEADER_FOOTER_SIZE) - bytesToTransmit;
			memset(&p_tcpipDataBuffer[bytesToTransmit], 0x00, frameCountDiff);
			bytesToTransmit += frameCountDiff;
		}
		fcsCalculated = FCS_Calculator(&p_tcpipDataBuffer[0], (uint32_t)bytesToTransmit);
		p_tcpipDataBuffer[bytesToTransmit] = (uint8_t)fcsCalculated;
		p_tcpipDataBuffer[bytesToTransmit + 1] = (uint8_t)(fcsCalculated >> 8);
		p_tcpipDataBuffer[bytesToTransmit + 2] = (uint8_t)(fcsCalculated >> 16);
		p_tcpipDataBuffer[bytesToTransmit + 3] = (uint8_t)(fcsCalculated >> 24);
		bytesToTransmit += SIZE_OF_FCS;
	}

	//ToDo Handle FCS and frame padding here by checking TXFCSVC bit in STDCAP register
	numberOfChunksToSend  = (bytesToTransmit/g_maxPayloadSize);
	if ((bytesToTransmit % g_maxPayloadSize) > 0)
	{
		numberOfChunksToSend++;
	}
	//PRINTF("Number of bytes to send %d, Number of chunks %d\n", bytesToTransmit, numberOfChunksToSend);

	for (sentChunkCount = 0; sentChunkCount < numberOfChunksToSend; sentChunkCount++)
	{
		uint8_t bufferIndex = 0;
		dataTransferHeader.stVarTxHeadBits.DNC = DNC_COMMANDTYPE_DATA;
		//if (SEQE | CONFIG0) //ToDo Add check here to verify if SEQ check if enabled in config register
		{
			dataTransferHeader.stVarTxHeadBits.SEQ = (uint8_t)(~dataTransferHeader.stVarTxHeadBits.SEQ);
		}
		//else
		{
			dataTransferHeader.stVarTxHeadBits.SEQ = 0;
		}

		dataTransferHeader.stVarTxHeadBits.NORX = 0;
		dataTransferHeader.stVarTxHeadBits.RSVD1 = 0;
		dataTransferHeader.stVarTxHeadBits.VS = 0;
		dataTransferHeader.stVarTxHeadBits.DV = 1;
		dataTransferHeader.stVarTxHeadBits.RSVD2 = 0;
		dataTransferHeader.stVarTxHeadBits.TS = 0;
		dataTransferHeader.stVarTxHeadBits.RSVD3 = 0;
		dataTransferHeader.stVarTxHeadBits.P = 0;

		if (sentChunkCount == 0)
		{
			dataTransferHeader.stVarTxHeadBits.SV = 1;
			dataTransferHeader.stVarTxHeadBits.SWO = 0;
		}
		else
		{
			dataTransferHeader.stVarTxHeadBits.SV = 0;
		}

		if (sentChunkCount == (numberOfChunksToSend-1))
		{
			dataTransferHeader.stVarTxHeadBits.EV = 1;
			if ((bytesToTransmit % g_maxPayloadSize) > 0)
			{
				dataTransferHeader.stVarTxHeadBits.EBO = ((bytesToTransmit % g_maxPayloadSize) - 1);
			}
			else
			{
				dataTransferHeader.stVarTxHeadBits.EBO = (g_maxPayloadSize-1);
			}
		}
		else
		{
			dataTransferHeader.stVarTxHeadBits.EV = 0;
			dataTransferHeader.stVarTxHeadBits.EBO = 0;
		}
		//PRINTF("EBO is BulkTransmit is %d \n", dataTransferHeader.stVarTxHeadBits.EBO);
		dataTransferHeader.stVarTxHeadBits.P = (!GetParity(dataTransferHeader.dataFrameHeadFoot));

		for (int8_t headerByteCount = 3; headerByteCount >= 0; headerByteCount--)
		{
			txBuffer[(sentChunkCount*(g_maxPayloadSize + HEADER_FOOTER_SIZE)) + bufferIndex] = dataTransferHeader.dataFrameHeaderBuffer[headerByteCount];
			bufferIndex++;
		}

		numberOfBytesToSend += bufferIndex;
		memcpy(&txBuffer[(sentChunkCount*(g_maxPayloadSize + HEADER_FOOTER_SIZE)) + bufferIndex], &p_tcpipDataBuffer[(sentChunkCount*g_maxPayloadSize)], g_maxPayloadSize);
		numberOfBytesToSend += g_maxPayloadSize;
		//PRINTF("Sending: Chunk Count %d, number of Bytes added to buff %d, Header 0x%08X \n", sentChunkCount, numberOfBytesToSend, dataTransferHeader.dataFrameHeadFoot);

		if (sentChunkCount == (numberOfChunksToSend-1))
		{
			SPI_Transfer((uint8_t *)&rxBuffer[0], (uint8_t *)&txBuffer[0], (uint16_t)numberOfBytesToSend);
		}
	}

	for (sentChunkCount = 0; sentChunkCount < numberOfChunksToSend; sentChunkCount++)
	{
		bool dataTransmissionStatus = true;
		uint16_t rxBufferIndex = (sentChunkCount * (g_maxPayloadSize+HEADER_FOOTER_SIZE));

		memmove((uint8_t *)&datatransferRxFooter.dataFrameHeadFoot, &(rxBuffer[rxBufferIndex + g_maxPayloadSize]), HEADER_FOOTER_SIZE);
		ConvertEndianness(datatransferRxFooter.dataFrameHeadFoot, &bigEndianRxFooter);
		datatransferRxFooter.dataFrameHeadFoot = bigEndianRxFooter;
		//PRINTF("Reception: Chunk Count %d, Footer 0x%08X \n", sentChunkCount, datatransferRxFooter.dataFrameHeadFoot);
		if (dataTransferHeader.stVarTxHeadBits.NORX == 0)
		{
			//ToDo Calculate received footer parity first and then go for next bits validation
			if (datatransferRxFooter.stVarRxFooterBits.EXST == 1)
			{
				// ToDo Error Handling based on checking which bits are set in STATUS registers
				dataTransmissionStatus = false;
				T1S_ClearStatus();
				PRINTF("EXST Bit set in TransmitData, Footer: 0x%08x\n", datatransferRxFooter.dataFrameHeadFoot);
			}

			if (datatransferRxFooter.stVarRxFooterBits.SYNC == 0)
			{
				// ToDo : SYNC bit indicates MACPHY configuration is not same as SPI so need to update caller of this function to configure MACPHY
				PRINTF("SYNC failure occured in TransmitData, Footer: 0x%08x\n", datatransferRxFooter.dataFrameHeadFoot);
				ConfigureMACPHY();
				//CheckConfigFileAndConfigMACPHY();
				dataTransmissionStatus = false;
			}

			if (datatransferRxFooter.stVarRxFooterBits.HDRB == 1)
			{
				PRINTF("Header Bad in TransmitData, Footer: 0x%08x\n", datatransferRxFooter.dataFrameHeadFoot);
				dataTransmissionStatus = false;
			}

			if (datatransferRxFooter.stVarRxFooterBits.DV == 1)
			{
				uint8_t bufferIndexStartOffset = 0;
				// This indicates if received bytes are valid and need to update User about this data
				// ToDo : Check how to handle this data and do what with this
				if (datatransferRxFooter.stVarRxFooterBits.SV == 1)
				{
					//PRINTF("I got SV bit set in TransmitDataChunk, RCA value is %d\n", (uint8_t)datatransferRxFooter.stVarRxFooterBits.RCA);
					if (datatransferRxFooter.stVarRxFooterBits.SWO > 0)
					{
						bufferIndexStartOffset = (uint8_t)datatransferRxFooter.stVarRxFooterBits.SWO;
					}
					//dataReceptionInProgress = true;
					receiveDataIndex = 0;
					(void)memcpy(&receiveMACPHYBuff[receiveDataIndex], &rxBuffer[rxBufferIndex+bufferIndexStartOffset], (g_maxPayloadSize - bufferIndexStartOffset));
					receiveDataIndex = g_maxPayloadSize - bufferIndexStartOffset;
				}

				if (datatransferRxFooter.stVarRxFooterBits.EV == 1)
				{
					//PRINTF("I got EV bit set in TransmitDataChunk\n");
					if (datatransferRxFooter.stVarRxFooterBits.FD == 1)
					{
						// ToDo Frame Drop bit set meaning, current receiving frame is not valid so need to read again - happens only in cut through mode
						PRINTF("Frame Drop in TransmitData, Footer: 0x%08x\n", datatransferRxFooter.dataFrameHeadFoot);
						dataTransmissionStatus = false;
					}
					//else
					{
						(void)memcpy(&receiveMACPHYBuff[receiveDataIndex], &rxBuffer[rxBufferIndex], (datatransferRxFooter.stVarRxFooterBits.EBO+1)); // +1 here as checksum is 4 bytes and so -3 is made whiles sending data into TCPIP
						receiveDataIndex += (datatransferRxFooter.stVarRxFooterBits.EBO+1);
						if (g_isFCSEnabled)
						{
							uint32_t fcsCalculated = 0;
							uint32_t fcsReceived = 0;
							fcsCalculated = FCS_Calculator(&receiveMACPHYBuff[0], (uint32_t)(receiveDataIndex-SIZE_OF_FCS));
							fcsReceived = receiveMACPHYBuff[(receiveDataIndex-SIZE_OF_FCS) + 3];
							fcsReceived = (uint32_t)(((fcsReceived << 8) | (uint8_t)receiveMACPHYBuff[(receiveDataIndex-SIZE_OF_FCS) + 2]));
							fcsReceived = (uint32_t)(((fcsReceived << 8) | (uint8_t)receiveMACPHYBuff[(receiveDataIndex-SIZE_OF_FCS) + 1]));
							fcsReceived = (uint32_t)((fcsReceived << 8) | (uint8_t)receiveMACPHYBuff[(receiveDataIndex-SIZE_OF_FCS)]);
							if (fcsReceived == fcsCalculated)
							{
								//PRINTF("FCS Received in Bulk data transmission was correct, received is 0x%08X and calculated is 0x%08X\n", fcsReceived, fcsCalculated);
							}
							else
							{
								PRINTF("FCS Received in Bulk data transmission was wrong, received is 0x%08X and calculated is 0x%08X\n", fcsReceived, fcsCalculated);
							}
						}
						//PRINTF("Wrote into TCPIP at Transmit Data Chunk side, Num of bytes : %d \n", (receiveDataIndex + (datatransferRxFooter.stVarRxFooterBits.EBO - 3)));
						if (TCPIP_Receive(&receiveMACPHYBuff[0], receiveDataIndex) == OK)
						{
							receiveDataIndex = 0;
						}
						//PRINTF("RCA value after sending data to TCPIP in TransmitDataChunk is %d\n", (uint8_t)datatransferRxFooter.stVarRxFooterBits.RCA);
					}
				}

				if (((datatransferRxFooter.stVarRxFooterBits.SV != 1) && (datatransferRxFooter.stVarRxFooterBits.EV != 1)) && (dataTransmissionStatus != false))
				{
					(void)memcpy(&receiveMACPHYBuff[receiveDataIndex], &rxBuffer[rxBufferIndex], g_maxPayloadSize);
					//PRINTF("SV bit not set and EV not set in TransmitDataChunk, Chunk count %d \n", sentChunkCount);
					receiveDataIndex += g_maxPayloadSize;
				}
			}
			else
			{
				//PRINTF("Received data is not valid, chunk count %d\n", sentChunkCount);
			}

			if (dataTransmissionStatus == false)
			{
				(void)memset(&receiveMACPHYBuff[0], 0, sizeof(receiveMACPHYBuff));
				receiveDataIndex = 0;
				PRINTF("Clearing data reception process in TransmitDataChunk due to Footer: 0x%08x where sentChunkCount of Tx is %d\n", datatransferRxFooter.dataFrameHeadFoot, sentChunkCount);
				return UNKNOWN_ERROR;
			}
		}
	}


	return OK;
}

uint32_t T1S_Receive(stDataReceive_t* p_dataReceiveInfo, uint16_t num_chunks_to_collect)
{
	uint32_t errorCode = OK;
	uint8_t txBuffer[2108]__attribute__ ((aligned(4))) = {0}; //! Total required chunks is 31(31 x 64) but kept one extra chunk bytes added extra to buffer so total 32x64
	uint8_t rxBuffer[2108]__attribute__ ((aligned(4))) = {0};
	uint8_t bufferIndex = 0;
	uint8_t receiveDataStartOffset = 0;
	uint8_t numberOfChunksToReceive = 0;
	uint16_t numberOfBytesToReceive = 0;
	uint32_t bigEndianRxFooter = 0;
	static uDataHeaderFooter_t dataTransferHeader;
	uDataHeaderFooter_t datatransferRxFooter;

	dataTransferHeader.stVarTxHeadBits.DNC = DNC_COMMANDTYPE_DATA;
	//if (SEQE | CONFIG0) //ToDo Add check here to verify if SEQ check if enabled in config register
	{
		dataTransferHeader.stVarTxHeadBits.SEQ = ~dataTransferHeader.stVarTxHeadBits.SEQ;
	}
	//else
	{
		dataTransferHeader.stVarTxHeadBits.SEQ = 0;
	}

	dataTransferHeader.stVarTxHeadBits.NORX = 0;
	dataTransferHeader.stVarTxHeadBits.VS = 0;
	dataTransferHeader.stVarTxHeadBits.RSVD1 = 0;
	dataTransferHeader.stVarTxHeadBits.DV = 0;
	dataTransferHeader.stVarTxHeadBits.SV = 0;
	dataTransferHeader.stVarTxHeadBits.SWO = 0;
	dataTransferHeader.stVarTxHeadBits.RSVD2 = 0;
	dataTransferHeader.stVarTxHeadBits.EV = 0;
	dataTransferHeader.stVarTxHeadBits.EBO = 0;
	dataTransferHeader.stVarTxHeadBits.RSVD3 = 0;
	dataTransferHeader.stVarTxHeadBits.TS = 0;
	dataTransferHeader.stVarTxHeadBits.P = 0;
	dataTransferHeader.stVarTxHeadBits.P = (!GetParity(dataTransferHeader.dataFrameHeadFoot));

	//ToDo Handle FCS and frame padding here by checking TXFCSVC bit in STDCAP register
	if (num_chunks_to_collect == 0xFF)
	{
		numberOfChunksToReceive = 1;
	}
	else
	{
		numberOfChunksToReceive = num_chunks_to_collect;
	}

	for (uint8_t headerForChunks = 0; headerForChunks < numberOfChunksToReceive; headerForChunks++)
	{
		bufferIndex = 0;
		for (int8_t headerByteCount = 3; headerByteCount >= 0; headerByteCount--)
		{
			txBuffer[(headerForChunks * (g_maxPayloadSize+HEADER_FOOTER_SIZE)) + bufferIndex] = dataTransferHeader.dataFrameHeaderBuffer[headerByteCount];
			bufferIndex++;
		}
	}

	numberOfBytesToReceive = numberOfChunksToReceive * (g_maxPayloadSize+HEADER_FOOTER_SIZE);

	SPI_Transfer((uint8_t *)&rxBuffer[0], (uint8_t *)&txBuffer[0], numberOfBytesToReceive);

	for (uint8_t headerForChunks = 0; headerForChunks < numberOfChunksToReceive; headerForChunks++)
	{
		uint16_t rxBufferIndexChunk = (headerForChunks * (g_maxPayloadSize+HEADER_FOOTER_SIZE));
		memmove((uint8_t *)&datatransferRxFooter.dataFrameHeadFoot, &rxBuffer[(rxBufferIndexChunk + g_maxPayloadSize)], HEADER_FOOTER_SIZE);
		ConvertEndianness(datatransferRxFooter.dataFrameHeadFoot, &bigEndianRxFooter);
		p_dataReceiveInfo->receivedFooter.dataFrameHeadFoot = datatransferRxFooter.dataFrameHeadFoot = bigEndianRxFooter;
		//if (datatransferRxFooter.stVarRxFooterBits.P == (!(GetParity(datatransferRxFooter.dataFrameHeadFoot)))) // This needs to be enabled
		//if (datatransferRxFooter.stVarRxFooterBits.P == ((GetParity(datatransferRxFooter.dataFrameHeadFoot))))
		{
			if (datatransferRxFooter.stVarRxFooterBits.EXST == 1)
			{
				PRINTF("EXST bit set high in ReceiveDataChunk of %d chunk count, Footer value is : 0x%08x\n", headerForChunks, datatransferRxFooter.dataFrameHeadFoot);
				T1S_ClearStatus();
				errorCode = UNKNOWN_ERROR;
				//receiveDataStatus = false;  // This isn't required because STATUS0 register cleared
			}

			if (datatransferRxFooter.stVarRxFooterBits.DV == 1)
			{
				//PRINTF("ReceiveDataChunk Footer value is : 0x%08x\n", datatransferRxFooter.dataFrameHeadFoot);
				if (datatransferRxFooter.stVarRxFooterBits.HDRB == 1)
				{
					PRINTF("HDRB bit set high in %d chunk count, Footer value is : 0x%08x\n", headerForChunks, datatransferRxFooter.dataFrameHeadFoot);
					errorCode = UNKNOWN_ERROR;
				}

				if (datatransferRxFooter.stVarRxFooterBits.SYNC == 0)
				{
					PRINTF("SYNC failure in %d chunk count, Footer value is : 0x%08x\n", headerForChunks, datatransferRxFooter.dataFrameHeadFoot);
					//p_dataReceiveInfo->syncFailure = true;
					ConfigureMACPHY();
					//CheckConfigFileAndConfigMACPHY();
					// ToDo: Check if function has to return from here
					errorCode = UNKNOWN_ERROR;
				}
				else
				{
					//p_dataReceiveInfo->syncFailure = false;
				}

				if (datatransferRxFooter.stVarRxFooterBits.SV == 1)
				{
					receiveDataIndex = 0;
					receiveDataStartOffset = (uint8_t)datatransferRxFooter.stVarRxFooterBits.SWO;
					if (datatransferRxFooter.stVarRxFooterBits.EV != 1)
					{
						(void)memcpy(&receiveMACPHYBuff[receiveDataIndex], &rxBuffer[(rxBufferIndexChunk + receiveDataStartOffset)], (g_maxPayloadSize-receiveDataStartOffset));
						receiveDataIndex = g_maxPayloadSize - receiveDataStartOffset;
					}
					//PRINTF("I got SV bit set in ReceiveDataChunk, receiveDataIndex = %d, RCA value is %d\n", receiveDataIndex, (uint8_t)datatransferRxFooter.stVarRxFooterBits.RCA);
				}

				if (datatransferRxFooter.stVarRxFooterBits.EV == 1)
				{
					//PRINTF("I got EV bit set in ReceiveDataChunk, RCA value is %d\n", (uint8_t)datatransferRxFooter.stVarRxFooterBits.RCA);
					//receiveDataEndOffset = datatransferRxFooter.stVarRxFooterBits.EBO;
					if (datatransferRxFooter.stVarRxFooterBits.FD == 1)
					{
						PRINTF("Frame Drop bit Set in %d chunk count, Footer value is : 0x%08x\n", headerForChunks, datatransferRxFooter.dataFrameHeadFoot);
						errorCode = UNKNOWN_ERROR;
					}
					else
					{
						stEthernetFrame_t stVarEthernetFrame = {0};
						(void)memcpy(&receiveMACPHYBuff[receiveDataIndex], &rxBuffer[rxBufferIndexChunk], (datatransferRxFooter.stVarRxFooterBits.EBO+1));
						receiveDataIndex += (datatransferRxFooter.stVarRxFooterBits.EBO+1);
						(void)memcpy(&stVarEthernetFrame.destMACAddr[0], &receiveMACPHYBuff[0], (size_t)(sizeof(stEthernetFrame_t)));
						//PRINTF("EBO is BulkReception is %d and Receive Data Index is %d \n", datatransferRxFooter.stVarRxFooterBits.EBO+1, receiveDataIndex);
						if (g_isFCSEnabled)
						{
							uint32_t fcsCalculated = 0;
							uint32_t fcsReceived = 0;
							fcsCalculated = FCS_Calculator(&receiveMACPHYBuff[0], (uint32_t)(receiveDataIndex-SIZE_OF_FCS));
							fcsReceived = receiveMACPHYBuff[(receiveDataIndex-SIZE_OF_FCS) + 3];
							fcsReceived = (uint32_t)(((fcsReceived << 8) | (uint8_t)receiveMACPHYBuff[(receiveDataIndex-SIZE_OF_FCS) + 2]));
							fcsReceived = (uint32_t)(((fcsReceived << 8) | (uint8_t)receiveMACPHYBuff[(receiveDataIndex-SIZE_OF_FCS) + 1]));
							fcsReceived = (uint32_t)((fcsReceived << 8) | (uint8_t)receiveMACPHYBuff[(receiveDataIndex-SIZE_OF_FCS)]);
							if (fcsReceived == fcsCalculated)
							{
								//PRINTF("FCS Received in Bulk data reception was correct, received is 0x%08X and calculated is 0x%08X\n", fcsReceived, fcsCalculated);
							}
							else
							{
								PRINTF("FCS Received in Bulk data reception was wrong, received is 0x%08X and calculated is 0x%08X\n", fcsReceived, fcsCalculated);
								errorCode = UNKNOWN_ERROR;
							}
						}
#if REMOTE_CONFIGURATION_ENABLED
						//PRINTF("Ether Type Received is 0x%04X\n", stVarEthernetFrame.ethernetType);
						///< If Ethernet Type is 0x0909 that is Configuration Ethernet frame
						if (stVarEthernetFrame.ethernetType == ETHERNET_TYPE_MACPHYCONFIG)
						{
							//PRINTF("Received EtherType 0x%04X Received MAC Dest Address is 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", stVarEthernetFrame.ethernetType, stVarEthernetFrame.destMACAddr[0], stVarEthernetFrame.destMACAddr[1], stVarEthernetFrame.destMACAddr[2],stVarEthernetFrame.destMACAddr[3],stVarEthernetFrame.destMACAddr[4],stVarEthernetFrame.destMACAddr[5]);
							///< Only if addressed to this device or if broadcast configuration for all nodes
							if ((0 == (memcmp(&stVarEthernetFrame.destMACAddr[0], &g_currentNodeMACAddr[0], SIZE_OF_MAC_ADDR))) || \
									(0 == (memcmp(&stVarEthernetFrame.destMACAddr[0], &g_broadCastMACAddr[0], SIZE_OF_MAC_ADDR))))
							{
								T1S_InterpretEthernetTypeFrame(&receiveMACPHYBuff[0], stVarEthernetFrame);
							}
							else
							{
								PRINTF("Received EtherType %d but not to our MAC Address, Received MAC Dest Address is 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", \
								stVarEthernetFrame.ethernetType, stVarEthernetFrame.destMACAddr[0], stVarEthernetFrame.destMACAddr[1], stVarEthernetFrame.destMACAddr[2],\
										stVarEthernetFrame.destMACAddr[3],stVarEthernetFrame.destMACAddr[4],stVarEthernetFrame.destMACAddr[5]);
							}
						}
						else
#endif
						{
							uint16_t bytesToSendOverTCPIP = receiveDataIndex;
							if (g_isFCSEnabled)
							{
								bytesToSendOverTCPIP = (receiveDataIndex-SIZE_OF_FCS);
							}

							if (TCPIP_Receive(&receiveMACPHYBuff[0], bytesToSendOverTCPIP) == OK)
							{
								receiveDataIndex = 0;
							}
							//PRINTF("Wrote into TCPIP at Receive Data Chunk side, receiveDataIndex = %d, datatransferRxFooter.stVarRxFooterBits.EBO = %d, Num of bytes : %d \n", receiveDataIndex, datatransferRxFooter.stVarRxFooterBits.EBO, (bytesToSendOverTCPIP + (datatransferRxFooter.stVarRxFooterBits.EBO - 3)));

							//PRINTF("RCA value after sending data to TCPIP in Receive Data Chunk is %d\n", (uint8_t)datatransferRxFooter.stVarRxFooterBits.RCA);
						}
					}
				}

				if (((datatransferRxFooter.stVarRxFooterBits.SV != 1) && (datatransferRxFooter.stVarRxFooterBits.EV != 1)) && (errorCode == OK))
				{
					(void)memcpy(&receiveMACPHYBuff[receiveDataIndex], &rxBuffer[rxBufferIndexChunk], g_maxPayloadSize);
					receiveDataIndex += g_maxPayloadSize;
					//PRINTF("SV bit not set and EV not set in ReceiveDataChunk\n");
				}

				if (errorCode != OK)
				{
					(void)memset(&receiveMACPHYBuff[0], 0, sizeof(receiveMACPHYBuff));
					receiveDataIndex = 0;
					PRINTF("Clearing data reception process in ReceiveDataChunk of %d chunk count due to Footer: 0x%08x\n", headerForChunks, datatransferRxFooter.dataFrameHeadFoot);
				}
			}
		}
	}

	return errorCode;
}

void ReadBufferStatusReg(void)
{
	bool executionStatus;
	stControlCmdReg_t stVarReadRegInfoInput;
	stControlCmdReg_t stVarReadRegData;

	// Reads Buffer Status register from MMS 0
	stVarReadRegInfoInput.memoryMap = 0;
	stVarReadRegInfoInput.length = 0;
	stVarReadRegInfoInput.address = 0x000B;
	//memset(&stVarReadRegInfoInput.databuffer[0], 0, MAX_REG_DATA_ONECHUNK);
	executionStatus = T1S_ReadMACPHYReg(&stVarReadRegInfoInput, &stVarReadRegData);
	if (executionStatus == false)
	{
		// ToDo Action to be taken if reading register fails
		PRINTF("Reading Buffer Status register failed\n");
		g_transmitCreditsAvailable_TXC = 0;
		receiveChunksAvailable_RCA = 0;
	}
	else
	{
		//PRINTF("Buffer Status reg value is 0x%08x \r", stVarReadRegData.databuffer[0]);
		receiveChunksAvailable_RCA = (uint8_t)(stVarReadRegData.databuffer[0] & 0x000000FF);
		g_transmitCreditsAvailable_TXC = (uint8_t)((stVarReadRegData.databuffer[0] >> 8) & 0x000000FF);
	}
}

uint32_t CheckRCABuffAndReceiveData(bool readTCPIPDataState)
{
	uint32_t errorCode;
	stDataReceive_t stVarReceiveData;
	//stControlCmdReg_t stVarReadRegData;
	ReadBufferStatusReg();

	if (receiveChunksAvailable_RCA > 0)
	{
		bool firstTimeCollect = false;
		uint8_t num_chunks_to_collect = 0;
		do
		{
			if (firstTimeCollect == false)
			{
				num_chunks_to_collect = 0xFF;
				firstTimeCollect = true;
			}
			else
			{
				num_chunks_to_collect = stVarReceiveData.receivedFooter.stVarRxFooterBits.RCA;
			}
			if ((errorCode = T1S_Receive(&stVarReceiveData, num_chunks_to_collect)) != OK)
			{
				PRINTF("Failed Receiving data chunk during IRQ request\n");
				dataReady = 1;
				errorOccurred = 1; //poll until 10 Interrupts to make sure its cleared out
			}
			else
			{
				g_transmitCreditsAvailable_TXC = stVarReceiveData.receivedFooter.stVarRxFooterBits.TXC;
			}
		} while (stVarReceiveData.receivedFooter.stVarRxFooterBits.RCA != 0);
	}
	else {
		dataReady = 0;
	}


	return OK;
}

uint32_t NCN26010_Loop(void)
{
	//bool executionStatus;
	bool readTCPIPDataStatus = true;
	//stDataReceive_t stVarReceiveData;
	while (1)
	{
		CheckRCABuffAndReceiveData(readTCPIPDataStatus);

		TCPIP_PollCallback(0);
		if (dataReady == 0 && !errorOccurred) {
			//NOTE IF THE INTERRUPT HAPPENS BETWEEN THIS CHECK AND WHEN WE GO TO SLEEP WE MAY MISS ONE INTERRUPT
			//NCN26010 BUFFER WILL HOLD THAT DATA SO THE DATA WON'T BE LOST BUT THERE MAY BE DELAY
			OS_WaitUntilISR();
		}

		if (exitFlag == true)
		{
			break;
		}
	}
	SPI_Cleanup();
	//return 0;
	return OK;
}

uint32_t NCN26010_Exit()
{
	exitFlag = true;
	return OK;
}


uint32_t T1S_ConfigurePLCA(uint8_t id, uint8_t max_id, bool mode)
{
	PLCA_ID = id;
	if (PLCA_ID == 0)
	{
		PLCA_MAX_NODE = max_id;
	}
	PLCA_MODE = mode;
	PRINTF ("Configuring PLCA , Node ID :%d, max_node_ID : %d\n", PLCA_ID, PLCA_MAX_NODE);
	return OK;
}


#if REMOTE_CONFIGURATION_ENABLED

void T1S_InterpretEthernetTypeFrame(uint8_t *receiveMACPHYBuff, stEthernetFrame_t stVarEthernetFrame)
{
	uint8_t numberOfConfigToDo = 0;
	bool isResponseToBeSent = false;
	bool executionStatus;
	uint8_t messageType = 0;
	uint8_t regReadResponse_SendBuffIndex = 0;
	uint8_t regReadResponseBuff[1000] = {0};
	uint16_t receivedDataBuffIndex = 0;

	PRINTF("\n Received Configuration Ethernet Frame with EtherType 0x%04X and MAC Dest Addr is 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x And Src Address is 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n", \
			stVarEthernetFrame.ethernetType, stVarEthernetFrame.destMACAddr[0], stVarEthernetFrame.destMACAddr[1], stVarEthernetFrame.destMACAddr[2],\
			stVarEthernetFrame.destMACAddr[3], stVarEthernetFrame.destMACAddr[4], stVarEthernetFrame.destMACAddr[5], stVarEthernetFrame.srcMACAddr[0],\
				stVarEthernetFrame.srcMACAddr[1], stVarEthernetFrame.srcMACAddr[2], stVarEthernetFrame.srcMACAddr[3], stVarEthernetFrame.srcMACAddr[4],
				stVarEthernetFrame.srcMACAddr[5]);

	receivedDataBuffIndex = (sizeof(stEthernetFrame_t));
	messageType = receiveMACPHYBuff[receivedDataBuffIndex++];
	memcpy(&regReadResponseBuff[0], &stVarEthernetFrame.srcMACAddr[0], SIZE_OF_MAC_ADDR);
	regReadResponse_SendBuffIndex += SIZE_OF_MAC_ADDR;
	memcpy(&regReadResponseBuff[regReadResponse_SendBuffIndex], &g_currentNodeMACAddr[0], SIZE_OF_MAC_ADDR);
	regReadResponse_SendBuffIndex += SIZE_OF_MAC_ADDR;
	memcpy(&regReadResponseBuff[regReadResponse_SendBuffIndex], &stVarEthernetFrame.ethernetType, sizeof(stVarEthernetFrame.ethernetType));
	regReadResponse_SendBuffIndex += sizeof(stVarEthernetFrame.ethernetType);
	numberOfConfigToDo = receiveMACPHYBuff[receivedDataBuffIndex++];
	/*for (uint16_t receiveIndexCount = 0; receiveIndexCount < receiveDataIndex; receiveIndexCount++)
	{
		PRINTF("0x%02X ", receiveMACPHYBuff[receiveIndexCount]);
	}
	PRINTF("\nEnd of Received Bytes in EtherType 0909\n");*/
	//PRINTF("Received Ethernet frame, need to verify if Topo Disc volt measurement\n");

	if (messageType == MESSAGETYPE_SEND)
	{
		//PRINTF("Step 2 inside Received Ethernet frame, need to verify if Topo Disc volt measurement\n");
		regReadResponseBuff[regReadResponse_SendBuffIndex++] = MESSAGETYPE_RESPONSE;
		regReadResponseBuff[regReadResponse_SendBuffIndex++] = numberOfConfigToDo;
		for (uint8_t configCount = 0; configCount < numberOfConfigToDo; configCount++)
		{
			//uint16_t regAddr = 0;
			stControlCmdReg_t stVarReadRegInfoInput;
			stControlCmdReg_t stVarReadRegData;
			stControlCmdReg_t stVarWriteRegInput;
			uint8_t typeOfConfig = receiveMACPHYBuff[receivedDataBuffIndex++];
			regReadResponseBuff[regReadResponse_SendBuffIndex++] = typeOfConfig;
			//PRINTF("Received Type of config is %d and MMS %d and Address 0x%04X\n", typeOfConfig,  receiveMACPHYBuff[receivedDataBuffIndex],\
					 (uint16_t)receiveMACPHYBuff[receivedDataBuffIndex+1]);
			if (typeOfConfig == CONFIGTYPE_REGREAD)
			{
			//	PRINTF("Received Register read command from other Ethernet node \n");
				isResponseToBeSent = true;
				stVarReadRegInfoInput.memoryMap = receiveMACPHYBuff[receivedDataBuffIndex++];
				stVarReadRegInfoInput.length = 0;
				//regAddr = (uint16_t)receiveMACPHYBuff[receivedDataBuffIndex++];
				stVarReadRegInfoInput.address =  ((receiveMACPHYBuff[receivedDataBuffIndex+1] << 8) | receiveMACPHYBuff[receivedDataBuffIndex]);//(uint16_t)((receiveMACPHYBuff[receivedDataBuffIndex++] << 8) | regAddr);
				receivedDataBuffIndex += sizeof(stVarReadRegInfoInput.address);
				memset(&stVarReadRegInfoInput.databuffer[0], 0, MAX_REG_DATA_ONECHUNK);
				executionStatus = T1S_ReadMACPHYReg(&stVarReadRegInfoInput, &stVarReadRegData);
				if (executionStatus == false)
				{
					// ToDo Action to be taken if reading register fails
					PRINTF("Reading address 0x%04X of MMS %d in Node Register read failed\n", stVarReadRegInfoInput.address, stVarReadRegInfoInput.memoryMap);
				}
				else
				{
					PRINTF("Reg value of addr 0x%04X of MMS %d is 0x%08X\n", stVarReadRegInfoInput.address, stVarReadRegInfoInput.memoryMap, stVarReadRegData.databuffer[0]);
				}
				regReadResponseBuff[regReadResponse_SendBuffIndex++] = stVarReadRegInfoInput.memoryMap;
				regReadResponseBuff[regReadResponse_SendBuffIndex++] = (uint8_t)stVarReadRegInfoInput.address;
				regReadResponseBuff[regReadResponse_SendBuffIndex++] = (uint8_t)((stVarReadRegInfoInput.address >> 8) & (0xFF));
				memcpy(&regReadResponseBuff[regReadResponse_SendBuffIndex], &stVarReadRegData.databuffer[0], EACH_REG_SIZE);
				regReadResponse_SendBuffIndex += EACH_REG_SIZE;
			}
			else if (typeOfConfig == CONFIGTYPE_REGWRITE)
			{
				uint32_t regWriteValue = 0;
				//PRINTF("Received Register write command from other Ethernet node \n");
				isResponseToBeSent = true;
				stVarWriteRegInput.memoryMap = receiveMACPHYBuff[receivedDataBuffIndex++];
				stVarWriteRegInput.length = 0;
				//regAddr = (uint16_t)receiveMACPHYBuff[receivedDataBuffIndex++];
				//stVarWriteRegInput.address = (uint16_t)((receiveMACPHYBuff[receivedDataBuffIndex++] << 8) | regAddr);
				stVarWriteRegInput.address =  ((receiveMACPHYBuff[receivedDataBuffIndex+1] << 8) | receiveMACPHYBuff[receivedDataBuffIndex]);
				receivedDataBuffIndex += sizeof(stVarWriteRegInput.address);
				regWriteValue = (uint32_t)((receiveMACPHYBuff[receivedDataBuffIndex+3] << 24) | (receiveMACPHYBuff[receivedDataBuffIndex+2] << 16)\
									| (receiveMACPHYBuff[receivedDataBuffIndex+1] << 8) | receiveMACPHYBuff[receivedDataBuffIndex]);
				receivedDataBuffIndex += EACH_REG_SIZE;

				stVarWriteRegInput.databuffer[0] = regWriteValue;

				executionStatus = T1S_WriteMACPHYReg(&stVarWriteRegInput);
				if (executionStatus == false)
				{
					PRINTF("Writing into reg failed during Node Configuration\n");
				}
				else
				{
					PRINTF("Register 0x%04X was written successfully with value 0x%08X through Node Configuration\n", stVarWriteRegInput.address, stVarWriteRegInput.databuffer[0]);
				}

				regReadResponseBuff[regReadResponse_SendBuffIndex++] = stVarWriteRegInput.memoryMap;
				regReadResponseBuff[regReadResponse_SendBuffIndex++] = (uint8_t)stVarWriteRegInput.address;
				regReadResponseBuff[regReadResponse_SendBuffIndex++] = (uint8_t)((stVarWriteRegInput.address >> 8) & (0xFF));
			}
			else
			{
				isResponseToBeSent = false;
				PRINTF("Invalid Type of config value received, received value is %d\n", typeOfConfig);
			}
		}
	}
	else if (messageType == MESSAGETYPE_RESPONSE)
	{
		isResponseToBeSent = false;
		for (uint8_t configCount = 0; configCount < numberOfConfigToDo; configCount++)
		{
			uint8_t mms;
			uint16_t regAddr;
			uint32_t regValue;
			uint8_t typeOfConfig = receiveMACPHYBuff[receivedDataBuffIndex++];

			if (typeOfConfig == CONFIGTYPE_REGREAD)
			{
				mms = receiveMACPHYBuff[receivedDataBuffIndex++];
				regAddr	= (uint16_t)((receiveMACPHYBuff[receivedDataBuffIndex+1] << 8) | receiveMACPHYBuff[receivedDataBuffIndex]);
				receivedDataBuffIndex += sizeof(regAddr);
				regValue = (uint32_t)((receiveMACPHYBuff[receivedDataBuffIndex+3] << 24) | (receiveMACPHYBuff[receivedDataBuffIndex+2] << 16)\
									| (receiveMACPHYBuff[receivedDataBuffIndex+1] << 8) | receiveMACPHYBuff[receivedDataBuffIndex]);
				receivedDataBuffIndex += EACH_REG_SIZE;
				PRINTF("Reg value requested through Node Configuration for reg addr 0x%04X of MMS %d is 0x%08X\n", regAddr, mms, regValue);
			}
			else if (typeOfConfig == CONFIGTYPE_REGWRITE)
			{
				mms = receiveMACPHYBuff[receivedDataBuffIndex++];
				regAddr	= (uint16_t)((receiveMACPHYBuff[receivedDataBuffIndex+1] << 8) | receiveMACPHYBuff[receivedDataBuffIndex]);
				receivedDataBuffIndex += sizeof(regAddr);

				PRINTF("Reg 0x%04X written through Node Configuration for MMS %d was successful\n", regAddr, mms);
			}
			else
			{
				PRINTF("Invalid Type of configuration sent through Node configuration, Type is %d\n", typeOfConfig);
			}
		}
	}
	else
	{
		isResponseToBeSent = false;
		PRINTF("Invalid Message Type, received value is %d", messageType);
	}

	if ((regReadResponse_SendBuffIndex > ETHERNET_HEADER_SIZE) && (isResponseToBeSent == true))
	{
		if ((T1S_Transmit(&regReadResponseBuff[0], regReadResponse_SendBuffIndex)) != OK)
		{
			PRINTF("Sending data failed during Node Configuration response\n");
		}
		else
		{
			PRINTF("Sending bulk data was successful during Node Configuration response, num of sent bytes: %d\n", regReadResponse_SendBuffIndex);
		}
	}
}


void T1S_RemoteRegWrite(uint8_t *mac_addr, uint8_t mms, uint16_t reg_address, uint32_t regValueToWrite)
{
	uint8_t sendConfigFrameBuff[100] = {0};
	//bool executionStatus = true;
	uint16_t sendFrameIndex = 0;
	uint16_t etherTypeForNodeConfig = ETHERNET_TYPE_MACPHYCONFIG;

	sendFrameIndex = 0;
	memcpy(&sendConfigFrameBuff[sendFrameIndex], mac_addr, SIZE_OF_MAC_ADDR);
	sendFrameIndex += SIZE_OF_MAC_ADDR;
	memcpy(&sendConfigFrameBuff[sendFrameIndex], &g_currentNodeMACAddr[0], SIZE_OF_MAC_ADDR);
	sendFrameIndex += SIZE_OF_MAC_ADDR;

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)etherTypeForNodeConfig;
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(etherTypeForNodeConfig >> 8);

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)MESSAGETYPE_SEND;

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)1; // Number of Config

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)CONFIGTYPE_REGWRITE;  // Config Type

	sendConfigFrameBuff[sendFrameIndex++] = mms;

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(reg_address);  // Reg Addr value
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(reg_address >> 8);  // Reg Addr value

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(regValueToWrite);  // Reg Value to write
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(regValueToWrite >> 8);  // Reg Value to write
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(regValueToWrite >> 16);  // Reg Value to write
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(regValueToWrite >> 24);  // Reg Value to write

	if (sendFrameIndex > ETHERNET_HEADER_SIZE)
	{
		//stBulkDataTransfer_t stVarBulkTransfer;

		//stVarBulkTransfer.totalBytesToTransfer = sendFrameIndex;
		/*for (uint8_t transmitCount = 0; transmitCount < sendFrameIndex; transmitCount++)
		{
			PRINTF("0x%02X ", sendConfigFrameBuff[transmitCount]);
		}*/
		//PRINTF(" \n  Sent Bytes were above \n");
		//executionStatus = T1S_TransmitBulkData(&sendConfigFrameBuff[0], &stVarBulkTransfer);

		if (T1S_Transmit(&sendConfigFrameBuff[0], sendFrameIndex) != OK)
		{
			PRINTF("Sending bulk data failed during Topology discovery\n");
		}
		else
		{
			PRINTF("Remote Write: Sending bulk data was successful during Topo disc, num of sent bytes: %d\n", sendFrameIndex);
		}
	}
	//return executionStatus;
}

void T1S_RemoteRegRead(uint8_t *mac_addr, uint8_t mms, uint16_t reg_address)
{
	uint8_t sendConfigFrameBuff[100] = {0};
	//bool executionStatus;
	uint16_t sendFrameIndex = 0;
	uint16_t etherTypeForNodeConfig = ETHERNET_TYPE_MACPHYCONFIG;

	sendFrameIndex = 0;
	memcpy(&sendConfigFrameBuff[sendFrameIndex], mac_addr, SIZE_OF_MAC_ADDR);
	sendFrameIndex += SIZE_OF_MAC_ADDR;
	memcpy(&sendConfigFrameBuff[sendFrameIndex], &g_currentNodeMACAddr[0], SIZE_OF_MAC_ADDR);
	sendFrameIndex += SIZE_OF_MAC_ADDR;

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)etherTypeForNodeConfig;
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(etherTypeForNodeConfig >> 8);

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)MESSAGETYPE_SEND;

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)1; // Number of Config

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)CONFIGTYPE_REGREAD;  // Config Type

	sendConfigFrameBuff[sendFrameIndex++] = mms;

	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(reg_address);  // Reg Addr value
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(reg_address >> 8);  // Reg Addr value
	PRINTF("Remote Read to Dest node MAC Address is %02X:%02X:%02X:%02X:%02X:%02X \n", sendConfigFrameBuff[0], sendConfigFrameBuff[1], sendConfigFrameBuff[2], sendConfigFrameBuff[3], sendConfigFrameBuff[4], sendConfigFrameBuff[5]);
	PRINTF("Remote Read to source node MAC Address is %02X:%02X:%02X:%02X:%02X:%02X \n", sendConfigFrameBuff[6], sendConfigFrameBuff[7], sendConfigFrameBuff[8], sendConfigFrameBuff[9], sendConfigFrameBuff[10], sendConfigFrameBuff[11]);
	PRINTF("Remote Read Reg Addr 0x%04X, Msg Type %d, MMS 0x%02X\n", ((uint16_t)((uint16_t)sendConfigFrameBuff[18]) | (uint16_t)sendConfigFrameBuff[19] << 8), sendConfigFrameBuff[14], sendConfigFrameBuff[17]);

	if (sendFrameIndex > ETHERNET_HEADER_SIZE)
	{
		//stBulkDataTransfer_t stVarBulkTransfer;

		//stVarBulkTransfer.totalBytesToTransfer = sendFrameIndex;
		/*for (uint8_t transmitCount = 0; transmitCount < sendFrameIndex; transmitCount++)
		{
			PRINTF("0x%02X ", sendConfigFrameBuff[transmitCount]);
		}*/
		//PRINTF(" \n  Sent Bytes were above \n");
		/*executionStatus = T1S_TransmitBulkData(&sendConfigFrameBuff[0], &stVarBulkTransfer);
		if (executionStatus == false)*/
		if (T1S_Transmit(&sendConfigFrameBuff[0], sendFrameIndex) != OK)
		{
			PRINTF("Sending bulk data failed during Topology discovery\n");
		}
		else
		{
			PRINTF("Remote Read: Sending bulk data was successful during Topo disc, num of sent bytes: %d\n", sendFrameIndex);
		}
	}
	//return executionStatus;
}

uint32_t T1S_SendRemoteConfiguration(uint8_t *targetMAC, stRemoteConfigurationCommand_t* commands, uint32_t numCommands)
{
	uint8_t sendConfigFrameBuff[200] = { 0 };
	uint16_t etherTypeForNodeConfig = ETHERNET_TYPE_MACPHYCONFIG;
	uint8_t sendFrameIndex = 0;

	memcpy(&sendConfigFrameBuff[sendFrameIndex], targetMAC, SIZE_OF_MAC_ADDR);
	sendFrameIndex += SIZE_OF_MAC_ADDR;
	memcpy(&sendConfigFrameBuff[sendFrameIndex], g_currentNodeMACAddr, SIZE_OF_MAC_ADDR);
	sendFrameIndex += SIZE_OF_MAC_ADDR;
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)etherTypeForNodeConfig;
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(etherTypeForNodeConfig >> 8);
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)MESSAGETYPE_SEND;
	//PRINTF("Total Bytes is %d, Step 3 passed\n", sendFrameIndex);
	sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)numCommands; // Number of Config
	for (uint8_t configLoaded = 0; configLoaded < numCommands; configLoaded++)
	{
		sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)commands[configLoaded].type;  // Config Type
		sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)commands[configLoaded].MMS;  // MMS value
		sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(commands[configLoaded].address);  // Reg Addr value
		sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(commands[configLoaded].address >> 8);  // Reg Addr value
		if (commands[configLoaded].type == 1)
		{
			sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(commands[configLoaded].data);  // Reg Value to write
			sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(commands[configLoaded].data >> 8);  // Reg Value to write
			sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(commands[configLoaded].data >> 16);  // Reg Value to write
			sendConfigFrameBuff[sendFrameIndex++] = (uint8_t)(commands[configLoaded].data >> 24);  // Reg Value to write
		}
	}
	//PRINTF("2. Reg Remote config, NumOfConfig %d, Config Type %d, MMS 0x%02X, Reg Addr  0x%04X\n", numCommands, configType, regAddress, regValueToWrite);
	PRINTF("Total Bytes to send over network is %d\n", sendFrameIndex);
	if (sendFrameIndex > ETHERNET_HEADER_SIZE)
	{

		if (T1S_Transmit(&sendConfigFrameBuff[0], sendFrameIndex) != OK)
		{
			PRINTF("Sending bulk data failed during Node Configuration through command line input\n");
			return UNKNOWN_ERROR;
		}
		else
		{
			PRINTF("Sending bulk data was successful during Node Configuration through command line input, num of sent bytes: %d\n", sendFrameIndex);
		}
	}
	return OK;
}

#endif
