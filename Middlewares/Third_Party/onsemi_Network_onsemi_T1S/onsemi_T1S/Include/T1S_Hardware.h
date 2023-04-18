#ifndef T1S_HARDWARE_H
#define T1S_HARDWARE_H
/***************************************************************************//**
* @mainpage 10Base-T1S MACPHY - NCN26010
***************************************************************************//**
* @file 	T1S_Hardware.h
* @brief 	Defines function prototypes for hardware specific functions required by NCN26010.
* @author 	Arndt Schuebel, Kyle Storey.
* $Rev:	$
* $Date:	$
******************************************************************************
* @copyright (c) 2022 ON Semiconductor. All rights reserved.
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
typedef void (*ISR_t)(uint32_t tick);
typedef void (*ErrorCallback_t)(uint32_t error);

/*************************************************************************************************
 *   Error Codes
 ************************************************************************************************/
#define OK                               	(uint32_t) 0
#define SPI_ERROR                        	(uint32_t) 1
#define SPI_INIT_ERROR                   	(uint32_t) 2
#define ISR_INIT_ERROR                   	(uint32_t) 3
//TODO fill in other error codes

/*************************************************************************************************
 *   Prototypes
 ************************************************************************************************/
/*!
    \brief Initialization Callback for SPI Code. Must be called before the first \a SPI_Transfer.
	\return OK if successfully initalized, SPI_INIT_ERROR otherwise.
*/
uint32_t SPI_Init(); //Configures to send and recieve SPI data to the NCN26010. Return true on success
/*!
    \brief Transmits and Recieves Data with NCN26010 over a bidirectional SPI interface.
    \param rx_buffer A pointer to where received data should be placed.
    \param tx_buffer A pointer to where data to be trasfered is stored.
    \param num_bytes_to_transfer The number of bytes to send and receive over SPI
	\return OK if all data successfuly transfered. SPI_ERROR otherwise.
*/
uint32_t SPI_Transfer(uint8_t* rx_buffer, uint8_t* tx_buffer, uint16_t num_bytes_to_transfer); //transfer data over spi. Return true on success
/*!
    \brief Configures hardware to call \a callback when the NCN26010 asserts its interrupt.
    \param callback A function pointer to \a ISR_t to call when the interrupt fires.
	\return OK if the interrupt service routine is successfully installed.
*/
uint32_t SetDataReadyISR(ISR_t callback); //register a callback for when we get a data ready interrupt
/*!
    \brief Uses hardware timers to delay the requested number of microseconds.
    \param microseconds The number of microseconds to delay
	\return OK if returning after the requested number of microseconds.
*/
void DelayMicroseconds(uint32_t microseconds); //returns after the number of microseconds specified
/*!
    \brief Reads hardware timers to get the current time in milliseconds.
	\return The current time in milliseconds.
*/
uint32_t GetTick(); //returns current time in milliseconds
/*!
    \brief Frees SPI module and deinitializes.
	\return OK if successfully cleaned up.
*/
uint32_t SPI_Cleanup();
/*!
    \brief Put MCU in a low power mode that will wait until an interrupt.
	\return OK after the next interrupt if successful.
*/
uint32_t SleepUntilISR();


#endif /*T1S_HARDWARE_H*/
