#ifndef T1S_OS_H
#define T1S_OS_H
/***************************************************************************//**
* @mainpage 10Base-T1S MACPHY - NCN26010
***************************************************************************//**
* @file 	T1S_OS.h
* @brief 	Defines function prototypes for operating system specific functions required by NCN26010.
* @author 	Kyle Storey, Tejashree Chaudhari.
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
#include <T1S_Hardware.h>
typedef void (*ISR_t)(uint32_t tick);

/*************************************************************************************************
 *   Error Codes
 ************************************************************************************************/
#define OK                               	(uint32_t) 0
#define OS_UNKNOWN_ERROR                 	(uint32_t) 1

/*************************************************************************************************
 *   Prototypes
 ************************************************************************************************/
/*!
    \brief Initialization Callback for OS Code. Called after \a SPI_Init.
	\return OK if successfully initalized, OS_UNKNOWN_ERROR otherwise.
*/
uint32_t OS_Init();
/*!
    \brief Yeilds CPU to other tasks until NCN26010 signals that data is available
	\return OK if returning after the data ready interrupt, OS_UNKNOWN_ERROR otherwise.
*/
uint32_t OS_WaitUntilDataReady();
/*!
    \brief Coordinates with hardware to call \a callback when the NCN26010 signals that data is available.
    Intended to allow the OS to observe these events.
    \param callback A function pointer to \a ISR_t to call when the interrupt fires.
	\return OK if the callback is successfully installed.
*/
uint32_t OS_SetDataReadyCallback(ISR_t callback);


/*!
    \brief Starts the OS Scheduler if an OS is avaialble. Otherwise begins an endless loop that shuttles messages to and from the TCP/IP Stack. 
*/
void OS_Start();


#endif /*T1S_OS_H*/
