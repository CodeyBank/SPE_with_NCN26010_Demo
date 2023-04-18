#ifndef FREERTOS_IP_CONFIG_H
#define FREERTOS_IP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
//#include "printf.h"
#define ipconfigUSE_IPv6 0
#define ipconfigUSE_TCP       1
#define ipconfigUSE_TCP_WIN       1

#define ipconfigTCP_TX_BUFFER_LENGTH   ( 10 * ipconfigTCP_MSS )
#define ipconfigTCP_RX_BUFFER_LENGTH   ( 10 * ipconfigTCP_MSS )

#define ipconfigNETWORK_MTU   1500
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES   1

#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS  10
#define ipconfigEVENT_QUEUE_LENGTH  15

#define ipconfigIP_TASK_PRIORITY			( configMAX_PRIORITIES - 2 )
#define ipconfigIP_TASK_STACK_SIZE_WORDS	( 4096 )
#define ipconfigUDP_MAX_SEND_BLOCK_TIME_TICKS ( 5000 / portTICK_PERIOD_MS )
#define ipconfigPACKET_FILLER_SIZE    4U

#define ipconfigUSE_DHCP    0

#define ipconfigZERO_COPY_RX_DRIVER 1
#define ipconfigZERO_COPY_TX_DRIVER 1

//#define ipconfigHAS_DEBUG_PRINTF	0
//#define FreeRTOS_debug_printf( MSG )		PRINTF MSG
#define ipconfigHAS_DEBUG_PRINTF 1
#define _Args(...) __VA_ARGS__
#define STRIP_PARENS(X) X
#define PASS_PARAMETERS(X) STRIP_PARENS( _Args X )
#define FreeRTOS_debug_printf(MSG) 0 //SEGGER_RTT_printf(0, PASS_PARAMETERS(MSG))
//#define FreeRTOS_printf( MSG )		PRINTF MSG
#define ipconfigHAS_PRINTF 1
#define FreeRTOS_printf(MSG) 0 //SEGGER_RTT_printf(0, PASS_PARAMETERS(MSG))

#define ipconfigSUPPORT_SELECT_FUNCTION 1 //seems to be needed for iperf

#ifdef __cplusplus
} /* extern "C" */
#endif



// iperf3 ---
#define USE_IPERF						        1
#define ipconfigIPERF_DOES_ECHO_UDP		        1

#define ipconfigIPERF_VERSION					3
#define ipconfigIPERF_STACK_SIZE_IPERF_TASK		680

#define ipconfigIPERF_TX_BUFSIZE				( 2 * ipconfigTCP_MSS )
#define ipconfigIPERF_TX_WINSIZE				( 2)
#define ipconfigIPERF_RX_BUFSIZE				( 2 * ipconfigTCP_MSS )
#define ipconfigIPERF_RX_WINSIZE				( 2 )

/* The iperf module declares a character buffer to store its send data. */
#define ipconfigIPERF_RECV_BUFFER_SIZE			( 4 * ipconfigTCP_MSS )

#endif /* FREERTOS_IP_CONFIG_H */
