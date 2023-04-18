/*
 * T1S_STM32_Hardware.c
 *
 *  Created on: Apr 26, 2022
 *      Author: zbmgzd
 */

#include "T1S_Hardware.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_dma.h"

#define SPI1_NSS  GPIO_PIN_9
#define SPI1_SCK  GPIO_PIN_21
#define SPI1_MISO GPIO_PIN_22
#define SPI1_MOSI GPIO_PIN_23
#include <stdbool.h>

#define ENDIAN_SWAP 1
#define SPI_BUSY ((SPI1->SR & (1 << 7)) == 0x80)
#define DMA_BUSY (spi_dma_transfer.State == HAL_DMA_STATE_BUSY || spi_dma2_transfer.State == HAL_DMA_STATE_BUSY)

/*** LOCAL VARIABLES ***/
static DMA_HandleTypeDef spi_dma_transfer,spi_dma2_transfer;
static SPI_HandleTypeDef SPI_OA_initialisation;
static GPIO_InitTypeDef SPI_OA_GPIO_Config;




uint32_t SPI_Init() {

	/* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

	  /*Configure GPIO pin : PA9 */
	  SPI_OA_GPIO_Config.Pin = GPIO_PIN_9;
	  SPI_OA_GPIO_Config.Mode = GPIO_MODE_OUTPUT_PP;
	  SPI_OA_GPIO_Config.Pull = GPIO_NOPULL;
	  SPI_OA_GPIO_Config.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &SPI_OA_GPIO_Config);

		/* DMA interrupt init */
		/* DMA1_Channel4_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 3);

		/*DMA1 initialisation for SPI TX  */

		#if defined(DMAMUX1)
			// Configuration of DMA.MUX1
			__HAL_RCC_DMAMUX1_CLK_ENABLE();
			spi_dma_transfer.DMAmuxChannel = DMAMUX1_Channel3;
			spi_dma_transfer.DMAmuxChannel->CCR= 0x00D;
		#endif

		/* DMA controller clock enable */
		__HAL_RCC_DMA1_CLK_ENABLE();



		/* Configure DMA1 request on DMA1_Channel3 */
		spi_dma_transfer.Instance = DMA1_Channel3;
		/* Disable the peripheral */
		__HAL_DMA_DISABLE(&spi_dma_transfer);

		spi_dma_transfer.Init.Request = DMA_REQUEST_2;
		spi_dma_transfer.Init.Direction = DMA_MEMORY_TO_PERIPH;
		spi_dma_transfer.Init.PeriphInc = DMA_PINC_DISABLE;
		spi_dma_transfer.Init.MemInc = DMA_MINC_ENABLE; // DMA increment enable
		spi_dma_transfer.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		spi_dma_transfer.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		spi_dma_transfer.Init.Mode = DMA_CIRCULAR;
		spi_dma_transfer.Init.Priority = DMA_PRIORITY_HIGH;


		if (HAL_DMA_Init(&spi_dma_transfer) != HAL_OK)
		{
			return SPI_INIT_ERROR;
		}


		/*DMA2 initialisation for SPI RX  */
		#if defined(DMAMUX2)
		// Configuration of DMA.MUX2
		__HAL_RCC_DMAMUX1_CLK_ENABLE();
		spi_dma2_transfer.DMAmuxChannel = DMAMUX1_Channel11;
		spi_dma2_transfer.DMAmuxChannel->CCR= 0x00C; // request from SPI1RX
		#endif

		/* DMA controller clock enable */
		__HAL_RCC_DMA2_CLK_ENABLE();



		/* Configure DMA2 request on DMA2_Channel3 */
		spi_dma2_transfer.Instance = DMA2_Channel3;
		/* Disable the peripheral */
		__HAL_DMA_DISABLE(&spi_dma2_transfer);

		spi_dma2_transfer.Init.Request = DMA_REQUEST_1;
		spi_dma2_transfer.Init.Direction = DMA_PERIPH_TO_MEMORY;
		spi_dma2_transfer.Init.PeriphInc = DMA_PINC_DISABLE;
		spi_dma2_transfer.Init.MemInc = DMA_MINC_ENABLE; // increment enable
		spi_dma2_transfer.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		spi_dma2_transfer.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		spi_dma2_transfer.Init.Mode = DMA_CIRCULAR;
		spi_dma2_transfer.Init.Priority = DMA_PRIORITY_HIGH;
		if (HAL_DMA_Init(&spi_dma2_transfer) != HAL_OK)
		{
			return SPI_INIT_ERROR;
		}

		 SPI_OA_initialisation.Instance= SPI1;
		 SPI_OA_initialisation.Init.Mode= SPI_MODE_MASTER;
		 SPI_OA_initialisation.Init.Direction= SPI_DIRECTION_2LINES;
		 SPI_OA_initialisation.Init.DataSize= SPI_DATASIZE_8BIT;
		 SPI_OA_initialisation.Init.CLKPolarity= SPI_POLARITY_LOW;
		 SPI_OA_initialisation.Init.CLKPhase= SPI_PHASE_1EDGE;
		 SPI_OA_initialisation.Init.NSS= SPI_NSS_SOFT;
		 SPI_OA_initialisation.Init.BaudRatePrescaler= SPI_BAUDRATEPRESCALER_4;
		 SPI_OA_initialisation.Init.FirstBit= SPI_FIRSTBIT_MSB;
		 SPI_OA_initialisation.Init.TIMode= SPI_TIMODE_DISABLE;
		 SPI_OA_initialisation.Init.CRCCalculation= SPI_CRCCALCULATION_DISABLE;
		 SPI_OA_initialisation.Init.CRCPolynomial= 0x001A; // reset value
		 SPI_OA_initialisation.Init.CRCLength= SPI_CRC_LENGTH_16BIT;
		 SPI_OA_initialisation.Init.NSSPMode= SPI_CRC_LENGTH_16BIT;
		 if (HAL_SPI_Init(&SPI_OA_initialisation) != HAL_OK)
		 {
			return SPI_INIT_ERROR;
		 }




		 return OK;
}

uint32_t SPI_Transfer(uint8_t* rx_buffer, uint8_t* tx_buffer, uint16_t num_bytes_to_transfer) {
		while(SPI_BUSY);

		HAL_GPIO_WritePin(GPIOA, SPI1_NSS, GPIO_PIN_RESET);
		SPI1->CR2 = 0x0F07;

		/* enable the DMA and enable interrupt on DMA1 channel 3 */

		/* TODO FIND MORE ELEGENT SOLUTION TO ENDIAN PROBLEM */
		if (ENDIAN_SWAP) {
			uint8_t tmp;
			for (int i = 0; i+1 < num_bytes_to_transfer + 16; i+= 2) {
				if (i+1 < num_bytes_to_transfer){
					tmp = tx_buffer[i];
					tx_buffer[i] = tx_buffer[i+1];
					tx_buffer[i+1] = tmp;
				}
				else {
					tx_buffer[i] = 0xDE;
					tx_buffer[i+1] = 0xAD;
				}

			}
		}

		//while((SPI1->SR & (1 << 7)) == 0x80);  // wait while TXE flag is 0 (TX is not empty)

		__HAL_DMA_ENABLE(&spi_dma_transfer);
		HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

		HAL_SPI_TransmitReceive_DMA(&SPI_OA_initialisation, (uint8_t *)tx_buffer, (uint8_t *)rx_buffer, num_bytes_to_transfer) ;

		//while((SPI1->SR & (1 << 7)) == 0x80);  // wait while TXE flag is 0 (TX is not empty)
		//while(spi_dma_transfer.Instance->CNDTR  !=0);	 //pool until last clock from SPI_TX

		while(DMA_BUSY || SPI_BUSY);

		HAL_SPI_Abort(&SPI_OA_initialisation);

		// while (HAL_SPI_GetState(&SPI_OA_initialisation) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(GPIOA, SPI1_NSS, GPIO_PIN_SET);

		/* TODO FIND MORE ELEGENT SOLUTION TO ENDIAN PROBLEM */
		if (ENDIAN_SWAP) {
			uint8_t tmp;
			for (int i = 0; i+1 < num_bytes_to_transfer + 16; i+= 2) {
				if (i+1 < num_bytes_to_transfer){
					tmp = rx_buffer[i];
					rx_buffer[i] = rx_buffer[i+1];
					rx_buffer[i+1] = tmp;
				}
				else {
					rx_buffer[i] = 0xA5;
					rx_buffer[i+1] = 0x5A;
				}

			}
		}
		return OK;

}

static ISR_t dataReadyCallback = NULL;
uint32_t SetDataReadyISR(ISR_t callback) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 5);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	dataReadyCallback = callback;
	return OK;
}

// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_ResumeTick();
    if(dataReadyCallback != NULL && GPIO_Pin == GPIO_PIN_8) // If The INT Source Is EXTI Line8 (A8 Pin)
    {
    	dataReadyCallback(0);//note tick is not implemented
    }
}

uint32_t SPI_Cleanup() {
	return OK;
}

uint32_t GetTick() {
	return HAL_GetTick();
}

uint32_t SleepUntilISR() {
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	return OK;
}



