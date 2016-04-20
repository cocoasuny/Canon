/**
  ******************************************************************************
  * @file    bluenrg_interface.h
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLUENRG_INTERFACE_H_
#define __BLUENRG_INTERFACE_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef SpiHandle;


void Enable_SPI_IRQ(void);
void Disable_SPI_IRQ(void);
void Clear_SPI_IRQ(void);
void Clear_SPI_EXTI_Flag(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void BNRG_SPI_Init(void);
void BlueNRG_RST(void);
uint8_t BlueNRG_DataPresent(void);
void    BlueNRG_HW_Bootloader(void);
int32_t BlueNRG_SPI_Read_All(SPI_HandleTypeDef *hspi,
                             uint8_t *buffer,
                             uint8_t buff_size);
int32_t BlueNRG_SPI_Write(SPI_HandleTypeDef *hspi,
                          uint8_t* data1,
                          uint8_t* data2,
                          uint8_t Nb_bytes1,
                          uint8_t Nb_bytes2);
void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1,int32_t n_bytes2);                          
#ifdef OPTIMIZED_SPI
/* Optimized functions for throughput test */
/* Used by the server (L0 and F4, not L4) */
HAL_StatusTypeDef HAL_SPI_TransmitReceive_Opt(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_Opt(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_Opt(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint8_t Size);
#endif /* OPTIMIZED_SPI */


#endif //__BLUENRG_INTERFACE_H_

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/