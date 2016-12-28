/**
  ******************************************************************************
  * @file    platform.h
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
#ifndef __PLATFORM_H_
#define __PLATFORM_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* Debug Switch */
#define PRINTFLOG
#define Debug_BlueNRF
//#define Debug_Sensor_Humidity
//#define Debug_Sensor_Press
//#define Debug_Sensor_Temperature
#define Debug_LedControl
#define DEBUG_OSXMOTIONFX
#define DEBUG_APP_CONTROL			//App控制命令调试
#define DEBUG_SENSOR_MANAGEMENT		//


/* Package Version only numbers 0->9 */
#define OSX_BMS_VERSION_MAJOR '3'
#define OSX_BMS_VERSION_MINOR '1'
#define OSX_BMS_VERSION_PATCH '1'

/* Define the BlueMicrosystem Name MUST be 7 char long */
#define NAME_BLUEMS 'B','M','1','V',OSX_BMS_VERSION_MAJOR,OSX_BMS_VERSION_MINOR,OSX_BMS_VERSION_PATCH
#define STM32_UUID ((uint32_t *)0x1FFF7590)
/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)


/* Shell Switch */
#define SHELL_ENABLE    //Shell使能
#define RTC_SHELL       //注释掉屏蔽RTC Shell功能
#define FATFS_SHELL

#define RXBUFFERSIZE          1

/* BLE Characteristic connection control */
#define W2ST_CONNECT_ENV           	(1    )		// Environmental Data 
#define W2ST_CONNECT_LED           	(1<<1 )		// LED status 
#define W2ST_CONNECT_ACC_GYRO_MAG  	(1<<2 )		// Acceleration/Gyroscope/Magneto 
#define W2ST_CONNECT_QUAT          	(1<<3 )		// Quaternions 
#define W2ST_CONNECT_AR            	(1<<4 )		// Activity Recognition 
#define W2ST_CONNECT_CP            	(1<<5 )     // Carry Position Recognition 
#define W2ST_CONNECT_GR          	(1<<6 )		// Gesture Recognition
#define W2ST_CONNECT_PM          	(1<<7 )		// Pedometer SW 
#define W2ST_CONNECT_STD_TERM      	(1<<8 )		// Standard Terminal 
#define W2ST_CONNECT_STD_ERR       	(1<<9 )		// Standard Error 
#define W2ST_CONNECT_ACC_EVENT     	(1<<10)		// HW Advance Features 
#define W2ST_CONNECT_GG_EVENT      	(1<<11)		// Gas Gouge Feature 

#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

/* sensor data management resource define */
#define SENSOR_MANAGEMENT_TIMER							TIM3
#define SENSOR_MANAGEMENT_TIM_CLK_ENABLE()				__HAL_RCC_TIM3_CLK_ENABLE();
#define	SENSOR_MANAGEMENT_TIM_IRQn						TIM3_IRQn
#define	SENSOR_MANAGEMENT_TIM_IRQHandler				TIM3_IRQHandler
#define SEMSOR_MANAGEMENT_PREPTY						0x0F
#define SEMSOR_MANAGEMENT_SUBPTY						0
#define SENSOR_DATA_UPDATE_TIMER_FREQ					(1000)  //最大1000/1 Hz输出频率
#define ENVIRONMENTAL_DATA_UPDATE_FREQ					(1)   	//Hz
#define MOTION_DATA_UPDATE_FREQ							(10)  	//Hz
#define MOTION_DATA_FUSION_FREQ							(100)  	//Hz



/******** Task define ********************/
/* BlueNRG HCI Process Task */
#define Task_BlueNRGHCI_Stack        500    //task stack
#define Task_BlueNRGHCI_Priority     2      //task priority

#define TASK_SENSOR_MANAGEMENT_STACK			1000 
#define TASK_SENSOR_MANAGEMENT_PRIORITY			4


/******* Message queue defien *************/
#define SENSOR_EVENT_QUEUE_SIZE	10

#ifdef PRINTFLOG
    #define Log  printf
#else
    #define Log  USBLog
#endif    




#define SYSCLK_FREQ 84000000


/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2



/**
* @brief SPI communication details between Nucleo F4 and BlueNRG
*        Expansion Board.
*/
// SPI Instance
#define BNRG_SPI_INSTANCE           SPI1
#define BNRG_SPI_CLK_ENABLE()       __SPI1_CLK_ENABLE()

// SPI Configuration
#define BNRG_SPI_MODE               SPI_MODE_MASTER
#define BNRG_SPI_DIRECTION          SPI_DIRECTION_2LINES
#define BNRG_SPI_DATASIZE           SPI_DATASIZE_8BIT
#define BNRG_SPI_CLKPOLARITY        SPI_POLARITY_LOW
#define BNRG_SPI_CLKPHASE           SPI_PHASE_1EDGE
#define BNRG_SPI_NSS                SPI_NSS_SOFT
#define BNRG_SPI_FIRSTBIT           SPI_FIRSTBIT_MSB
#define BNRG_SPI_TIMODE             SPI_TIMODE_DISABLED
#define BNRG_SPI_CRCPOLYNOMIAL      7
#define BNRG_SPI_BAUDRATEPRESCALER  SPI_BAUDRATEPRESCALER_4
#define BNRG_SPI_CRCCALCULATION     SPI_CRCCALCULATION_DISABLED

// SPI Reset Pin: PB.1
#define BNRG_SPI_RESET_PIN          GPIO_PIN_1
#define BNRG_SPI_RESET_MODE         GPIO_MODE_OUTPUT_PP
#define BNRG_SPI_RESET_PULL         GPIO_PULLUP
#define BNRG_SPI_RESET_SPEED        GPIO_SPEED_LOW
#define BNRG_SPI_RESET_ALTERNATE    0
#define BNRG_SPI_RESET_PORT         GPIOB
#define BNRG_SPI_RESET_CLK_ENABLE() __GPIOB_CLK_ENABLE()

// SCLK: PA.5
#define BNRG_SPI_SCLK_PIN           GPIO_PIN_5
#define BNRG_SPI_SCLK_MODE          GPIO_MODE_AF_PP
#define BNRG_SPI_SCLK_PULL          GPIO_PULLDOWN
#define BNRG_SPI_SCLK_SPEED         GPIO_SPEED_HIGH
#define BNRG_SPI_SCLK_ALTERNATE     GPIO_AF5_SPI1
#define BNRG_SPI_SCLK_PORT          GPIOA
#define BNRG_SPI_SCLK_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

// MISO (Master Input Slave Output): PA.6
#define BNRG_SPI_MISO_PIN           GPIO_PIN_6
#define BNRG_SPI_MISO_MODE          GPIO_MODE_AF_PP
#define BNRG_SPI_MISO_PULL          GPIO_NOPULL
#define BNRG_SPI_MISO_SPEED         GPIO_SPEED_HIGH
#define BNRG_SPI_MISO_ALTERNATE     GPIO_AF5_SPI1
#define BNRG_SPI_MISO_PORT          GPIOA
#define BNRG_SPI_MISO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

// MOSI (Master Output Slave Input): PA.7
#define BNRG_SPI_MOSI_PIN           GPIO_PIN_7
#define BNRG_SPI_MOSI_MODE          GPIO_MODE_AF_PP
#define BNRG_SPI_MOSI_PULL          GPIO_NOPULL
#define BNRG_SPI_MOSI_SPEED         GPIO_SPEED_HIGH
#define BNRG_SPI_MOSI_ALTERNATE     GPIO_AF5_SPI1
#define BNRG_SPI_MOSI_PORT          GPIOA
#define BNRG_SPI_MOSI_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

// NSS/CSN/CS: PA.4
#define BNRG_SPI_CS_PIN             GPIO_PIN_4
#define BNRG_SPI_CS_MODE            GPIO_MODE_OUTPUT_PP
#define BNRG_SPI_CS_PULL            GPIO_PULLUP
#define BNRG_SPI_CS_SPEED           GPIO_SPEED_HIGH
#define BNRG_SPI_CS_ALTERNATE       0
#define BNRG_SPI_CS_PORT            GPIOA
#define BNRG_SPI_CS_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

// IRQ: PC.4
#define BNRG_SPI_IRQ_PIN            GPIO_PIN_4
#define BNRG_SPI_IRQ_MODE           GPIO_MODE_IT_RISING
#define BNRG_SPI_IRQ_PULL           GPIO_NOPULL
#define BNRG_SPI_IRQ_SPEED          GPIO_SPEED_HIGH
#define BNRG_SPI_IRQ_ALTERNATE      0
#define BNRG_SPI_IRQ_PORT           GPIOC
#define BNRG_SPI_IRQ_CLK_ENABLE()   __GPIOC_CLK_ENABLE()

// EXTI External Interrupt for SPI
// NOTE: if you change the IRQ pin remember to implement a corresponding handler
// function like EXTI0_IRQHandler() in the user project
#define BNRG_SPI_EXTI_IRQn          EXTI4_IRQn
#define BNRG_SPI_EXTI_IRQHandler    EXTI4_IRQHandler
#define BNRG_SPI_EXTI_PIN           BNRG_SPI_IRQ_PIN
#define BNRG_SPI_EXTI_PORT          BNRG_SPI_IRQ_PORT
#define RTC_WAKEUP_IRQHandler       RTC_WKUP_IRQHandler

/* GPIO define for mems */
//LSM6DS3
#define MEMS_INT1_GPIO_PORT           GPIOC
#define MEMS_INT1_GPIO_CLK_ENABLE()   __GPIOC_CLK_ENABLE()
#define MEMS_INT1_GPIO_CLK_DISABLE()  __GPIOC_CLK_DISABLE()
#define MEMS_INT1_PIN                 GPIO_PIN_0

#define MEMS_INT1_EXTI_IRQn           EXTI0_IRQn


#endif /* __PLATFORM_H_ */


