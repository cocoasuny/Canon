/**
 ******************************************************************************
 * @file    bsp_timer.c
 * @author  Jason
 * @version V1.0.0
 * @date    2016-11-5
 * @brief   The management of bsp timers
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_timer.h"


/**
  * @brief  the timer of sensor management init
  * @param  None
  * @retval None
  */
void bsp_sensor_management_timer_init(void)
{
	uint32_t uwPrescalerValue = 0;
	
	/*##-1- Configure the TIM peripheral #######################################*/ 
	/* -----------------------------------------------------------------------
		In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
		since APB1 prescaler is different from 1.   
		TIM3CLK = 2 * PCLK1  
		PCLK1 = HCLK / 2 
		=> TIM3CLK = HCLK = SystemCoreClock
		To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
		Prescaler = (TIM3CLK / TIM3 counter clock) - 1
		Prescaler = (SystemCoreClock /10 KHz) - 1

	Note: 
	SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
	variable value. Otherwise, any configuration based on this variable will be incorrect.
	This variable is updated in three ways:
	1) by calling CMSIS function SystemCoreClockUpdate()
	2) by calling HAL API function HAL_RCC_GetSysClockFreq()
	3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
	----------------------------------------------------------------------- */  

	/* Compute the prescaler value to have TIM3 counter clock equal to 1 KHz */
	uwPrescalerValue = (uint32_t) ((SystemCoreClock / SENSOR_DATA_UPDATE_TIMER_FREQ) - 1);

	/* Set TIMx instance */
	gSensorManagementTimHandle.Instance = SENSOR_MANAGEMENT_TIMER;

	/* Initialize TIM3 peripheral as follow:
	+ Period = 1000 - 1
	+ Prescaler = ((SystemCoreClock/2)/10000) - 1
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
	gSensorManagementTimHandle.Init.Period = SENSOR_DATA_UPDATE_TIMER_FREQ - 1;
	gSensorManagementTimHandle.Init.Prescaler = uwPrescalerValue;
	gSensorManagementTimHandle.Init.ClockDivision = 0;
	gSensorManagementTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&gSensorManagementTimHandle) != HAL_OK)
	{
		/* Initialization Error */
		#ifdef DEBUG_SENSOR_MANAGEMENT
			printf("sensor management timer init error\r\n");
		#endif
	}

	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
	/* Start Channel1 */
	if(HAL_TIM_Base_Start_IT(&gSensorManagementTimHandle) != HAL_OK)
	{
		/* Starting Error */
		#ifdef DEBUG_SENSOR_MANAGEMENT
			printf("sensor management timer start error\r\n");
		#endif
	}	
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t cnt = 0;
	static uint32_t	time = 0;

	cnt++;
	if(cnt >=10)
	{
		printf("timer:%d\r\n",HAL_GetTick() - time);
		cnt = 0;
		time = HAL_GetTick();
	}
}


/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/


