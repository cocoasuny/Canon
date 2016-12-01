/**
 ******************************************************************************
 * @file    sensor_management.c
 * @author  Jason
 * @version V1.0.0
 * @date    2016-11-5
 * @brief   The management of sensors
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sensor_management.h"

/**
  * @brief  sensor management task handler
  * @param  None
  * @retval None
  */
void sensor_management_task_handle(void *pvParameters)
{   
    while(1)
    {
		ble_send_environmental_data();
        vTaskDelay(1000);
    }
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
void ble_send_environmental_data(void)
{
	float 				SensorValue = 0;
    int32_t 			PressToSend = 0;
    uint16_t 			HumToSend = 0;
    int16_t 			Temp2ToSend = 0,Temp1ToSend = 0;
    int32_t 			decPart = 0, intPart = 0;
	
	/* Pressure,Humidity, and Temperatures*/
	BSP_PRESSURE_GetPressure((float *)&SensorValue);
	MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
	PressToSend=intPart*100+decPart;
	
	BSP_HUM_TEMP_GetHumidity((float *)&SensorValue);
	MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
	HumToSend = intPart*10+decPart;

	BSP_HUM_TEMP_GetTemperature((float *)&SensorValue);
	MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
	Temp1ToSend = intPart*10+decPart; 
	
	Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
}






/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/

