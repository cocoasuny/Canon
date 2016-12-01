/**
 ******************************************************************************
 * @file    sensor_management.h
 * @author  Jason
 * @version V1.0.0
 * @date    2016-11-5
 * @brief   The header of sensor_management.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */
 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSOR_MANAGEMENT_H
#define __SENSOR_MANAGEMENT_H


/* Includes ------------------------------------------------------------------*/
#include "main.h"



/* extern functios declare ---------------------------------------------------*/
void sensor_management_task_handle(void *pvParameters);
extern void ble_send_environmental_data(void);


#endif /* __SENSOR_MANAGEMENT_H */


/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/





/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/

