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

/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001

/* extern functios declare ---------------------------------------------------*/
void sensor_management_task_handle(void *pvParameters);
void ble_send_environmental_data(void);
void ble_send_motion_data(void);
void set_2G_accelerometer_fullScale(void);
void set_4G_accelerometer_fullScale(void);


#endif /* __SENSOR_MANAGEMENT_H */


/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/





/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/

