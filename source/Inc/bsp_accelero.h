/**
 ******************************************************************************
 * @file    bsp_accelero.h
 * @author  Jason
 * @version V1.0.0
 * @date    2016-12-3
 * @brief   The header of bsp_accelero.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */
 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_ACCELERO_H
#define __BSP_ACCELERO_H


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "LSM6DS3_ACC_GYRO_driver_HL.h"

#define ACCELERO_SENSORS_MAX_NUM	2


/* type struct define */
typedef enum
{
	ACCELERO_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
	LSM6DS3_X_0,
	LSM303AGR_X_0	
} ACCELERO_ID_t;


/* extern functios declare ---------------------------------------------------*/
DrvStatusTypeDef BSP_ACCELERO_Init( ACCELERO_ID_t id,void **handle);
DrvStatusTypeDef BSP_ACCELERO_DeInit( void **handle );
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_ACCELERO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_ACCELERO_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_ACCELERO_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Axes( void *handle, SensorAxes_t *acceleration );
DrvStatusTypeDef BSP_ACCELERO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value );
DrvStatusTypeDef BSP_ACCELERO_Get_Sensitivity( void *handle, float *sensitivity );
DrvStatusTypeDef BSP_ACCELERO_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_ACCELERO_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_ACCELERO_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_ACCELERO_Get_FS( void *handle, float *fullScale );
DrvStatusTypeDef BSP_ACCELERO_Set_FS( void *handle, SensorFs_t fullScale );
DrvStatusTypeDef BSP_ACCELERO_Set_FS_Value( void *handle, float fullScale );
DrvStatusTypeDef BSP_ACCELERO_Get_Axes_Status( void *handle, uint8_t *xyz_enabled );
DrvStatusTypeDef BSP_ACCELERO_Set_Axes_Status( void *handle, uint8_t *enable_xyz );

DrvStatusTypeDef BSP_ACCELERO_Enable_Free_Fall_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Disable_Free_Fall_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Set_Free_Fall_Threshold_Ext( void *handle, LSM6DS3_ACC_GYRO_FF_THS_t thr );

DrvStatusTypeDef BSP_ACCELERO_Enable_Pedometer_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Disable_Pedometer_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Pedometer_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Get_Step_Count_Ext( void *handle, uint16_t *step_count );
DrvStatusTypeDef BSP_ACCELERO_Reset_Step_Counter_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Set_Pedometer_Threshold_Ext( void *handle, uint8_t thr );

DrvStatusTypeDef BSP_ACCELERO_Enable_Tilt_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Disable_Tilt_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Tilt_Detection_Status_Ext( void *handle, uint8_t *status );

DrvStatusTypeDef BSP_ACCELERO_Enable_Wake_Up_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Disable_Wake_Up_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Set_Wake_Up_Threshold_Ext( void *handle, uint8_t thr );

DrvStatusTypeDef BSP_ACCELERO_Enable_Single_Tap_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Disable_Single_Tap_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Enable_Double_Tap_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Disable_Double_Tap_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Threshold_Ext( void *handle, uint8_t thr );
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Shock_Time_Ext( void *handle, uint8_t time );
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Quiet_Time_Ext( void *handle, uint8_t time );
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Duration_Time_Ext( void *handle, uint8_t time );

DrvStatusTypeDef BSP_ACCELERO_Enable_6D_Orientation_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Disable_6D_Orientation_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XL_Ext( void *handle, uint8_t *xl );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XH_Ext( void *handle, uint8_t *xh );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YL_Ext( void *handle, uint8_t *yl );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YH_Ext( void *handle, uint8_t *yh );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZL_Ext( void *handle, uint8_t *zl );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZH_Ext( void *handle, uint8_t *zh );







#endif /* __BSP_ACCELERO_H */


/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/

