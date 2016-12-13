/**
 ******************************************************************************
 * @file    bsp_gyro.h
 * @author  Jason
 * @version V1.0.0
 * @date    2016-12-5
 * @brief   The header of bsp_gyro.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */
 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_GYRO_H
#define __BSP_GYRO_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "LSM6DS3_ACC_GYRO_driver_HL.h"
#include "main.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_GYRO Gyro
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_GYRO_Public_Types Public types
  * @{
  */

typedef enum
{
  GYRO_SENSORS_AUTO = -1,        /* Always first element and equal to -1 */
  LSM6DS3_G_0                    /* DIL24 adapter. */
} GYRO_ID_t;

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_GYRO_Public_Defines Public defines
  * @{
  */

#define GYRO_SENSORS_MAX_NUM 2

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_GYRO_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_GYRO_Init( GYRO_ID_t id, void **handle );
DrvStatusTypeDef BSP_GYRO_DeInit( void **handle );
DrvStatusTypeDef BSP_GYRO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_GYRO_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_GYRO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_GYRO_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_GYRO_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_GYRO_Get_Axes( void *handle, SensorAxes_t *angular_velocity );
DrvStatusTypeDef BSP_GYRO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value );
DrvStatusTypeDef BSP_GYRO_Get_Sensitivity( void *handle, float *sensitivity );
DrvStatusTypeDef BSP_GYRO_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_GYRO_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_GYRO_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_GYRO_Get_FS( void *handle, float *fullScale );
DrvStatusTypeDef BSP_GYRO_Set_FS( void *handle, SensorFs_t fullScale );
DrvStatusTypeDef BSP_GYRO_Set_FS_Value( void *handle, float fullScale );
DrvStatusTypeDef BSP_GYRO_Get_Axes_Status( void *handle, uint8_t *xyz_enabled );
DrvStatusTypeDef BSP_GYRO_Set_Axes_Status( void *handle, uint8_t *enable_xyz );

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_GYRO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
