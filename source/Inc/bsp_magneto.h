/**
 ******************************************************************************
 * @file    bsp_gyro.h
 * @author  Jason
 * @version V1.0.0
 * @date    2016-12-5
 * @brief   The header of bsp_magneto.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */
 
 
/* Define to prevent recursive inclusion -------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_MAGNETO_H
#define __BSP_MAGNETO_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "LSM303AGR_MAG_driver_HL.h"
#include "main.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_MAGNETO Magnetometer
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_MAGNETO_Public_Types Public types
  * @{
  */

typedef enum
{
  MAGNETO_SENSORS_AUTO = -1,     /* Always first element and equal to -1 */
  LSM303AGR                      /* Default on board. */
} MAGNETO_ID_t;

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_MAGNETO_Public_Defines Public defines
  * @{
  */

#define MAGNETO_SENSORS_MAX_NUM 1

/**
 * @}
 */


/** @addtogroup X_NUCLEO_IKS01A1_MAGNETO_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_MAGNETO_Init( MAGNETO_ID_t id, void **handle );
DrvStatusTypeDef BSP_MAGNETO_DeInit( void **handle );
DrvStatusTypeDef BSP_MAGNETO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_MAGNETO_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_MAGNETO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_MAGNETO_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_MAGNETO_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_MAGNETO_Get_Axes( void *handle, SensorAxes_t *magnetic_field );
DrvStatusTypeDef BSP_MAGNETO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value );
DrvStatusTypeDef BSP_MAGNETO_Get_Sensitivity( void *handle, float *sensitivity );
DrvStatusTypeDef BSP_MAGNETO_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_MAGNETO_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_MAGNETO_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_MAGNETO_Get_FS( void *handle, float *fullScale );
DrvStatusTypeDef BSP_MAGNETO_Set_FS( void *handle, SensorFs_t fullScale );
DrvStatusTypeDef BSP_MAGNETO_Set_FS_Value( void *handle, float fullScale );

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

#endif /* __BSP_MAGNETO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
