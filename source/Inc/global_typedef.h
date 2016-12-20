/**
  ******************************************************************************
  * @file           : global_typedef.h
  * @version        : v1.0
  * @brief          : 
  ******************************************************************************
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
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
#ifndef __GLOBALTYPEDEF_H
#define __GLOBALTYPEDEF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"


/** 
 * @brief Structure containing acceleration value (in mg) of each axis.
 */
typedef struct {
  int AXIS_X;
  int AXIS_Y;
  int AXIS_Z;
} AxesRaw_t;
/**
 * @brief  Sensor axes raw data structure definition
 */
typedef struct
{
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
} SensorAxesRaw_t;

/**
 * @brief  Sensor axes data structure definition
 */
typedef struct
{
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} SensorAxes_t;



/**
 * @brief  Sensor output data rate enumerator definition
 */
typedef enum
{
  ODR_LOW,
  ODR_MID_LOW,
  ODR_MID,
  ODR_MID_HIGH,
  ODR_HIGH
} SensorOdr_t;



/**
 * @brief  Sensor full scale enumerator definition
 */
typedef enum
{
  FS_LOW,
  FS_MID,
  FS_HIGH
} SensorFs_t;


/**
 * @brief  Component's Status enumerator definition.
 */
typedef enum
{
  COMPONENT_OK = 0,
  COMPONENT_ERROR,
  COMPONENT_TIMEOUT,
  COMPONENT_NOT_IMPLEMENTED
} DrvStatusTypeDef;

/**
 * @brief  Component's Context structure definition.
 */
typedef struct
{

  /* Identity */
  uint8_t who_am_i;
  
  
  /* Configuration */
  uint8_t ifType;        /* 0 means I2C, 1 means SPI, etc. */
  uint8_t address;       /* Sensor I2C address (NOTE: Not a unique sensor ID). */
  uint8_t spiDevice;     /* Sensor Chip Select for SPI Bus */
  uint8_t instance;      /* Sensor instance (NOTE: Sensor ID unique only within its class). */
  uint8_t isInitialized; /* Sensor setup done. */
  uint8_t isEnabled;     /* Sensor ON. */
  uint8_t isCombo;       /* Combo sensor (component consists of more sensors). */
  
  /* Pointer to the Data */
  void *pData;
  
  /* Pointer to the Virtual Table */
  void *pVTable;
  /* Pointer to the Extended Virtual Table */
  void *pExtVTable;
} DrvContextTypeDef;

/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  void *HandleAccSensor;
  void *HandleGyroSensor;
  void *HandleMagSensor;
}MEMS_HANDLE_t;

/**
 * @brief  sensor management event structure definition
 */
typedef enum
{
	EVENT_SENSOR_DEFAULT=0,
	EVENT_SENSOR_CALIBRATION,
	EVENT_SENSOR_ENVIRONMENT_DATA_UPDATE,
	EVENT_SENSOR_MOTION_DATA_UPDATA,
	EVENT_SNESOR_MOTION_DATA_FUSION
}SENSOR_MANAGE_EVENT_ID_T;
/**
 * @brief  sensor management message structure definition
 */
typedef struct
{
	SENSOR_MANAGE_EVENT_ID_T		eventID;			
}SENSOR_MSG_T;

typedef enum
{
	CONNECT = 0,
	DISCONNECT
}BLE_STATUS;
/**
 * @brief  device information structure definition
 */
typedef struct
{
	BLE_STATUS bleStatus;
}DEVICE_STATUS_T;

#endif /* __GLOBALTYPEDEF_H */

