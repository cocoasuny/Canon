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


#endif /* __GLOBALTYPEDEF_H */

