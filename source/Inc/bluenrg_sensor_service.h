/**
  ******************************************************************************
  * @file           : bluenrg_sensor_service.h
  * @version        : v1.0
  * @brief          : Header for bluenrg_sensor_service.c file.
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
#ifndef __BLUENRG_SENSOR_SERVICE_H
#define __BLUENRG_SENSOR_SERVICE_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"   
#include "hci.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"
#include "global_typedef.h"

#include <stdlib.h> 


/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION
/* For defining how many events skip when there is a congestion */
#define ACC_BLUENRG_CONGESTION_SKIP 30
#endif /* ACC_BLUENRG_CONGESTION */


/* For enabling MotionCP integration */
#define OSX_BMS_MOTIONCP

/* For enabling MotionAR integration */
#define OSX_BMS_MOTIONAR

/* For enabling MotionGR integration */
#define OSX_BMS_MOTIONGR

/* For enabling MotionPM integration */
#define OSX_BMS_MOTIONPM

/* Define The transmission interval in Multiple of 10ms for quaternions*/
#define QUAT_UPDATE_MUL_10MS 3

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
#define W2ST_CONSOLE_MAX_CHAR_LEN 20

/* Define the symbol used for defining each termination string
used in Console service */
#define W2ST_CONSOLE_END_STRING "\0"


/* function declare */
tBleStatus Add_HWServW2ST_Service(void);
tBleStatus AccGyroMag_Update(SensorAxes_t *Acc,SensorAxes_t *Gyro,SensorAxes_t *Mag);
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1);
tBleStatus LED_Update(uint8_t LedStatus);
tBleStatus GG_Update(void);
tBleStatus Add_SWServW2ST_Service(void);
tBleStatus Quat_Update(SensorAxes_t *data);
//tBleStatus ActivityRec_Update(osx_MAR_output_t ActivityCode);
//tBleStatus CarryPosRec_Update(osx_MCP_output_t CarryPositionCode);
//tBleStatus GestureRec_Update(osx_MGR_output_t GestureCode);
//tBleStatus AccPedo_Update(osx_MPM_output_t *PM_Data);
tBleStatus Calib_Notify(uint32_t Feature,uint8_t Command,uint8_t val);
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data);
tBleStatus AccEvent_Notify(uint16_t Command);
tBleStatus Add_ConsoleW2ST_Service(void);
tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
tBleStatus Term_Update(uint8_t *data,uint8_t length);
tBleStatus Add_ConfigW2ST_Service(void);

#endif /* __BLUENRG_SENSOR_SERVICE_H */

