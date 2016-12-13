/**
  ******************************************************************************
  * @file    main.h
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
#ifndef __MAIN_H_
#define __MAIN_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "bsp.h"
#include "platform.h"
#include "bluenrg_interface.h"
#include "hci.h"
#include "bluenrg_utils.h"
#include "bluenrg_service.h"
#include "freertostask.h"
#include "bluenrg_sensor_service.h"
#include "global_typedef.h"
#include "shell.h"
#include "bsp_rtc_calendar.h"
#include "Log.h"
#include "sensor_management.h"
#include "bsp_hum_temp.h"
#include "bsp_pressure.h"
#include "bsp_accelero.h"
#include "bsp_gyro.h"
#include "bsp_magneto.h"
#include "osx_motion_fx.h"
#include "MotionFX_Manager.h"




/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

#define ON		(1)
#define OFF		(0)

extern volatile AxesRaw_t gAxesData;
extern uint16_t gLedFlashTime;
extern uint8_t  gRxBuffer[RXBUFFERSIZE];
extern char SDPath[4];  /* SD logical drive path */
extern FATFS SDFatFs;  /* File system object for SD card logical drive */
extern FIL  MyFile;     /* File object */
extern MEMS_HANDLE_t					gMEMSHandler;
extern osxMFX_calibFactor 				magOffset; 
extern float 							sensitivity_Mul;  /* Acc sensitivity multiply by FROM_MG_TO_G constant */

void MX_GPIO_Init(void);
void MX_SDIO_SD_Init(void);



#endif /* __MAIN_H_ */

