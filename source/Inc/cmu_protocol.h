/**
 ******************************************************************************
 * @file    cmu_protocol.h
 * @author  Jason
 * @version V1.0.0
 * @date    2016-12-14
 * @brief   The header of cmu_protocol.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */
 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CMU_PROTOCOL_H
#define __CMU_PROTOCOL_H


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*************** Don't Change the following defines *************/

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
#define W2ST_CONSOLE_MAX_CHAR_LEN 20

/* Define the symbol used for defining each termination string
used in Console service */
#define W2ST_CONSOLE_END_STRING "\0"

/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001

/* Feature mask for Sensor fusion short precision */
#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100

/* Feature mask for Accelerometer events */
#define FEATURE_MASK_ACC_EVENTS 0x00000400

/* Feature mask for LED */
#define FEATURE_MASK_LED 0x20000000

/* W2ST command for asking the calibration status */
#define W2ST_COMMAND_CAL_STATUS 0xFF
/* W2ST command for resetting the calibration */
#define W2ST_COMMAND_CAL_RESET  0x00
/* W2ST command for stopping the calibration process */
#define W2ST_COMMAND_CAL_STOP   0x01


/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1    )
/* LED status */
#define W2ST_CONNECT_LED           (1<<1 )
/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2 )
/* Quaternions */
#define W2ST_CONNECT_QUAT          (1<<3 )

/* Activity Recognition */
#ifdef OSX_BMS_MOTIONAR
  #define W2ST_CONNECT_AR          (1<<4 )
#endif /* OSX_BMS_MOTIONAR */

/* Carry Position Recognition */
#ifdef OSX_BMS_MOTIONCP
  #define W2ST_CONNECT_CP          (1<<5 )
#endif /* OSX_BMS_MOTIONCP */

/* Gesture Recognition */
#ifdef OSX_BMS_MOTIONGR
  #define W2ST_CONNECT_GR          (1<<6 )
#endif /* OSX_BMS_MOTIONGR */

/* Pedometer SW */
#ifdef OSX_BMS_MOTIONPM
  #define W2ST_CONNECT_PM          (1<<7 )
#endif /* OSX_BMS_MOTIONPM */

/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<8 )

/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<9 )

/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<10)

/* Gas Gouge Feature */
#define W2ST_CONNECT_GG_EVENT      (1<<11)


/* extern functios declare ---------------------------------------------------*/
void cmu_config_command_parsing(uint8_t * att_data, uint8_t data_length);
uint32_t debug_console_command_parsing(uint8_t * att_data, uint8_t data_length);








#endif /* __CMU_PROTOCOL_H */


/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/






