/**
 ******************************************************************************
 * @file    bsp_hwAdvanceFeatures.h
 * @author  Jason
 * @version V1.0.0
 * @date    2017-1-9
 * @brief   The header of bsp_hwAdvanceFeatures.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */
 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_HWADVANCEFEATURES_H
#define __BSP_HWADVANCEFEATURES_H


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------- */
typedef enum
{
  ACC_NOT_USED     = 0x00,
  ACC_6D_OR_TOP    = 0x01,
  ACC_6D_OR_LEFT   = 0x02,
  ACC_6D_OR_BOTTOM = 0x03,
  ACC_6D_OR_RIGTH  = 0x04,
  ACC_6D_OR_UP     = 0x05,
  ACC_6D_OR_DOWN   = 0x06,
  ACC_TILT         = 0x08,
  ACC_FREE_FALL    = 0x10,
  ACC_SINGLE_TAP   = 0x20,
  ACC_DOUBLE_TAP   = 0x40,
  ACC_WAKE_UP      = 0x80
} AccEventType;

/* Imported Variables -------------------------------------------------------------*/
static float DefaultAccODR;

/* extern functios declare ---------------------------------------------------*/
void enable_hw_double_tap(void);
void disable_hw_double_tap(void);
void enable_hw_single_tap(void);
void disable_hw_single_tap(void);


#endif /* __BSP_HWADVANCEFEATURES_H */


/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/



