/**
 ******************************************************************************
 * @file    bsp_hwAdvanceFeatures.c
 * @author  Jason
 * @version V1.0.0
 * @date    2017-1-9
 * @brief   基于硬件高级功能实现
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_hwAdvanceFeatures.h"


/**
  * @brief  This function enables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void enable_hw_double_tap(void)
{
    /* Disable all the HW features before */
//    DisableHWFeatures();
    
    BSP_ACCELERO_Get_ODR(gMEMSHandler.HandleAccSensor,&DefaultAccODR);

    /* Enable Double Tap detection */
    if(BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(gMEMSHandler.HandleAccSensor)==COMPONENT_ERROR)
    {
        #ifdef DEBUG_HW_ADVANCE_FEATURE
            printf("Error Enabling Double Tap Detection\r\n");
        #endif
    }
    #ifdef DEBUG_HW_ADVANCE_FEATURE
    else 
    {
        printf("Enabled Double Tap\r\n");
    }
    #endif

    if(BSP_ACCELERO_Set_Tap_Threshold_Ext(gMEMSHandler.HandleAccSensor,LSM6DS3_TAP_THRESHOLD_MID)==COMPONENT_ERROR)
    {
        #ifdef DEBUG_HW_ADVANCE_FEATURE
            printf("Error setting Double Tap Treshold\r\n");
        #endif
    }   
}

/**
  * @brief  This function disables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void disable_hw_double_tap(void)
{
    /* Disable Double Tap detection */
    if(BSP_ACCELERO_Disable_Double_Tap_Detection_Ext(gMEMSHandler.HandleAccSensor)==COMPONENT_ERROR)
    {
        #ifdef DEBUG_HW_ADVANCE_FEATURE
            printf("Error Disabling Double Tap Detection\r\n");
        #endif
    }
    #ifdef DEBUG_HW_ADVANCE_FEATURE    
    else 
    {
        printf("Disabled Double Tap\r\n");
    }
    #endif

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(gMEMSHandler.HandleAccSensor,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void enable_hw_single_tap(void)
{
    /* Disable all the HW features before */
//    DisableHWFeatures();

    /* Enable Single Tap detection */
    if(BSP_ACCELERO_Enable_Single_Tap_Detection_Ext(gMEMSHandler.HandleAccSensor)==COMPONENT_ERROR)
    {
        #ifdef DEBUG_HW_ADVANCE_FEATURE
            printf("Error Enabling Single Tap Detection\r\n");
        #endif
    } 
    #ifdef DEBUG_HW_ADVANCE_FEATURE
    else 
    {
        printf("Enabled Sigle Tap\r\n");
    }
    #endif
}
/**
  * @brief  This function disables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void disable_hw_single_tap(void)
{
    /* Disable Single Tap detection */
    if(BSP_ACCELERO_Disable_Single_Tap_Detection_Ext(gMEMSHandler.HandleAccSensor)==COMPONENT_ERROR)
    {
        #ifdef DEBUG_HW_ADVANCE_FEATURE
            printf("Error Disabling Single Tap Detection\r\n");
        #endif
    }
    #ifdef DEBUG_HW_ADVANCE_FEATURE
    else
    {
        printf("Disabled Sigle Tap\r\n");
    }
    #endif

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(gMEMSHandler.HandleAccSensor,DefaultAccODR);
}


