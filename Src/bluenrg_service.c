/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   Add a sample service using a vendor specific profile.
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
#include "bluenrg_service.h"
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
#include <stdlib.h>


/* Ble parameters define */
const char              *board_name = "CANNON V2";      //Device Name 
uint8_t                  tx_power_level = 7;            //Tx Power
uint16_t                 adv_interval = 100;            //Adv Interval
ble_gap_adv_params_t     m_adv_params;
static uint8_t adv_name[20], adv_name_len = 7,  local_name[20], local_name_len;
uint16_t service_handle, dev_name_char_handle, appearance_char_handle;   //默认服务


/* Private function prototypes -----------------------------------------------*/
static void ble_set_adv_param(char* adv_name, uint8_t*adv_address, uint8_t tx_power_pevel, uint16_t adv_interval);
static tBleStatus ble_address(uint8_t* advaddress);
static tBleStatus ble_device_set_name(const char* new_device_name);
static tBleStatus ble_set_tx_power(uint8_t level);
static void ble_device_set_advertising_interval(uint16_t interval);



/*
*********************************************************************************************************
*	函 数 名: BlueNRG_Init
*	功能说明: BlueNRG初始化
*	形    参：None
*	返 回 值: None
*********************************************************************************************************/
void BlueNRG_Init(void)
{
#ifdef Debug_BlueNRF    
    uint8_t  hwVersion;
    uint16_t fwVersion;
#endif
    
    /* Initialize the BlueNRG SPI driver */
    BNRG_SPI_Init();

    /* Initialize the BlueNRG HCI */
    HCI_Init();

    /* Reset BlueNRG hardware */
    BlueNRG_RST();
    
#ifdef Debug_BlueNRF
    /* get the BlueNRG HW and FW versions */
    getBlueNRGVersion(&hwVersion, &fwVersion);
    DLog("HWver %d, FWver %d\r\n", hwVersion, fwVersion);
#endif    
    
    /* Advertising Init */
    Advertising_Init();
    
    /* Service Init */
    
    /* Start Advertise */
    Ble_Device_Start_Advertising();    
}

/*******************************************************************************
* Function Name  : Advertising_Init
* Description    : 蓝牙广播数据初始化
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void Advertising_Init(void)
{
    uint8_t bdAddr[6];
    char name[32]; 
    
    HCI_get_bdAddr(bdAddr);
    sprintf(name, "%s %01x%01x", board_name, bdAddr[0], bdAddr[1]);
    
    /*Config Adv Parameter And Ready to Adv*/
    ble_set_adv_param(name, bdAddr, tx_power_level, adv_interval);
}

/*******************************************************************************
* Function Name  : Ble_Device_Start_Advertising
* Description    : 蓝牙开始广播
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
tBleStatus Ble_Device_Start_Advertising(void)
{
    tBleStatus ret;
    uint8_t uuid_length = 3;
    uint8_t serviceUUIDList[] = {AD_TYPE_16_BIT_SERV_UUID,0xFE,0x90};

    /* disable scan response */
    hci_le_set_scan_resp_data(0,NULL);
    ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                     adv_name_len, adv_name);

    /*min_adv_interval > 32*0.625*/
    ret = aci_gap_set_discoverable(ADV_IND, m_adv_params.interval, m_adv_params.interval, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                  local_name_len, (char*)local_name, uuid_length, serviceUUIDList, 0, 0);//// start advertising

    return ret;
}

/*******************************************************************************
* Function Name  : HCI_get_bdAddr
* Description    : 获取设备唯一ID
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void HCI_get_bdAddr(uint8_t *addr)
{
    /* Generate 48bit bdAddr from Unique device ID register (96 bits).
     * NOTE: Might be not unique! So change to other method if need.
     */

#define CHIP_UNIQUE_ID_REG	(0x1FFF7A10)	// move to a proper place?
    if (addr) {
        addr[0] = (*(__IO uint8_t *) CHIP_UNIQUE_ID_REG+5);
        addr[1] = (*(__IO uint8_t *) CHIP_UNIQUE_ID_REG+4);
        addr[2] = (*(__IO uint8_t *) CHIP_UNIQUE_ID_REG+3);
        addr[3] = (*(__IO uint8_t *) CHIP_UNIQUE_ID_REG+2);
        addr[4] = (*(__IO uint8_t *) CHIP_UNIQUE_ID_REG+1);
        addr[5] = (*(__IO uint8_t *) CHIP_UNIQUE_ID_REG+0);
    }
}
/*******************************************************************************
* Function Name  : ble_set_adv_param
* Description    : 设置蓝牙广播数据
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void ble_set_adv_param(char* adv_name, uint8_t*adv_address, uint8_t tx_power_pevel, uint16_t adv_interval)
{
    /*Set Adv Address*/
    ble_address(adv_address);
    
    /*Set Adv Name*/
    ble_device_set_name(adv_name);
        
    /*Set Tx Power Level*/
    ble_set_tx_power(tx_power_pevel);
    
    /* Range: 0x0020 to 0x4000
       Default: 1.28 s
       Time = AdvInterval * 0.625 msec
    */
    ble_device_set_advertising_interval(adv_interval);
}

/*******************************************************************************
* Function Name  : ble_address
* Description    : 设置蓝牙地址
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
tBleStatus ble_address(uint8_t* advaddress)
{
    tBleStatus ret;
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                    CONFIG_DATA_PUBADDR_LEN,
                                    advaddress);
    if(ret) 
    {
        return BLE_SET_BD_ADDR_FAILED;
    }
    
    return 0;
}
/*******************************************************************************
* Function Name  : ble_device_set_name
* Description    : 设置蓝牙设备名称
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
tBleStatus ble_device_set_name(const char* new_device_name)
{
    adv_name_len = strlen(new_device_name);
    local_name_len = adv_name_len+1;
    memcpy(adv_name,new_device_name,adv_name_len);
    local_name[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(local_name+1,new_device_name,adv_name_len);

    return BLE_STATUS_SUCCESS;
}
/*******************************************************************************
* Function Name  : ble_set_tx_power
* Description    : 设置蓝牙发送功率
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
tBleStatus ble_set_tx_power(uint8_t level)
{
    tBleStatus ret;
    /* Set output power level */
    ret = aci_hal_set_tx_power_level(1,level);

    return ret;
}
/*******************************************************************************
* Function Name  : ble_device_set_advertising_interval
* Description    : 设置蓝牙广播间隔
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void ble_device_set_advertising_interval(uint16_t interval)
{
    /*min_adv_interval > 32*0.625*/
    m_adv_params.interval = interval;
}
/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
    
}




