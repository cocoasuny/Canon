/**
  ******************************************************************************
  * @file    bluenrg_sensor_service.c
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
#include "bluenrg_sensor_service.h"


/* accelerometer service and char UUID define */
#define COPY_ACC_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x02,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_FREE_FALL_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0xe2,0x3e,0x78,0xa0, 0xcf,0x4a, 0x11,0xe1, 0x8f,0xfc, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x34,0x0a,0x1b,0x80, 0xcf,0x4b, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* variables ---------------------------------------------------------*/
uint16_t accServHandle, freeFallCharHandle, accCharHandle;


/*******************************************************************************
* Function Name  : Add_Acc_Service
* Description    : 添加三轴传感器服务
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
tBleStatus Add_Acc_Service(void)
{
    tBleStatus ret;

    uint8_t uuid[16];

    /* add service */
    COPY_ACC_SERVICE_UUID(uuid);
    ret = aci_gatt_add_serv(UUID_TYPE_128, uuid,PRIMARY_SERVICE,7,&accServHandle);
    if(ret != BLE_STATUS_SUCCESS) goto fail;    

    /* add Free Fall characteristic */
    COPY_FREE_FALL_UUID(uuid);
    ret = aci_gatt_add_char(accServHandle, UUID_TYPE_128, uuid, 1,
                           CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 0, &freeFallCharHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;

    /* add accelerometer characteristic */
    COPY_ACC_UUID(uuid);  
    ret =  aci_gatt_add_char(accServHandle, UUID_TYPE_128, uuid, 6,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &accCharHandle);
    if (ret != BLE_STATUS_SUCCESS) goto fail;

    return BLE_STATUS_SUCCESS; 

    fail:
    #ifdef Debug_BlueNRF
        DLog("Error while adding ACC service.\n");
    #endif
    return BLE_STATUS_ERROR ;    
}
/*******************************************************************************
* Function Name  : BlueNRG_Update_Acc
* Description    : BlueNRG Update Acceleration Value
* Input          : Structure containing acceleration value in mg
* Output         : None
* Return         : None
*******************************************************************************/
tBleStatus BlueNRG_Update_Acc(AxesRaw_t *data)
{  
    tBleStatus ret;    
    uint8_t buff[6];

    STORE_LE_16(buff,data->AXIS_X);
    STORE_LE_16(buff+2,data->AXIS_Y);
    STORE_LE_16(buff+4,data->AXIS_Z);

    ret = aci_gatt_update_char_value(accServHandle, accCharHandle, 0, sizeof(buff), buff);

    if (ret != BLE_STATUS_SUCCESS)
    {
        App_Error_Check(ret);
    }
    return ret;	
}




