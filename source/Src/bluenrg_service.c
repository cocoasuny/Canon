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
#include <stdlib.h>
#include "bsp_hum_temp.h"
#include "bsp_pressure.h"


/* Ble parameters define */
uint8_t 			g_tx_power_level = 6;
uint16_t 			service_handle, dev_name_char_handle, appearance_char_handle;
volatile uint16_t 	connection_handle = 0;
static uint8_t 		bdaddr[6] = {0};

/* Private function prototypes -----------------------------------------------*/


/**
   * @brief BlueNRG初始化
   * @param  None
   * @retval None
   */
void BlueNRG_Init(void)
{
    /* Initialize the BlueNRG SPI driver */
    BNRG_SPI_Init();

    /* Initialize the BlueNRG HCI */
    HCI_Init();

    /* Reset BlueNRG hardware */
    BlueNRG_RST();
		    
    /* Advertising Init */
    Advertising_Init();

    /* Service Init */
    Service_Init();

    /* Start Advertise */
    Start_Advertise();
}

/**
   * @brief Advertise 初始化
   * @param None
   * @retval None
   */
void Advertising_Init(void)
{
#ifdef Debug_BlueNRF
    uint8_t  hwVersion;
    uint16_t fwVersion;
#endif
	int 			ret = 0;
	
    /* get the BlueNRG HW and FW versions */
    getBlueNRGVersion(&hwVersion, &fwVersion);
#ifdef Debug_BlueNRF
    printf("HWver %x, FWver %x\r\n", hwVersion, fwVersion);
#endif

	/* Reset BlueNRG again otherwise it will fail. */
	BlueNRG_RST();
	
	/* Create a Unique BLE MAC */
	{
		bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
		bdaddr[1] = (STM32_UUID[0]    )&0xFF;
		bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
		bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
		
		/* if IDB05A1 = Number between 100->199
		 * if IDB04A1 = Number between 0->99
		 * where Y == (OSX_BMS_VERSION_MAJOR + OSX_BMS_VERSION_MINOR)&0xF */
		bdaddr[4] = (((OSX_BMS_VERSION_MAJOR-48)*10) + (OSX_BMS_VERSION_MINOR-48)+100)&0xFF;
		bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
	}
	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
									CONFIG_DATA_PUBADDR_LEN,
									bdaddr);
#ifdef Debug_BlueNRF
	if(ret != 0)
	{
		printf("Setting Pubblic BD_ADDR failed\r\n");
	}
#endif		
}

/**
   * @brief Services初始化
   * @param  None
   * @retval None
   */
tBleStatus Service_Init(void)
{
    tBleStatus ret;

    /*gatt_Init*/
    ret = aci_gatt_init();

    if(ret != BLE_STATUS_SUCCESS)
    {
		#ifdef Debug_BlueNRF
			printf("GATT_Init failed...0x%x\r\n",ret);
		#endif			
        return ret;
    }

    ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    if(ret != BLE_STATUS_SUCCESS)
    {
		#ifdef Debug_BlueNRF
			printf("GAP_Init failed...0x%x\r\n",ret);
		#endif		
        return ret;
    }
	
	ret = hci_le_set_random_address(bdaddr);
    if(ret != BLE_STATUS_SUCCESS)
    {
		#ifdef Debug_BlueNRF
			printf("Setting the Static Random BD_ADDR failed:0x%x\r\n",ret);
		#endif		
        return ret;
    }
	
    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL,
                                       7,
                                       16,
                                       USE_FIXED_PIN_FOR_PAIRING,
                                       123456,
                                       BONDING);
    if (ret != BLE_STATUS_SUCCESS)
    {
		#ifdef Debug_BlueNRF
			printf("BLE Stack Initialized.0x%x\r\n",ret);
		#endif		
        return ret;
    }

	/**********  add  SERVICEs ***********/
	ret = Add_HWServW2ST_Service();
	if(ret != BLE_STATUS_SUCCESS) 
	{
		#ifdef Debug_BlueNRF
			printf("Error while adding HW Service W2ST:0x%x\r\n",ret);
		#endif
		return BLE_STATUS_ERROR;
	}

	ret = Add_SWServW2ST_Service();
	if(ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("Error while adding SW Service W2ST:0x%x\r\n",ret);
		#endif
		return BLE_STATUS_ERROR;
	}

	ret = Add_ConsoleW2ST_Service();
	if(ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("\r\nError while adding Console Service W2ST:0x%x\r\n",ret);
		#endif
		return BLE_STATUS_ERROR;
	}	
	
	ret = Add_ConfigW2ST_Service();
	if(ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("\r\nError while adding Config Service W2ST:0x%x\r\n",ret);
		#endif
		return BLE_STATUS_ERROR;
	}	
	
    return BLE_STATUS_SUCCESS;
}
/**
   * @brief 开始广播
   * @param None
   * @retval None
   */
tBleStatus Start_Advertise(void)
{
    tBleStatus 			ret;
    const char 			local_name[DeviceMaxName] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_BLUEMS};
	const char 			BoardName[8] = {NAME_BLUEMS,0};
	uint8_t 			manuf_data[26] = {
											2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
											8,0x09,NAME_BLUEMS, // Complete Name
											13,0xFF,0x01/*SKD version */,
											0x02,
											0x00, /* */
											0xE0, /* ACC+Gyro+Mag*/
											0x00, /*  */
											0x00, /*  */
											0x00, /* BLE MAC start */
											0x00,
											0x00,
											0x00,
											0x00,
											0x00, /* BLE MAC stop */
										  };
	/* BLE MAC */
	manuf_data[20] = bdaddr[5];
	manuf_data[21] = bdaddr[4];
	manuf_data[22] = bdaddr[3];
	manuf_data[23] = bdaddr[2];
	manuf_data[24] = bdaddr[1];
	manuf_data[25] = bdaddr[0];

	manuf_data[16] |= 0x20; /* Led */
	manuf_data[17] |= 0x02; /* Battery Present */										  
	manuf_data[17] |= 0x04; /* One Temperature value*/
	manuf_data[17] |= 0x08; /* Humidity */		
	manuf_data[17] |= 0x10; /* Pressure value*/	
    manuf_data[18] |= 0x04;  /* Accelerometer Events */
    manuf_data[18] |= 0x01; /* Sensor Fusion */
	manuf_data[19] |= 0x10; /* ActivityRec */
	manuf_data[19] |= 0x08;	/* CarryPosRec */
	manuf_data[19] |= 0x02; /* Gesture Rec */
	manuf_data[19] |= 0x01; /* Pedometer */
    
	ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   strlen(BoardName), (uint8_t *)BoardName);  
    
    /* Set output power level */
	ret = Ble_SetTx_Power(g_tx_power_level);
    
	/* disable scan response */
	hci_le_set_scan_resp_data(0,NULL);  
	ret = aci_gap_set_discoverable(ADV_IND, 0, 0, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);  
	/* Send Advertising data */
	aci_gap_update_adv_data(26, manuf_data);
	
    return ret;
}
/**
   * @brief Set the transmit power
   * @param level: the power
   * @retval None
   */
tBleStatus Ble_SetTx_Power(uint8_t level)
{
    tBleStatus ret;
    
    /* Set output power level */
    ret = aci_hal_set_tx_power_level(1,level);

    return ret;
}
/**
   * @brief set the advertise address
   * @param None 
   * @retval None
   */
tBleStatus Ble_AdvAddress_Set(void)
{ 
	uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
	tBleStatus ret = BLE_STATUS_ERROR;
		
	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
							  CONFIG_DATA_PUBADDR_LEN,
							  bdaddr);
#ifdef Debug_BlueNRF
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf("Device Address failed...0x%x\r\n",ret);
    }
#endif

    return ret;
}

/**
   * @brief handle the read request
   * @param handle: the handle
   * @retval None
   */
static void Read_Request_CB(uint16_t handle)
{
    //获取handle
	if(handle == EnvironmentalCharHandle + 1)
	{
		/* Read Request for Pressure,Humidity, and Temperatures*/
		float 			SensorValue;
		int32_t 		PressToSend=0;
		uint16_t 		HumToSend=0;
		int16_t 		Temp2ToSend=0,Temp1ToSend=0;
		int32_t 		decPart, intPart;

		/* pressure */
		BSP_PRESSURE_GetPressure((float *)&SensorValue);
		MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
		PressToSend=intPart*100+decPart;
				

		/* humidity */
		BSP_HUM_TEMP_GetHumidity((float *)&SensorValue);
		MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
		HumToSend = intPart*10+decPart;

		/* temperature */
		BSP_HUM_TEMP_GetTemperature((float *)&SensorValue);
		MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
		Temp1ToSend = intPart*10+decPart; 

		Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
	} 
	else if(handle == LedCharHandle + 1)
	{
		uint8_t ledStatus;
		BSP_LED_Status_Get(&ledStatus);
		/* Read Request for Led Status */
		LED_Update(ledStatus);
	} 
	else if(handle == AccEventCharHandle +1)
	{
		/* Read Request for Acc Pedometer DS3 */
		uint16_t StepCount;
//		StepCount = GetStepHWPedometer();
		
		AccEvent_Notify(StepCount);
	#ifdef OSX_BMS_MOTIONPM
	}
	else if(handle == AccPedoCharHandle + 1)
	{
		/* Read Request for osxMotionPM */
//		AccPedo_Update(&PM_DataOUT);
		#endif /* OSX_BMS_MOTIONPM */
	}
	else if (handle == StdErrCharHandle + 1)
	{
		/* Send again the last packet for StdError */
		Stderr_Update_AfterRead();
	} 
	else if (handle == TermCharHandle + 1) 
	{
		/* Send again the last packet for Terminal */
		Term_Update_AfterRead();
		#ifdef OSX_BMS_MOTIONAR
	}
	else if(handle == ActivityRecCharHandle + 1)
	{
//		ActivityRec_Update(ActivityCode);
		#endif /* OSX_BMS_MOTIONAR */
		#ifdef OSX_BMS_MOTIONCP
	}
	else if(handle == CarryPosRecCharHandle + 1)
	{
//		CarryPosRec_Update(CarryPositionCode);
		#endif /* OSX_BMS_MOTIONCP */
		#ifdef OSX_BMS_MOTIONGR
	}
	else if(handle == GestureRecCharHandle + 1)
	{
//		GestureRec_Update(GestureRecognitionCode);
		#endif /* OSX_BMS_MOTIONGR */
		#ifdef STM32_SENSORTILE
	}
	else if(handle == GGCharHandle + 1)
	{
		GG_Update();
		#endif /* STM32_SENSORTILE */
	}

	//EXIT:
	if(connection_handle != 0)
	{
		aci_gatt_allow_read(connection_handle);
	}
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
	//  connected = TRUE;
	connection_handle = handle;

	printf("Connected to device:");
	for(int i = 5; i > 0; i--)
	{
		printf("%02X-", addr[i]);
	}
	printf("%02X\r\n", addr[0]);
}
/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None 
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
//  connected = FALSE;
	printf("Disconnected\r\n");
//  /* Make the device connectable again. */
//  set_connectable = TRUE;
//  notification_enabled = FALSE;
    Start_Advertise();
}
/**
 * @brief  This function is called attribute value corresponding to 
 *         ledButtonCharHandle characteristic gets modified.
 * @param  Handle of the attribute
 * @param  Size of the modified attribute data
 * @param  Pointer to the modified attribute data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data, uint8_t offset)
{
//	uint8_t i = 0;
	
	/* If GATT client has modified 'LED Control characteristic' value, toggle LED2 */
//	if(handle == ledControlCharHandle + 1)
//	{   
//		gLedFlashTime = att_data[0]*256 + att_data[1];
//		#ifdef Debug_LedControl
//			printf("remote control:%d,%d\r\n",data_length,gLedFlashTime);
//			for(i=0;i<data_length;i++)
//			{
//				printf("0x%x,",att_data[i]);
//			}
//			printf("\r\n");
//		#endif
//	}
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
	hci_uart_pckt *hci_pckt = pckt;
	
	/* obtain event packet */
	hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

	if(hci_pckt->type != HCI_EVENT_PKT)
	return;

	switch(event_pckt->evt)
	{
		case EVT_DISCONN_COMPLETE:
		{
			GAP_DisconnectionComplete_CB();
		}
		break;

		case EVT_LE_META_EVENT:
		{
			evt_le_meta_event *evt = (void *)event_pckt->data;

			switch(evt->subevent)
			{
				case EVT_LE_CONN_COMPLETE:
				{
					evt_le_connection_complete *cc = (void *)evt->data;
					GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
				}
				break;
			}
		}
		break;

		case EVT_VENDOR:
		{
			evt_blue_aci *blue_evt = (void*)event_pckt->data;
			switch(blue_evt->ecode)
			{
				case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:         
				{
					/* this callback is invoked when a GATT attribute is modified
					extract callback data and pass to suitable handler function */
					
					evt_gatt_attr_modified *evt = (evt_gatt_attr_modified*)blue_evt->data;
					Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data,evt->offset); 
				}
				break; 

				case EVT_BLUE_GATT_READ_PERMIT_REQ:
				{
					evt_gatt_read_permit_req *pr = (void*)blue_evt->data;                    
					Read_Request_CB(pr->attr_handle);                    
				}
				break;
			}
		}
		break;
	} 
}




