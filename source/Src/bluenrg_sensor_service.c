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
/* UUID and Macros used for exporting the BLE Characteristics and Services */
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define STORE_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

#define STORE_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )

#define MCR_FAST_TERM_UPDATE_FOR_OTA(data) aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, 1 , data)

/* Hardware Characteristics Service */
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x04,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_LED_W2ST_CHAR_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x20,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_GG_W2ST_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x00,0x02,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Software Characteristics Service */
#define COPY_SW_SENS_W2ST_SERVICE_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x02,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#ifdef OSX_BMS_MOTIONAR
#define COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x10,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
#define COPY_CARRY_POSITION_REC_W2ST_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x08,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
#define COPY_GESTURE_REC_W2ST_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
#define COPY_ACC_PEDO_W2ST_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#endif /* OSX_BMS_MOTIONPM */

/* Console Service */
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TERM_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_STDERR_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Configuration Service */
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
	tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;

	if (breath > 0) 
	{
		breath--;
	} 
	else 
	{
		ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);

		if (ret != BLE_STATUS_SUCCESS)
		{
			breath = ACC_BLUENRG_CONGESTION_SKIP;
		}
	}

	return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */

/* variables ---------------------------------------------------------*/
static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t AccGyroMagCharHandle;
static uint16_t AccEventCharHandle;
static uint16_t LedCharHandle;
static uint16_t GGCharHandle;

static uint16_t SWServW2STHandle;
static uint16_t QuaternionsCharHandle;

#ifdef OSX_BMS_MOTIONAR
static uint16_t ActivityRecCharHandle;
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
static uint16_t CarryPosRecCharHandle;
#endif /* OSX_BMS_MOTIONCP */

#ifdef OSX_BMS_MOTIONGR
static uint16_t GestureRecCharHandle;
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
static uint16_t AccPedoCharHandle;
#endif /* OSX_BMS_MOTIONPM */

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

static uint8_t  EnvironmentalCharSize=2; /* Size for Environmental BLE characteristic */

static uint32_t SizeOfUpdateBlueFW=0;

static uint16_t connection_handle = 0;

/* private function */
static tBleStatus Term_Update_AfterRead(void);
static tBleStatus Stderr_Update_AfterRead(void);

/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
	tBleStatus ret;

	uint8_t uuid[16];

	COPY_CONFIG_SERVICE_UUID(uuid);
	ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}

	COPY_CONFIG_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
							CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 1, &ConfigCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}

	return BLE_STATUS_SUCCESS;

	fail:
		#ifdef Debug_BlueNRF
			printf("Error while adding Configuration service.\r\n");
		#endif
	return BLE_STATUS_ERROR;
}


/**
 * @brief  Add the Console service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConsoleW2ST_Service(void)
{
	tBleStatus ret;

	uint8_t uuid[16];

	COPY_CONSOLE_SERVICE_UUID(uuid);
	ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*2,&ConsoleW2STHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}

	COPY_TERM_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
							CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 1, &TermCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}

	COPY_STDERR_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
							CHAR_PROP_NOTIFY | CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 1, &StdErrCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}

	return BLE_STATUS_SUCCESS;

	fail:
	#ifdef Debug_BlueNRF
		printf("Error while adding Console service.\r\n");
	#endif
	return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
	tBleStatus ret;
	uint8_t Offset;
	uint8_t DataToSend;

	/* Split the code in packages*/
	for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN)
	{
		DataToSend = (length-Offset);
		DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

		/* keep a copy */
		memcpy(LastStderrBuffer,data+Offset,DataToSend);
		LastStderrLen = DataToSend;

		ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
		if (ret != BLE_STATUS_SUCCESS) 
		{
			return BLE_STATUS_ERROR;
		}
		HAL_Delay(10);
	}

	return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
	tBleStatus ret;
	uint8_t Offset;
	uint8_t DataToSend;

	/* Split the code in packages */
	for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN)
	{
		DataToSend = (length-Offset);
		DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

		/* keep a copy */
		memcpy(LastTermBuffer,data+Offset,DataToSend);
		LastTermLen = DataToSend;

		ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
		if (ret != BLE_STATUS_SUCCESS)
		{
			#ifdef Debug_BlueNRF
				printf("Error Updating Stdout Char\r\n");
			#endif
			return BLE_STATUS_ERROR;
		}
		HAL_Delay(20);
	}

	return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
	tBleStatus ret;

	ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
	if (ret != BLE_STATUS_SUCCESS)
	{
		return BLE_STATUS_ERROR;
	}

	return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
	tBleStatus ret;

	ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
	if (ret != BLE_STATUS_SUCCESS) 
	{
		#ifdef Debug_BlueNRF
			printf("Error Updating Stdout Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}

	return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the SW Feature service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_SWServW2ST_Service(void)
{
	tBleStatus ret;

	uint8_t uuid[16];

	uint8_t max_attr_records = 1 /* Sensor Fusion Short */;
	
	#ifdef OSX_BMS_MOTIONAR
		max_attr_records++;
	#endif /* OSX_BMS_MOTIONAR */

	#ifdef OSX_BMS_MOTIONCP
		max_attr_records++;
	#endif /* OSX_BMS_MOTIONCP */

	#ifdef OSX_BMS_MOTIONGR
		max_attr_records++;
	#endif /* OSX_BMS_MOTIONGR */

	#ifdef OSX_BMS_MOTIONPM
		max_attr_records++;
	#endif /* OSX_BMS_MOTIONPM */

	COPY_SW_SENS_W2ST_SERVICE_UUID(uuid);
	ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*max_attr_records,&SWServW2STHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}

	COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+6*SEND_N_QUATERNIONS,
							CHAR_PROP_NOTIFY,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &QuaternionsCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}

	#ifdef OSX_BMS_MOTIONAR
	COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
							CHAR_PROP_NOTIFY | CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &ActivityRecCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}
	#endif /* OSX_BMS_MOTIONAR */

	#ifdef OSX_BMS_MOTIONCP
	COPY_CARRY_POSITION_REC_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
							CHAR_PROP_NOTIFY | CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &CarryPosRecCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}
	#endif /* OSX_BMS_MOTIONCP */

	#ifdef OSX_BMS_MOTIONGR
	COPY_GESTURE_REC_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
							CHAR_PROP_NOTIFY | CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &GestureRecCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}
	#endif /* OSX_BMS_MOTIONGR */

	#ifdef OSX_BMS_MOTIONPM
	COPY_ACC_PEDO_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, uuid, 2+4+2,
							CHAR_PROP_NOTIFY|CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &AccPedoCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}
	#endif /* OSX_BMS_MOTIONPM */

	return BLE_STATUS_SUCCESS; 

	fail:
	#ifdef Debug_BlueNRF
		printf("Error while adding SW's Characteristcs service.\n");
	#endif
	return BLE_STATUS_ERROR;
}
/**
 * @brief  Update quaternions characteristic value
 * @param  SensorAxes_t *data Structure containing the quaterions
 * @retval tBleStatus      Status
 */
tBleStatus Quat_Update(SensorAxes_t *data)
{
	tBleStatus ret;    

	uint8_t buff[2+ 6*SEND_N_QUATERNIONS];

	STORE_LE_16(buff  ,(HAL_GetTick()>>3));

	#if SEND_N_QUATERNIONS == 1
		STORE_LE_16(buff+2,data[0].AXIS_X);
		STORE_LE_16(buff+4,data[0].AXIS_Y);
		STORE_LE_16(buff+6,data[0].AXIS_Z);
	#elif SEND_N_QUATERNIONS == 2
		STORE_LE_16(buff+2,data[0].AXIS_X);
		STORE_LE_16(buff+4,data[0].AXIS_Y);
		STORE_LE_16(buff+6,data[0].AXIS_Z);

		STORE_LE_16(buff+8 ,data[1].AXIS_X);
		STORE_LE_16(buff+10,data[1].AXIS_Y);
		STORE_LE_16(buff+12,data[1].AXIS_Z);
	#elif SEND_N_QUATERNIONS == 3
		STORE_LE_16(buff+2,data[0].AXIS_X);
		STORE_LE_16(buff+4,data[0].AXIS_Y);
		STORE_LE_16(buff+6,data[0].AXIS_Z);

		STORE_LE_16(buff+8 ,data[1].AXIS_X);
		STORE_LE_16(buff+10,data[1].AXIS_Y);
		STORE_LE_16(buff+12,data[1].AXIS_Z);

		STORE_LE_16(buff+14,data[2].AXIS_X);
		STORE_LE_16(buff+16,data[2].AXIS_Y);
		STORE_LE_16(buff+18,data[2].AXIS_Z);
	#else
		#error SEND_N_QUATERNIONS could be only 1,2,3
	#endif
	ret = ACI_GATT_UPDATE_CHAR_VALUE(SWServW2STHandle, QuaternionsCharHandle, 0, 2+6*SEND_N_QUATERNIONS, buff);

	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
		  printf("Error Updating Quat Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}
	return BLE_STATUS_SUCCESS;
}
#ifdef OSX_BMS_MOTIONAR
/**
 * @brief  Update Activity Recognition value
 * @param  osx_MAR_output_t ActivityCode Activity Recognized
 * @retval tBleStatus      Status
 */
//tBleStatus ActivityRec_Update(osx_MAR_output_t ActivityCode)
//{
//	tBleStatus ret;

//	uint8_t buff[2+ 1];

//	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
//	buff[2] = ActivityCode;

//	ret = aci_gatt_update_char_value(SWServW2STHandle, ActivityRecCharHandle, 0, 2+1, buff);

//	if (ret != BLE_STATUS_SUCCESS)
//	{
//		#ifdef Debug_BlueNRF
//			printf("Error Updating ActivityRec Char\r\n");
//		#endif
//		return BLE_STATUS_ERROR;
//	}
//	return BLE_STATUS_SUCCESS;
//}
#endif /* OSX_BMS_MOTIONAR */

#ifdef OSX_BMS_MOTIONCP
/**
 * @brief  Update Carry Postion Recognition value
 * @param  osx_MCP_output_t CarryPositionCode Carry Position Recognized
 * @retval tBleStatus      Status
 */
//tBleStatus CarryPosRec_Update(osx_MCP_output_t CarryPositionCode)
//{
//	tBleStatus ret;

//	uint8_t buff[2+ 1];

//	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
//	buff[2] = CarryPositionCode;

//	ret = aci_gatt_update_char_value(SWServW2STHandle, CarryPosRecCharHandle, 0, 2+1, buff);

//	if (ret != BLE_STATUS_SUCCESS)
//	{
//		#ifdef Debug_BlueNRF
//			printf("Error Updating CarryPosRec Char\r\n");
//		#endif
//		return BLE_STATUS_ERROR;
//	}
//	return BLE_STATUS_SUCCESS;
//}
#endif /* OSX_BMS_MOTIONCP */


#ifdef OSX_BMS_MOTIONGR
/**
 * @brief  Update Gesture Recognition value
 * @param  osx_MGR_output_t GestureCode Gesture Recognized
 * @retval tBleStatus      Status
 */
//tBleStatus GestureRec_Update(osx_MGR_output_t GestureCode)
//{
//	tBleStatus ret;

//	uint8_t buff[2+ 1];

//	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
//	buff[2] = GestureCode;

//	ret = aci_gatt_update_char_value(SWServW2STHandle, GestureRecCharHandle, 0, 2+1, buff);

//	if (ret != BLE_STATUS_SUCCESS)
//	{
//		#ifdef Debug_BlueNRF
//			printf("Error Updating Gesture Rec Char\r\n");
//		#endif
//		return BLE_STATUS_ERROR;
//	}
//	return BLE_STATUS_SUCCESS;
//}
#endif /* OSX_BMS_MOTIONGR */

#ifdef OSX_BMS_MOTIONPM
/**
 * @brief  Update Accelerometer Pedometer characteristics
 * @param  osxMPM_output *PM_Data Pedometer result
 * @retval tBleStatus Status
 */
//tBleStatus AccPedo_Update(osx_MPM_output_t *PM_Data)
//{  
//	tBleStatus ret;
//	uint8_t buff[2+4+2];
//	uint16_t Cadence = (uint16_t) PM_Data->Cadence;

//	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
//	STORE_LE_32(buff+2,PM_Data->Nsteps);
//	STORE_LE_16(buff+6,Cadence);

//	ret = aci_gatt_update_char_value(SWServW2STHandle, AccPedoCharHandle, 0, 2+4+2,buff);

//	if (ret != BLE_STATUS_SUCCESS)
//	{
//		#ifdef Debug_BlueNRF
//			printf("Error Updating AccPedo Char\r\n");
//		#endif
//		return BLE_STATUS_ERROR;
//	}
//	return BLE_STATUS_SUCCESS;
//}
#endif /* OSX_BMS_MOTIONPM */
/**
 * @brief  Send a notification when the Sensor Fusion Library changes the calibrations status
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t val Calibration value
 * @retval tBleStatus Status
 */
tBleStatus Calib_Notify(uint32_t Feature,uint8_t Command,uint8_t val)
{
	tBleStatus ret;
	uint8_t buff[2+4+1+1];

	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
	STORE_BE_32(buff+2,Feature);
	buff[6] = Command;
	buff[7] = (val==0x01) ? 100: val;

	ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("Error Updating Configuration Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}
	return BLE_STATUS_SUCCESS;
}

/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
	tBleStatus ret;
	uint8_t buff[2+4+1+1];

	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
	STORE_BE_32(buff+2,Feature);
	buff[6] = Command;
	buff[7] = data;

	ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("Error Updating Configuration Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}
	return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notify(uint16_t Command)
{
	tBleStatus ret;
	uint8_t buff[2+2];

	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
	STORE_LE_16(buff+2,Command);

	ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+2,buff);
	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("Error Updating AccEvent_Notify Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}
	return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
	tBleStatus ret;
	int32_t NumberChars = 4;

	uint8_t uuid[16];

	/* Battery Present */
	NumberChars++;

	COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
	ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
							1+3*NumberChars,
							&HWServW2STHandle);

	if (ret != BLE_STATUS_SUCCESS) 
	{
		goto fail;
	}

	/* Fill the Environmental BLE Characteristc */
	COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
	uuid[14] |= 0x04; /* One Temperature value*/
	EnvironmentalCharSize+=2;

	uuid[14] |= 0x08; /* Humidity */
	EnvironmentalCharSize+=2;

	uuid[14] |= 0x10; /* Pressure value*/
	EnvironmentalCharSize+=4;

	ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize,
							CHAR_PROP_NOTIFY|CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &EnvironmentalCharHandle);

	if (ret != BLE_STATUS_SUCCESS) 
	{
		goto fail;
	}

	COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3*3*2,
							CHAR_PROP_NOTIFY,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &AccGyroMagCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}

	COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2,
							CHAR_PROP_NOTIFY | CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &AccEventCharHandle);

	if (ret != BLE_STATUS_SUCCESS) 
	{
		goto fail;
	}

	COPY_LED_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
							CHAR_PROP_NOTIFY | CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &LedCharHandle);

	if (ret != BLE_STATUS_SUCCESS)
	{
		goto fail;
	}


	COPY_GG_W2ST_CHAR_UUID(uuid);
	ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2+2+2+1,
							CHAR_PROP_NOTIFY | CHAR_PROP_READ,
							ATTR_PERMISSION_NONE,
							GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
							16, 0, &GGCharHandle);

	if (ret != BLE_STATUS_SUCCESS) 
	{
		goto fail;
	}


	return BLE_STATUS_SUCCESS;

	fail:
	#ifdef Debug_BlueNRF
		printf("Error while adding HW's Characteristcs service.\n");
	#endif
	return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  SensorAxes_t Acc Structure containing acceleration value in mg
 * @param  SensorAxes_t Gyro Structure containing Gyroscope value
 * @param  SensorAxes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(SensorAxes_t *Acc,SensorAxes_t *Gyro,SensorAxes_t *Mag)
{  
	tBleStatus ret;
	int32_t AXIS_X;
	int32_t AXIS_Y;
	int32_t AXIS_Z;

	uint8_t buff[2+3*3*2];

	STORE_LE_16(buff,(HAL_GetTick()>>3));

	STORE_LE_16(buff+2 ,Acc->AXIS_X);
	STORE_LE_16(buff+4 ,Acc->AXIS_Y);
	STORE_LE_16(buff+6 ,Acc->AXIS_Z);

	Gyro->AXIS_X/=100;
	Gyro->AXIS_Y/=100;
	Gyro->AXIS_Z/=100;

	STORE_LE_16(buff+8 ,Gyro->AXIS_X);
	STORE_LE_16(buff+10,Gyro->AXIS_Y);
	STORE_LE_16(buff+12,Gyro->AXIS_Z);

	/* Apply Magneto calibration */
//	AXIS_X = Mag->AXIS_X - magOffset.magOffX;
//	AXIS_Y = Mag->AXIS_Y - magOffset.magOffY;
//	AXIS_Z = Mag->AXIS_Z - magOffset.magOffZ;

	STORE_LE_16(buff+14,AXIS_X);
	STORE_LE_16(buff+16,AXIS_Y);
	STORE_LE_16(buff+18,AXIS_Z);

	ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);

	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("Error Updating Acc/Gyro/Mag Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}
	return BLE_STATUS_SUCCESS;	
}
/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 */
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1)
{
	tBleStatus ret;
	uint8_t BuffPos;

	uint8_t buff[2+4/*Press*/+2/*Hum*/+2/*Temp2*/+2/*Temp1*/];

	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
	BuffPos=2;

	/* press */
	STORE_LE_32(buff+BuffPos,Press);
	BuffPos+=4;

	/* humidity */
	STORE_LE_16(buff+BuffPos,Hum);
	BuffPos+=2;

	/* temperature 1 */
	STORE_LE_16(buff+BuffPos,Temp1);
	BuffPos+=2;

	ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("Error Updating Environmental Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}
	return BLE_STATUS_SUCCESS;
}
/**
 * @brief  Update LEDs characteristic value
 * @param  uint8_t LedStatus LEDs status 0/1 (off/on)
 * @retval tBleStatus   Status
 */
tBleStatus LED_Update(uint8_t LedStatus)
{
	tBleStatus ret;

	uint8_t buff[2+1];

	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
	buff[2] = LedStatus;

	ret = aci_gatt_update_char_value(HWServW2STHandle, LedCharHandle, 0, 2+1,buff);

	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("Error Updating Temp Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}
	return BLE_STATUS_SUCCESS;
}
/**
 * @brief  Update Gas Gouge characteristic value
 * @param  None
 * @retval tBleStatus   Status
 */
tBleStatus GG_Update(void)
{
	tBleStatus ret;
	uint32_t voltage, soc;
	uint8_t v_mode;

	uint8_t buff[2+2+2+2+1];

	/* Update Gas Gouge Status */
//	BSP_GG_Task(TargetBoardFeatures.HandleGGComponent, &v_mode);

//	/* Read the Gas Gouge Status */
//	BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
//	BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);

	STORE_LE_16(buff  ,(HAL_GetTick()>>3));
	STORE_LE_16(buff+2,soc*10);
	STORE_LE_16(buff+4,voltage);
	STORE_LE_16(buff+6,-1); /* Because it's not possible measure the current */

	if(soc<15) {
	/* if it's < 15% Low Battery*/
	buff[8] = 0x00; /* Low Battery */
	} else {
	static uint32_t PreVoltage = 0;
	static uint32_t Status     = 0x04; /* Unknown */
	if(PreVoltage!=0) {
	if(PreVoltage>voltage) {
	Status = 0x01; /* Discharging */
	} else if(PreVoltage<voltage){
	Status = 0x03; /* Charging */
	}
	}
	buff[8] = Status;
	PreVoltage = voltage;
	}

	ret = aci_gatt_update_char_value(HWServW2STHandle, GGCharHandle, 0, 2+2+2+2+1,buff);

	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef Debug_BlueNRF
			printf("Error Updating GG Char\r\n");
		#endif
		return BLE_STATUS_ERROR;
	}
	return BLE_STATUS_SUCCESS;
}
