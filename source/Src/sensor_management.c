/**
 ******************************************************************************
 * @file    sensor_management.c
 * @author  Jason
 * @version V1.0.0
 * @date    2016-11-5
 * @brief   The management of sensors
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sensor_management.h"


#define OSXMOTIONFX_CHECK_CALIBRATION ((uint32_t)0x12345678)

/* private function declare --------------------------------------------------*/
static void ReCalibration(void);
static unsigned char RecallCalibrationFromMemory(void);
static unsigned char ResetCalibrationInMemory(void);
static void vSensorDataUpdateTimerCallback( TimerHandle_t pxTimer );

/* private variables declare -------------------------------------------------*/
static bool isCal = false;
static uint32_t CalibrationStructureRAM[8]={0};

/**
  * @brief  sensor management task handler
  * @param  None
  * @retval None
  */
void sensor_management_task_handle(void *pvParameters)
{   
	SENSOR_MSG_T			sensorManageQueueMsgValue;
	const TickType_t 		xMaxBlockTime = pdMS_TO_TICKS(100); /* 设置最大等待时间为 100ms */
	TimerHandle_t			xSensorDataUpdateTimer = NULL;      //用于控制sensor数据更新频率（更新至App）
	
	/* creat event queue for sensor management */
	sensorManageEventQueue = xQueueCreate(SENSOR_EVENT_QUEUE_SIZE,sizeof(SENSOR_MSG_T));
	#ifdef DEBUG_SENSOR_MANAGEMENT
		if(sensorManageEventQueue == NULL)
		{
			printf("sensor menagement queue creat fail\r\n");
		}
	#endif
		
	/* seneor manage event queue init */
	sensorManageQueueMsgValue.eventID = EVENT_SENSOR_DEFAULT;
	
	/* Enable Mems sensors */
	BSP_ACCELERO_Sensor_Enable(gMEMSHandler.HandleAccSensor);
	BSP_GYRO_Sensor_Enable(gMEMSHandler.HandleGyroSensor);
	BSP_MAGNETO_Sensor_Enable(gMEMSHandler.HandleMagSensor);

    /* Set Accelerometer Full Scale to 2G,and Read the Acc Sensitivity */
    set_2G_accelerometer_fullScale();
    
    
	/* osxMotinFX init */
	MotionFX_manager_init();
	MotionFX_manager_start_9X();
	
	/* Check if the calibration is already available in memory */
	RecallCalibrationFromMemory();

	/* creat a timer for sensor data update control */
	xSensorDataUpdateTimer = xTimerCreate("MEMSTim",SENSOR_DATA_UPDATE_TIMER_FREQ,pdTRUE,(void *)1,vSensorDataUpdateTimerCallback);
	if(xSensorDataUpdateTimer != NULL)
	{
		xTimerStart(xSensorDataUpdateTimer,xMaxBlockTime);
	}
		
    while(1)
    {
		//[code block]message event handle 
		{
			if(pdPASS == xQueueReceive(sensorManageEventQueue,(void *)&sensorManageQueueMsgValue,xMaxBlockTime))
			{
				switch(sensorManageQueueMsgValue.eventID)
				{
					case EVENT_SENSOR_CALIBRATION:
					{
						#ifdef DEBUG_SENSOR_MANAGEMENT
							printf("sensor calibration\r\n");
						#endif
						ReCalibration();
					}
					break;
					case EVENT_SENSOR_ENVIRONMENT_DATA_UPDATE:
					{
						#ifdef DEBUG_SENSOR_MANAGEMENT
							//printf("environment data update\r\n");
						#endif
						ble_send_environmental_data();
					}
					break;
					case EVENT_SENSOR_MOTION_DATA_UPDATA:
					{
						#ifdef DEBUG_SENSOR_MANAGEMENT
							//printf("motion data update\r\n");
						#endif		
						ble_send_motion_data();
					}
					break;
					default:break;
				}
			}
		}
		//[code block]end of message event handle 
		
    }
}

/**
  * @brief  call back for control the sensor data update frequence
  * @param  TimerHandle_t pxTimer
  * @retval None
  */
static void vSensorDataUpdateTimerCallback(TimerHandle_t pxTimer)
{
	SENSOR_MSG_T			sensorManageQueueMsgValue;
	const TickType_t 		xMaxBlockTime = pdMS_TO_TICKS(10); /* 设置最大等待时间为 10ms */
	uint16_t				envDataCntCtl = (1000 / ENVIRONMENTAL_DATA_UPDATE_FREQ) * SENSOR_DATA_UPDATE_TIMER_FREQ;
	uint16_t				motionDataCntCtl = (1000 / MOTION_DATA_UPDATE_FREQ) * SENSOR_DATA_UPDATE_TIMER_FREQ;
	static uint16_t			envDataCnt = 0;		//环境传感器数据上传频率计数
	static uint16_t			motionDataCnt = 0;	//运动原始数据上传频率计数	
	
	pxTimer = pxTimer;
	sensorManageQueueMsgValue.eventID = EVENT_SENSOR_DEFAULT;
	
	if(gDevInfo.bleStatus == CONNECT)
	{
		/* increase the conters */
		envDataCnt++;
		motionDataCnt++;
		
		if(envDataCnt >= envDataCntCtl)  //update the environment data
		{
			envDataCnt = 0;
			sensorManageQueueMsgValue.eventID = EVENT_SENSOR_ENVIRONMENT_DATA_UPDATE;
			xQueueSend(sensorManageEventQueue,(void *)&sensorManageQueueMsgValue,xMaxBlockTime);
		}
		
		if(motionDataCnt >= motionDataCntCtl) //update the motion data
		{
			motionDataCnt = 0;
			sensorManageQueueMsgValue.eventID = EVENT_SENSOR_MOTION_DATA_UPDATA;
			xQueueSend(sensorManageEventQueue,(void *)&sensorManageQueueMsgValue,xMaxBlockTime);		
		}
	}
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
void ble_send_environmental_data(void)
{
	float 				SensorValue = 0;
    int32_t 			PressToSend = 0;
    uint16_t 			HumToSend = 0;
    int16_t 			Temp2ToSend = 0,Temp1ToSend = 0;
    int32_t 			decPart = 0, intPart = 0;
	
	/* Pressure,Humidity, and Temperatures*/
	BSP_PRESSURE_GetPressure((float *)&SensorValue);
	MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
	PressToSend=intPart*100+decPart;
	
	BSP_HUM_TEMP_GetHumidity((float *)&SensorValue);
	MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
	HumToSend = intPart*10+decPart;

	BSP_HUM_TEMP_GetTemperature((float *)&SensorValue);
	MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
	Temp1ToSend = intPart*10+decPart; 
	
	Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
void ble_send_motion_data(void)
{
	SensorAxes_t ACC_Value;
	SensorAxes_t GYR_Value;
	SensorAxes_t MAG_Value;

	/* Read the Acc values */
	BSP_ACCELERO_Get_Axes(gMEMSHandler.HandleAccSensor,&ACC_Value);

	/* Read the Magneto values */
	BSP_MAGNETO_Get_Axes(gMEMSHandler.HandleMagSensor,&MAG_Value);

	/* Read the Gyro values */
	BSP_GYRO_Get_Axes(gMEMSHandler.HandleGyroSensor,&GYR_Value);
	
//	printf("\r\nAcc Data:%d,%d,%d\r\n",ACC_Value.AXIS_X,ACC_Value.AXIS_Y,ACC_Value.AXIS_Z);
//	printf("Gyro Data:%d,%d,%d\r\n",GYR_Value.AXIS_X,GYR_Value.AXIS_Y,GYR_Value.AXIS_Z);
//	printf("Mag Data:%d,%d,%d\r\n",MAG_Value.AXIS_X,MAG_Value.AXIS_Y,MAG_Value.AXIS_Z);

	AccGyroMag_Update(&ACC_Value,&GYR_Value,&MAG_Value);
}

/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
void set_2G_accelerometer_fullScale(void)
{
    float sensitivity = 0;
    
    /* Set Full Scale to +/-2g */
    BSP_ACCELERO_Set_FS_Value(gMEMSHandler.HandleAccSensor,2.0f);

    /* Read the Acc Sensitivity */
    BSP_ACCELERO_Get_Sensitivity(gMEMSHandler.HandleAccSensor,&sensitivity);
    sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
}

/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
void set_4G_accelerometer_fullScale(void)
{
    float sensitivity = 0;
    
    /* Set Full Scale to +/-4g */
    BSP_ACCELERO_Set_FS_Value(gMEMSHandler.HandleAccSensor,4.0f);

    /* Read the Acc Sensitivity */
    BSP_ACCELERO_Get_Sensitivity(gMEMSHandler.HandleAccSensor,&sensitivity);
    sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
}

/**
  * @brief  Reset the magneto calibration
  * @param  None
  * @retval None
  */
static void ReCalibration(void)
{	
	/* Reset the Compass Calibration */
	isCal=false;

	/* Notifications of Compass Calibration */
	Calib_Notify(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal);

	/* Reset the Calibration */
	osx_MotionFX_compass_forceReCalibration();
	#ifdef DEBUG_SENSOR_MANAGEMENT
		printf("Force ReCalibration\n\r");
	#endif

	ResetCalibrationInMemory();

	/* Reset Calibation offset */
	magOffset.magOffX = magOffset.magOffY= magOffset.magOffZ=0;
}

/**
 * @brief  Check if there are valid calibration values in memory and read them
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
static unsigned char RecallCalibrationFromMemory(void)
{
	/* ReLoad the Calibration Values from RAM */
	unsigned char Success = 1;

	if(CalibrationStructureRAM[0] == OSXMOTIONFX_CHECK_CALIBRATION)
	{
		magOffset.magOffX    = (signed short) CalibrationStructureRAM[1];
		magOffset.magOffY    = (signed short) CalibrationStructureRAM[2];
		magOffset.magOffZ    = (signed short) CalibrationStructureRAM[3];
		memcpy(&magOffset.magGainX, &(CalibrationStructureRAM[4]), sizeof(uint32_t));
		memcpy(&magOffset.magGainY, &(CalibrationStructureRAM[5]), sizeof(uint32_t));
		memcpy(&magOffset.magGainZ, &(CalibrationStructureRAM[6]), sizeof(uint32_t));
		memcpy(&magOffset.expMagVect, &(CalibrationStructureRAM[7]), sizeof(uint32_t));

		/* Set the Calibration Structure */
		osx_MotionFX_setCalibrationData(&magOffset);

		/* Control the calibration status */
		isCal = osx_MotionFX_compass_isCalibrated();
	}
	else
	{
		isCal = false;
	}
	return Success;
}

/**
 * @brief  Reset the Magnetometer calibration values in memory
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
static unsigned char ResetCalibrationInMemory(void)
{
	/* Reset Calibration Values in RAM */
	unsigned char Success = 1;
	int8_t Counter;

	for(Counter = 0; Counter < 8; Counter++)
	{
		CalibrationStructureRAM[Counter] = 0xFFFFFFFF;
	}

	return Success;
}



















/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/

