/**
 ******************************************************************************
 * @file    cmu_protocol.c
 * @author  Jason
 * @version V1.0.0
 * @date    2016-12-14
 * @brief   与App通讯协议解析
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "cmu_protocol.h"


/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void cmu_config_command_parsing(uint8_t * att_data, uint8_t data_length)
{
	uint32_t 				FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
	uint8_t 				Command = att_data[4];
	uint8_t 				Data    = att_data[5];
	const TickType_t 		xMaxBlockTime = pdMS_TO_TICKS(300); /* 设置最大等待时间为 300ms */
	SENSOR_MSG_T			sensorManageQueueMsgValue;

	#ifdef DEBUG_APP_CONTROL
		uint8_t	i=0;
		printf("[App CMD]:");
		for(i=0;i<data_length;i++)
		{
			printf("0x%x,",att_data[i]);
		}
		printf("\r\n");
	#endif
			
	switch (FeatureMask)
	{
		case FEATURE_MASK_SENSORFUSION_SHORT:
		{
			/* Sensor Fusion */
			switch (Command)
			{
				case W2ST_COMMAND_CAL_STATUS:
				{
					#ifdef DEBUG_APP_CONTROL			
						printf("Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
					#endif 
					/* Replay with the calibration status for the feature */
					/* Control the calibration status */
					{
						unsigned char calStatus = osx_MotionFX_compass_isCalibrated();
						Calib_Notify(FeatureMask,Command,calStatus);
					}
				}
				break;
				case W2ST_COMMAND_CAL_RESET:
				{
					#ifdef DEBUG_APP_CONTROL
						printf("Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
					#endif /* OSX_BMS_DEBUG_CONNECTION */
					/* Reset the calibration */
					sensorManageQueueMsgValue.eventID = EVENT_SENSOR_CALIBRATION;
					xQueueSend(sensorManageEventQueue,(void *)&sensorManageQueueMsgValue,xMaxBlockTime);
				}
				break;
				case W2ST_COMMAND_CAL_STOP:
				{
					#ifdef DEBUG_APP_CONTROL
						printf("Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
					#endif 
					/* Do nothing in this case */
				}
				break;
				default:
				{
					#ifdef DEBUG_APP_CONTROL
						printf("Calibration UNKNOW Signal For Feature=%lx\n\r",FeatureMask);
					#endif
				}
				break;
			}
		}		
		break;
		case FEATURE_MASK_ACC_EVENTS:
		{
			/* Acc events */
			#ifdef DEBUG_APP_CONTROL
				printf("Conf Sig F=%lx C=%c D=%x\n\r",FeatureMask,Command,Data);
			#endif      
			switch(Command) 
			{
				case 'f':
				{
					/* FreeFall */
					switch(Data) 
					{
						case 1:
						{
//							EnableHWFreeFall();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
						case 0:
						{
//							DisableHWFreeFall();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
					}
				}
				break;
				case 'd':
				{
					/* Double Tap */
					switch(Data) 
					{
						case 1:
						{
//							EnableHWDoubleTap();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
						case 0:
						{
//							DisableHWDoubleTap();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
					}
				}
				break;
				case 's':
				{
					/* Single Tap */
					switch(Data) 
					{
						case 1:
						{
//							EnableHWSingleTap();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
						case 0:
						{
//							DisableHWSingleTap();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
					}
				}
				break;
				case 'p':
				{
					/* Pedometer */
					switch(Data) 
					{
						case 1:
						{
//							EnableHWPedometer();
//							ResetHWPedometer();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
						case 0:
						{
//							DisableHWPedometer();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
					}
				}
				break;
				case 'w':
				{
					/* Wake UP */
					switch(Data)
					{
						case 1:
						{
//							EnableHWWakeUp();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
						case 0:
						{
//							DisableHWWakeUp();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
					}
				}
				break;
				case 't':
				{
					/* Tilt */
					switch(Data) 
					{
						case 1:
						{
//							EnableHWTilt();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
						case 0:
						{
//							DisableHWTilt();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						}
						break;
					}
				}
				break;
				case 'o' :
				{
					/* Tilt */
					switch(Data) 
					{
						case 1:
//							EnableHWOrientation6D();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						break;
						case 0:
//							DisableHWOrientation6D();
//							Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
						break;
					}
				}
				break;
			}
		}	
		break;
		case FEATURE_MASK_LED:
		{
			/* Led events */
			#ifdef DEBUG_APP_CONTROL
				printf("Conf Sig F=%lx C=%2x\r\n",FeatureMask,Command);
			#endif 
			switch(Command) 
			{
				case 1:
//					TargetBoardFeatures.LedStatus=1;
//					LedOnTargetPlatform();
//					Config_Notify(FEATURE_MASK_LED,Command,Data);
				break;
				case 0:
//					TargetBoardFeatures.LedStatus=0;
//					LedOffTargetPlatform();
//					Config_Notify(FEATURE_MASK_LED,Command,Data);
				break;
			}
			/* Update the LED feature */
//			if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED)) 
//			{
//				LED_Update(TargetBoardFeatures.LedStatus);
//			}
		}
		break;
	}
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
uint32_t debug_console_command_parsing(uint8_t * att_data, uint8_t data_length)
{
	uint32_t 					SendBackData = 1;
	static uint32_t 			osxMotionLicStart=0; 
	int32_t 					BytesToWrite = 0;
	uint8_t 					BufferToWrite[256] = {0};	
  
	if(osxMotionLicStart==0)
	{
		/* No osxMotion License activation phase */
		if((att_data[0]=='?') & (att_data[1]=='?')) 
		{
			/* Print Legend */
			SendBackData=0;

			BytesToWrite =sprintf((char *)BufferToWrite,"Command:\r\n"
														"pr->HW pedometer reset\r\n"
														"info-> System Info\r\n"
														"powerstatus-> Battery Status [%% mV]\r\n"
														"versionFw-> FW Version\r\n"
														"versionBle-> Ble Version\r\n");
			Term_Update(BufferToWrite,BytesToWrite);
		} 
		else if((att_data[0]=='p') & (att_data[1]=='r'))
		{
			/* Reset the pedometer DS3 HW counter */
//			ResetHWPedometer();
			SendBackData=0;
		}
		#ifndef STM32_NUCLEO
		else if((att_data[0]=='f') & (att_data[1]=='w')) 
		{
			/* Jump to STM32 System Memory for firmware upgrade - Bootloader */
			BytesToWrite =sprintf((char *)BufferToWrite,"\r\nThe board restarts\r\n");
			Term_Update(BufferToWrite,BytesToWrite);
			BytesToWrite =sprintf((char *)BufferToWrite,"in Boot Loader mode\r\n");
//			Term_Update(BufferToWrite,BytesToWrite);      
//			#if defined (__IAR_SYSTEMS_ICC__)
//				RestartInBootLoaderMode = BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER;
//			#elif defined (__CC_ARM)
//				*RestartInBootLoaderMode = BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER;
//			#elif defined (__GNUC__)
//				RestartInBootLoaderMode = BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER;
//			#else
//				#error "Toolchain not supported"
//			#endif
//			HAL_NVIC_SystemReset();
		}
		#endif /* STM32_NUCLEO */
		else if(!strncmp("versionFw",(char *)(att_data),9))
		{
//			BytesToWrite =sprintf((char *)BufferToWrite,"%s_%s_%c.%c.%c\r\n",
//														"F401"
//														,OSX_BMS_PACKAGENAME,
//														OSX_BMS_VERSION_MAJOR,
//														OSX_BMS_VERSION_MINOR,
//														OSX_BMS_VERSION_PATCH);
//			Term_Update(BufferToWrite,BytesToWrite);
//			SendBackData=0;
		} 
		else if(!strncmp("powerstatus",(char *)(att_data),11))
		{
			SendBackData=0;
	
			uint32_t voltage, soc;
//			uint8_t v_mode;
			/* Update Gas Gouge Status */
//			BSP_GG_Task(TargetBoardFeatures.HandleGGComponent, &v_mode);

//			/* Read the Gas Gouge Status */
//			BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
//			BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);

			BytesToWrite =sprintf((char *)BufferToWrite,"Battery %ld%% %ld mV\r\n",soc,voltage);
			
			Term_Update(BufferToWrite,BytesToWrite);
		} 
		else if(!strncmp("info",(char *)(att_data),4)) 
		{
			SendBackData=0;

//			BytesToWrite =sprintf((char *)BufferToWrite,"\r\nSTMicroelectronics %s:\r\n"
//														"\tVersion %c.%c.%c\r\n"
//														"\tSTM32476RG-SensorTile board"
//														"\r\n",
//														OSX_BMS_PACKAGENAME,
//														OSX_BMS_VERSION_MAJOR,OSX_BMS_VERSION_MINOR,OSX_BMS_VERSION_PATCH);
			Term_Update(BufferToWrite,BytesToWrite);

			BytesToWrite =sprintf((char *)BufferToWrite,"\t(HAL %ld.%ld.%ld_%ld)\r\n"
														"\tCompiled %s %s"
														#if defined (__IAR_SYSTEMS_ICC__)
															" (IAR)\r\n",
														#elif defined (__CC_ARM)
															" (KEIL)\r\n",
														#elif defined (__GNUC__)
															" (openstm32)\r\n",
														#endif
														HAL_GetHalVersion() >>24,
														(HAL_GetHalVersion() >>16)&0xFF,
														(HAL_GetHalVersion() >> 8)&0xFF,
														HAL_GetHalVersion()      &0xFF,
														__DATE__,__TIME__);
			Term_Update(BufferToWrite,BytesToWrite);
		} 
		else if(!strncmp("upgradeFw",(char *)(att_data),9))
		{
//			uint32_t uwCRCValue;
//			uint8_t *PointerByte = (uint8_t*) &SizeOfUpdateBlueFW;

//			SizeOfUpdateBlueFW=atoi((char *)(att_data+9));
//			PointerByte[0]=att_data[ 9];
//			PointerByte[1]=att_data[10];
//			PointerByte[2]=att_data[11];
//			PointerByte[3]=att_data[12];

//			/* Check the Maximum Possible OTA size */
//			if(SizeOfUpdateBlueFW>OSX_BMS_MAX_PROG_SIZE)
//			{
//				OSX_BMS_PRINTF("OTA %s SIZE=%ld > %d Max Allowed\r\n",OSX_BMS_PACKAGENAME,SizeOfUpdateBlueFW, OSX_BMS_MAX_PROG_SIZE);
//				/* Answer with a wrong CRC value for signaling the problem to BlueMS application */
//				PointerByte[0]= att_data[13];
//				PointerByte[1]=(att_data[14]!=0) ? 0 : 1;/* In order to be sure to have a wrong CRC */
//				PointerByte[2]= att_data[15];
//				PointerByte[3]= att_data[16];
//				BytesToWrite = 4;
//				Term_Update(BufferToWrite,BytesToWrite);
//			} 
//			else
//			{
//				PointerByte = (uint8_t*) &uwCRCValue;
//				PointerByte[0]=att_data[13];
//				PointerByte[1]=att_data[14];
//				PointerByte[2]=att_data[15];
//				PointerByte[3]=att_data[16];

//				OSX_BMS_PRINTF("OTA %s SIZE=%ld uwCRCValue=%lx\r\n",OSX_BMS_PACKAGENAME,SizeOfUpdateBlueFW,uwCRCValue);

//				/* Reset the Flash */
//				StartUpdateFWBlueMS(SizeOfUpdateBlueFW,uwCRCValue);

//				/* Reduce the connection interval */
//				{
//					int ret = aci_l2cap_connection_parameter_update_request(connection_handle,
//									10 /* interval_min*/,
//									10 /* interval_max */,
//									0   /* slave_latency */,
//									400 /*timeout_multiplier*/);
//					/* Go to infinite loop if there is one error */
//					if (ret != BLE_STATUS_SUCCESS)
//					{
//						while (1) 
//						{
//							OSX_BMS_PRINTF("Problem Changing the connection interval\r\n");
//						}
//					}
//				}

//				/* Signal that we are ready sending back the CRC value*/
//				BufferToWrite[0] = PointerByte[0];
//				BufferToWrite[1] = PointerByte[1];
//				BufferToWrite[2] = PointerByte[2];
//				BufferToWrite[3] = PointerByte[3];
//				BytesToWrite = 4;
//				Term_Update(BufferToWrite,BytesToWrite);
//			}
//			SendBackData=0;
		} 
		else if(!strncmp("versionBle",(char *)(att_data),10))
		{
			uint8_t  hwVersion;
			uint16_t fwVersion;
			/* get the BlueNRG HW and FW versions */
			getBlueNRGVersion(&hwVersion, &fwVersion);
			BytesToWrite =sprintf((char *)BufferToWrite,"%s_%d.%d.%c\r\n",
														(hwVersion > 0x30) ? "BleMS" : "Ble",
														fwVersion>>8, 
														(fwVersion>>4)&0xF,
														(hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a');
			Term_Update(BufferToWrite,BytesToWrite);
			SendBackData=0;
		} 
		else if((att_data[0]=='u') & (att_data[1]=='i') & (att_data[2]=='d'))
		{
			/* Write back the STM32 UID */
			uint8_t *uid = (uint8_t *)STM32_UUID;
			uint32_t MCU_ID = STM32_MCU_ID[0]&0xFFF;
			BytesToWrite =sprintf((char *)BufferToWrite,"%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\r\n",
														uid[ 3],uid[ 2],uid[ 1],uid[ 0],
														uid[ 7],uid[ 6],uid[ 5],uid[ 4],
														uid[11],uid[ 10],uid[9],uid[8],
														MCU_ID);
			Term_Update(BufferToWrite,BytesToWrite);
			SendBackData=0;
		} 
		else if((att_data[0]=='l') & (att_data[1]=='i') & (att_data[2]=='c'))
		{
			/* Write back the osxMotion License status */
//			LicensesStatus();
			SendBackData=0;
		} 
		else if((att_data[0]=='X') & (att_data[1]=='X') & (att_data[2]=='0'))
		{
//			ResetLicensesStatus();
			BytesToWrite =sprintf((char *)BufferToWrite,"Reset of All the osxMotion licenses\r\n");
			Term_Update(BufferToWrite,BytesToWrite);
			BytesToWrite =sprintf((char *)BufferToWrite,"The Board will restart in 5 seconds\r\n");
			Term_Update(BufferToWrite,BytesToWrite);
			HAL_Delay(5000);
			HAL_NVIC_SystemReset();
			SendBackData=0;
		} 
		else
		{
			/* Try to understand if we want to Initializes one osxMotion License */
//			osxMotionLicStart = LicenseParsingConsoleCommand(att_data, data_length);
			if(osxMotionLicStart)
			{
				SendBackData=0;
			}
		}
	} 
	else
	{
		/* We need to Finalize one osxMotion License */
//		osxMotionLicStart = LicenseParsingConsoleCommand(att_data, data_length);
		SendBackData=0;
	}
	return SendBackData;
}


/************************ (C) COPYRIGHT Cocoasuny *****END OF FILE****/

