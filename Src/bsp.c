/**
  ******************************************************************************
  * File Name          : bsp.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "main.h"


static uint8_t UsbSendData(uint8_t* pBuf, uint16_t nLen);


/*
*********************************************************************************************************
*	�� �� ��: Bsp_Init
*	����˵��: Bsp Init
*	��    �Σ�None
*	�� �� ֵ: None
*********************************************************************************************************/
void Bsp_Init(void)
{
    /* Initialize all configured peripherals */
    HAL_DeInit();
    MX_GPIO_Init();
    MX_SDIO_SD_Init();
    
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();

    /* init code for FATFS */
    MX_FATFS_Init();  
    
    HAL_Delay(5000);
    DLog("Start...\r\n");
}    




/**
  * @brief  Configures LED GPIO.
  * @param  Led: LED to be configured. 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  */
void BSP_LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable the GPIO_LED clock */
    __GPIOB_CLK_ENABLE();

    /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: LED to be set on 
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  */
void BSP_LED_On(void )
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off. 
  * @param  Led: LED to be set off
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  */
void BSP_LED_Off(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: LED to be toggled
  *          This parameter can be one of the following values:
  *            @arg  LED1
  *            @arg  LED2
  *            @arg  LED3
  *            @arg  LED4
  */
void BSP_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

/*
*********************************************************************************************************
*	�� �� ��: UsbSendData
*	����˵��: �����⴮�ڷ�������
*	��    �Σ�pBuf, ���ݻ����ַ
*			  nLen, ���ݳ��ȣ����ֽ�Ϊ��λ
*	�� �� ֵ:	=USBD_OK�����ͳɹ�
*				=USBD_BUSY��Txæ����Ҫ�ط�
*				=USBD_FAIL������ʧ��
*	ע    �ͣ�
*	��    �ߣ���������
*********************************************************************************************************
*/
static uint8_t UsbSendData(uint8_t* pBuf, uint16_t nLen)
{
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)pBuf, nLen);
	return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

/*
*********************************************************************************************************
*	�� �� ��: UsbPrintf
*	����˵��: �����⴮�ڴ�ӡ�ַ���, ���ϴ����ߺ󣬴�PC���ĳ����ն�������Թ۲������﷨��printf��ͬ��
*	��    �Σ�lpszFormat, ��ʽ�����ִ�
*			  ..., ��������
*	�� �� ֵ: ��
*	ע    �ͣ�
*	��    �ߣ���������
*********************************************************************************************************
*/
void DLog(const char* lpszFormat, ...)
{
	int nLen;
	char szBuffer[CMD_BUFFER_LEN+1];
	va_list args;
	va_start(args, lpszFormat);
	nLen = vsnprintf(szBuffer, CMD_BUFFER_LEN+1, lpszFormat, args);
	UsbSendData((uint8_t*)szBuffer, nLen);
	va_end(args);
}


