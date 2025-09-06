/**
  ******************************************************************************
  * @file    bsp_i2c.c
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   Ӳ��I2C
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-MINI STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "./i2c/bsp_i2c.h"
#include "stm32f1xx.h"
#include "main.h"

/*
*********************************************************************************************************
*	�� �� ��: i2c_CfgGpio
*	����˵��: ����I2C���ߵ�GPIO������Ӳ��I2C�ķ�ʽʵ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void I2cMaster_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ʹ��I2Cxʱ�� */
	SENSORS_I2C_RCC_CLK_ENABLE();

	/* ʹ��I2C GPIO ʱ�� */
	SENSORS_I2C_SCL_GPIO_CLK_ENABLE();
	SENSORS_I2C_SDA_GPIO_CLK_ENABLE();

	/* ����I2Cx����: SCL ----------------------------------------*/
	GPIO_InitStructure.Pin =  SENSORS_I2C_SCL_GPIO_PIN; 
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull= GPIO_NOPULL;
//	GPIO_InitStructure.Alternate=SENSORS_I2C_AF; 
	HAL_GPIO_Init(SENSORS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	/* ����I2Cx����: SDA ----------------------------------------*/
	GPIO_InitStructure.Pin = SENSORS_I2C_SDA_GPIO_PIN;  
	HAL_GPIO_Init(SENSORS_I2C_SDA_GPIO_PORT, &GPIO_InitStructure); 
	
	if(HAL_I2C_GetState(&I2C_Handle) == HAL_I2C_STATE_RESET)
	{	
		/* ǿ�Ƹ�λI2C����ʱ�� */  
		SENSORS_I2C_FORCE_RESET(); 

		/* �ͷ�I2C����ʱ�Ӹ�λ */  
		SENSORS_I2C_RELEASE_RESET(); 
		
		/* I2C ���� */
        I2C_Handle.Instance = SENSORS_I2C;
        I2C_Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
        I2C_Handle.Init.ClockSpeed      = 400000;
        I2C_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        I2C_Handle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
        I2C_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        I2C_Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
        I2C_Handle.Init.OwnAddress1     = 0;
        I2C_Handle.Init.OwnAddress2     = 0;     
		/* ��ʼ��I2C */
		HAL_I2C_Init(&I2C_Handle);

    }

}

