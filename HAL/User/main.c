/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2024-xx-xx
 * @brief   ���������Ѫ��
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ��С�� STM32F103C8 ���İ� 
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

#include "main.h"
#include "stm32f1xx.h"
#include <stdlib.h>
#include "usart/bsp_debug_usart.h"
#include "led/bsp_led.h" 
#include "i2c/bsp_i2c.h"
#include "max30102.h"
#include "max30102_fir.h"
/* =========================================����������====================================== */

void SystemClock_Config(void);

/* =========================================����������====================================== */
#define CACHE_NUMS 300//������
#define PPG_DATA_THRESHOLD 50000 	//�����ֵ
uint8_t max30102_int_flag=0;  		//�жϱ�־

float ppg_data_cache_RED[CACHE_NUMS]={0};  //������
float ppg_data_cache_IR[CACHE_NUMS]={0};  //������
I2C_HandleTypeDef I2C_Handle;

/**
 * @brief  ������
 * @param  ��
 * @retval ��
 */
int main(void)
{
    uint16_t cache_counter=0;  //���������
	float max30102_data[2],fir_output[2];
    
    HAL_Init();
	/* ��ʼ��ϵͳʱ��Ϊ72MHz */
    SystemClock_Config();
    /* ��ʼ��USART ����ģʽΪ 115200 8-N-1 */
    DEBUG_USART_Config();
	/* ��ʼ��LED */
	LED_GPIO_Config();
	printf("\r\n��ӭʹ��Ұ��С������Ѫ�����ģ�飡\r\n");
    printf("\r\n�뽫ָ������������Ѫ�����ģ���ϣ�ʹ����ȫ����ģ������");
    printf("\n���̵�����������ڶ�ȡ�У��Ե�Ƭ�̴��ڽ����ӡ����");
    printf("\n�뱣����ָ��ģ���ľ�ֹ״̬���Լ������\r\n");
    
    /* ��ʼ������Ѫ��ģ�� */
	max30102_init();
    /* FIR�˲������ʼ�� */
	max30102_fir_init();

	while(1)
	{
		
		if(max30102_int_flag)			//�ж��źŲ���
        { 
            max30102_int_flag = 0;
            max30102_fifo_read(max30102_data);		//��ȡ����
        
            ir_max30102_fir(&max30102_data[0],&fir_output[0]);//ʵ��ir���ݲɼ���ǰ�棬red�����ں���
            red_max30102_fir(&max30102_data[1],&fir_output[1]);  //�˲�
        
            if((max30102_data[0]>PPG_DATA_THRESHOLD)&&(max30102_data[1]>PPG_DATA_THRESHOLD))  //������ֵ��˵���������нӴ�
            {		
                ppg_data_cache_IR[cache_counter]=fir_output[0];
                ppg_data_cache_RED[cache_counter]=fir_output[1];
                cache_counter++;
                LED2_ON
            }
            else				//С����ֵ
            {
                cache_counter=0;
                LED2_OFF
            }


            if(cache_counter>=CACHE_NUMS)  //�ռ���������
            {
                uint16_t heart_rate = max30102_getHeartRate(ppg_data_cache_IR,CACHE_NUMS);
                float spo2 = max30102_getSpO2(ppg_data_cache_IR,ppg_data_cache_RED,CACHE_NUMS);

                printf("���ʣ�%d  ��/min   ", heart_rate);
                printf("Ѫ����%.2f  %%\n", spo2);

                cache_counter=0;
            }
            
        }
				
				
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==MAX30102_INT_Pin)
    {
        max30102_int_flag=1;
    }
}
/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000
 *            HCLK(Hz)                       = 72000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            HSE PREDIV1                    = 1
 *            PLLMUL                         = 9
 *            Flash Latency(WS)              = 2
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef clkinitstruct = {0};
    RCC_OscInitTypeDef oscinitstruct = {0};

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscinitstruct.HSEState = RCC_HSE_ON;
    oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    oscinitstruct.PLL.PLLState = RCC_PLL_ON;
    oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
    {
        /* Initialization Error */
        while (1)
            ;
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
    clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
    {
        /* Initialization Error */
        while (1)
            ;
    }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
