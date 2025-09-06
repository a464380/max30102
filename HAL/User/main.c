/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2024-xx-xx
 * @brief   检测心率与血氧
 ******************************************************************************
 * @attention
 *
 * 实验平台:野火小智 STM32F103C8 核心板 
 * 论坛    :http://www.firebbs.cn
 * 淘宝    :https://fire-stm32.taobao.com
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
/* =========================================函数申明区====================================== */

void SystemClock_Config(void);

/* =========================================变量定义区====================================== */
#define CACHE_NUMS 300//缓存数
#define PPG_DATA_THRESHOLD 50000 	//检测阈值
uint8_t max30102_int_flag=0;  		//中断标志

float ppg_data_cache_RED[CACHE_NUMS]={0};  //缓存区
float ppg_data_cache_IR[CACHE_NUMS]={0};  //缓存区
I2C_HandleTypeDef I2C_Handle;

/**
 * @brief  主函数
 * @param  无
 * @retval 无
 */
int main(void)
{
    uint16_t cache_counter=0;  //缓存计数器
	float max30102_data[2],fir_output[2];
    
    HAL_Init();
	/* 初始化系统时钟为72MHz */
    SystemClock_Config();
    /* 初始化USART 配置模式为 115200 8-N-1 */
    DEBUG_USART_Config();
	/* 初始化LED */
	LED_GPIO_Config();
	printf("\r\n欢迎使用野火小智心率血氧检测模块！\r\n");
    printf("\r\n请将指尖轻贴在心率血氧检测模块上，使其完全覆盖模块检测区");
    printf("\n当绿灯亮则代表正在读取中，稍等片刻串口将会打印数据");
    printf("\n请保持手指与模块间的静止状态，以减少误差\r\n");
    
    /* 初始化心率血氧模块 */
	max30102_init();
    /* FIR滤波计算初始化 */
	max30102_fir_init();

	while(1)
	{
		
		if(max30102_int_flag)			//中断信号产生
        { 
            max30102_int_flag = 0;
            max30102_fifo_read(max30102_data);		//读取数据
        
            ir_max30102_fir(&max30102_data[0],&fir_output[0]);//实测ir数据采集在前面，red数据在后面
            red_max30102_fir(&max30102_data[1],&fir_output[1]);  //滤波
        
            if((max30102_data[0]>PPG_DATA_THRESHOLD)&&(max30102_data[1]>PPG_DATA_THRESHOLD))  //大于阈值，说明传感器有接触
            {		
                ppg_data_cache_IR[cache_counter]=fir_output[0];
                ppg_data_cache_RED[cache_counter]=fir_output[1];
                cache_counter++;
                LED2_ON
            }
            else				//小于阈值
            {
                cache_counter=0;
                LED2_OFF
            }


            if(cache_counter>=CACHE_NUMS)  //收集满了数据
            {
                uint16_t heart_rate = max30102_getHeartRate(ppg_data_cache_IR,CACHE_NUMS);
                float spo2 = max30102_getSpO2(ppg_data_cache_IR,ppg_data_cache_RED,CACHE_NUMS);

                printf("心率：%d  次/min   ", heart_rate);
                printf("血氧：%.2f  %%\n", spo2);

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
