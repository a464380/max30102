#include "max30102.h"
#include "./i2c/bsp_i2c.h"


void max30102_i2c_write(uint8_t reg_adder,uint8_t data)
{
	uint8_t transmit_data[2];
	transmit_data[0] = reg_adder;
	transmit_data[1] = data;
	i2c_transmit(transmit_data,2);
}

void max30102_i2c_read(uint8_t reg_adder,uint8_t *pdata, uint8_t data_size)
{
    uint8_t adder = reg_adder;
    i2c_transmit(&adder,1);
    i2c_receive(pdata,data_size);
}

void max30102_int_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE(); 

    /*Configure GPIO pin : MAX30102_INT_Pin */
    GPIO_InitStruct.Pin = MAX30102_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MAX30102_INT_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

void max30102_init(void)
{ 
	uint8_t data;
	
	I2cMaster_Init();   //��ʼ��I2C�ӿ�
	delay_ms(500);
    
    max30102_int_gpio_init();   //�ж���������
    
	max30102_i2c_write(MODE_CONFIGURATION,0x40);  //reset the device
	
	delay_ms(5);
	
	max30102_i2c_write(INTERRUPT_ENABLE1,0xE0);
	max30102_i2c_write(INTERRUPT_ENABLE2,0x00);  //interrupt enable: FIFO almost full flag, new FIFO Data Ready,
																						 	//                   ambient light cancellation overflow, power ready flag, 
																							//						    		internal temperature ready flag
	
	max30102_i2c_write(FIFO_WR_POINTER,0x00);
	max30102_i2c_write(FIFO_OV_COUNTER,0x00);
	max30102_i2c_write(FIFO_RD_POINTER,0x00);   //clear the pointer
	
	max30102_i2c_write(FIFO_CONFIGURATION,0x4F); 
	
	max30102_i2c_write(MODE_CONFIGURATION,0x03); 
	
	max30102_i2c_write(SPO2_CONFIGURATION,0x2F); 
	
	max30102_i2c_write(LED1_PULSE_AMPLITUDE,0x2F);	//IR LED
	max30102_i2c_write(LED2_PULSE_AMPLITUDE,0x2F); //RED LED current
	
	max30102_i2c_write(TEMPERATURE_CONFIG,0x01);   //temp
	
	max30102_i2c_read(INTERRUPT_STATUS1,&data,1);
	max30102_i2c_read(INTERRUPT_STATUS2,&data,1);  //clear status
}


void max30102_fifo_read(float *output_data)
{
    uint8_t receive_data[6];
	uint32_t data[2];
	max30102_i2c_read(FIFO_DATA,receive_data,6);
    data[0] = ((receive_data[0]<<16 | receive_data[1]<<8 | receive_data[2]) & 0x03ffff);
    data[1] = ((receive_data[3]<<16 | receive_data[4]<<8 | receive_data[5]) & 0x03ffff);
	*output_data = data[0];
	*(output_data+1) = data[1];

}

#define MAX_HR_HISTORY 5
#define HR_MIN 40
#define HR_MAX 180
#define INIT_BUFFER_TIME 3
#define SAMPLERATE 100
#define MAX_INVALID_COUNT 5  // ������Ч����

static uint16_t hr_history[MAX_HR_HISTORY] = {0};
static uint8_t hr_index = 0;
static uint8_t hr_count = 0;

static uint8_t init_done = 0;
static uint16_t sample_counter = 0;
static uint8_t consecutive_invalid = 0;
static uint16_t last_valid_hr = 0; // ������һ����Ч����

uint16_t max30102_getHeartRate(float *input_data, uint16_t cache_nums)
{
    sample_counter += cache_nums;

    // 1. ��ʼ������
    if (!init_done) {
        if (sample_counter < INIT_BUFFER_TIME * SAMPLERATE) {
            return 0;
        } else {
            init_done = 1;
            sample_counter = 0;
        }
    }

    // 2. ��ֵ��ֵ
    float avg = 0;
    for (uint16_t i = 0; i < cache_nums; i++) avg += input_data[i];
    avg /= cache_nums;

    // 3. Ѱ�Ҷ����
    uint16_t peaks[10];
    uint8_t peak_count = 0;
    for (uint16_t i = 1; i < cache_nums - 1; i++) {
        if (input_data[i] > avg &&
            input_data[i] > input_data[i - 1] &&
            input_data[i] > input_data[i + 1]) {
            if (peak_count < 10) peaks[peak_count++] = i;
        }
    }

    if (peak_count < 2) {
        consecutive_invalid++;
        if (consecutive_invalid >= MAX_INVALID_COUNT) {
            last_valid_hr = 0;
            init_done = 0;
            consecutive_invalid = 0;
            return 0;
        }
        return last_valid_hr; // ����һ����Чֵ
    }

    // 4. ����ƽ������
    uint32_t interval_sum = 0;
    for (uint8_t i = 1; i < peak_count; i++) {
        interval_sum += (peaks[i] - peaks[i - 1]);
    }
    uint16_t avg_interval = interval_sum / (peak_count - 1);

    if (avg_interval == 0) return last_valid_hr;

    uint16_t hr = (uint16_t)((SAMPLERATE * 60.0f) / avg_interval);

    // 5. �쳣�޳�
    if (hr < HR_MIN || hr > HR_MAX) {
        consecutive_invalid++;
        if (consecutive_invalid >= MAX_INVALID_COUNT) {
            last_valid_hr = 0;
            init_done = 0;
            consecutive_invalid = 0;
            return 0;
        }
        return last_valid_hr; // �����ϴ���Ч
    }

    // 6. ƽ��
    hr_history[hr_index] = hr;
    hr_index = (hr_index + 1) % MAX_HR_HISTORY;
    if (hr_count < MAX_HR_HISTORY) hr_count++;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < hr_count; i++) sum += hr_history[i];
    uint16_t smoothed_hr = sum / hr_count;

    last_valid_hr = smoothed_hr;
    consecutive_invalid = 0;
    return smoothed_hr;
}




// uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums)
// {
// 		float input_data_sum_aver = 0;
// 		uint16_t i,temp;
		
		
// 		for(i=0;i<cache_nums;i++)
// 		{
// 		input_data_sum_aver += *(input_data+i);
// 		}
// 		input_data_sum_aver = input_data_sum_aver/cache_nums;
// 		for(i=0;i<cache_nums;i++)
// 		{
// 				if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
// 				{
// 					temp = i;
// 					break;
// 				}
// 		}
// 		i++;
// 		for(;i<cache_nums;i++)
// 		{
// 				if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
// 				{
// 					temp = i - temp;
// 					break;
// 				}
// 		}
// 		if((temp>10)&&(temp<300))
// 		{
//             return 6000 / temp;
// 		}
// 		else
// 		{
// 			return 0;
// 		}
// }


float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums)
{
    // float ir_max=*ir_input_data,ir_min=*ir_input_data;
    // float red_max=*red_input_data,red_min=*ir_input_data;
	float ir_max = *ir_input_data, ir_min = *ir_input_data;
	float red_max = *red_input_data, red_min = *red_input_data;
	
    float R;
    uint16_t i;
    
    // Ѱ�Ҽ�ֵ
    for(i=1;i<cache_nums;i++)
    {
        if(ir_max<*(ir_input_data+i)) ir_max=*(ir_input_data+i);
        if(ir_min>*(ir_input_data+i)) ir_min=*(ir_input_data+i);
        if(red_max<*(red_input_data+i)) red_max=*(red_input_data+i);
        if(red_min>*(red_input_data+i)) red_min=*(red_input_data+i);
    }
    
    // ����źű仯�Ƿ��㹻
    float ir_variation = ir_max - ir_min;
    float red_variation = red_max - red_min;
    
    // ����źű仯̫С������Ĭ��ֵ
    if(ir_variation < 1000 || red_variation < 1000) {
        return 98.0; // ��������Ѫ��ֵ
    }
    
    R = (ir_variation * red_min) / (red_variation * ir_min);
	
    // ����Rֵ��Χ�������쳣���
    if(R < 0.1 || R > 3.0) {
        return 98.0; // ��������Ѫ��ֵ
    }
    
    // ʹ�øĽ���Ѫ�����㹫ʽ
    float spo2 = (-45.060 * R * R + 30.354 * R + 94.845);
    
    // ����Ѫ��ֵ�ں���Χ��
    if(spo2 < 70.0) spo2 = 70.0;
    if(spo2 > 100.0) spo2 = 100.0;
    
    return spo2;
}