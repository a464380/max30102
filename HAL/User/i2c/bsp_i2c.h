#ifndef _BSP_I2C_H
#define _BSP_I2C_H


#include <inttypes.h>
#include "main.h"
extern I2C_HandleTypeDef I2C_Handle;
#define  i2c_transmit(pdata,data_size)              HAL_I2C_Master_Transmit(&I2C_Handle,I2C_WRITE_ADDR,pdata,data_size,10)
#define  i2c_receive(pdata,data_size)   			HAL_I2C_Master_Receive(&I2C_Handle,I2C_READ_ADDR,pdata,data_size,10)
#define  delay_ms(ms)                               HAL_Delay(ms)

/* ����I2C�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����4�д��뼴������ı�SCL��SDA������ */

#define SENSORS_I2C              		      I2C2
#define SENSORS_I2C_RCC_CLK_ENABLE()   	 __HAL_RCC_I2C2_CLK_ENABLE()
#define SENSORS_I2C_FORCE_RESET()    		 __HAL_RCC_I2C2_FORCE_RESET()
#define SENSORS_I2C_RELEASE_RESET()  		 __HAL_RCC_I2C2_RELEASE_RESET()

/*���Ŷ���*/ 
#define SENSORS_I2C_SCL_GPIO_PORT         GPIOB
#define SENSORS_I2C_SCL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define SENSORS_I2C_SCL_GPIO_PIN         	GPIO_PIN_10
 
#define SENSORS_I2C_SDA_GPIO_PORT         GPIOB
#define SENSORS_I2C_SDA_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define SENSORS_I2C_SDA_GPIO_PIN          GPIO_PIN_11

#define SENSORS_I2C_AF                  	 GPIO_AF4_I2C2




void I2cMaster_Init(void);




#endif

