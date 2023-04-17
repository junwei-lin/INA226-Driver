#ifndef __INA226_H
#define __INA226_H

#include "main.h"


#define PIN_SCL     HS_HAL_PIN_GPIO43 
#define PIN_SDA     HS_HAL_PIN_GPIO44 



#define IIC_SCL_H  // HS_HAL_GPIO_WritePin(PIN_SCL,1)    根据不同芯片替换
#define IIC_SCL_L  // HS_HAL_GPIO_WritePin(PIN_SCL,0) 
#define IIC_SDA_H  // HS_HAL_GPIO_WritePin(PIN_SDA ,1)  
#define IIC_SDA_L  // HS_HAL_GPIO_WritePin(PIN_SDA ,0)  
#define READ_SDA   // HS_HAL_GPIO_ReadPin(PIN_SDA)  

#define CHIP1_WRITE_ADDR                 0x80
#define CHIP1_READ_ADDR                  0x81	 
#define CHIP2_WRITE_ADDR                 0x82
#define CHIP2_READ_ADDR                  0x83

#define CONFIG_REG       0x00  //配置寄存器
#define SHUNT_V_REG      0x01  //分流电压寄存器
#define BUS_V_REG        0x02  //总线电压寄存器
#define POWER_REG        0x03  //功率寄存器
#define CURRENT_REG      0x04  //电流寄存器
#define CALIB_REG        0x05  //较准寄存器
#define MASK_EN_REG      0x06  //屏蔽/使能寄存器
#define ALERT_REG        0x07  //警报限值寄存器
#define MAIN_ID_REG      0xFE  //制造商id寄存器
#define CHIP_ID_REG      0xFF  //芯片id寄存器

#define CURRENT_LSB      500 //单位uA
#define SHUNT_R          5  //单位mΩ
#define CAL              (5120000 / CURRENT_LSB / SHUNT_R) //较准寄存器配置值

/*IN226数据*/
typedef struct{
	float bus_v;
	float shunt_v;
	float cur;
	float power;
}INA226_DATA;

/*取平均次数*/
typedef enum{
	AVG_NUM_1,   //缺省值
	AVG_NUM_4,
	AVG_NUM_16,
	AVG_NUM_128,
	AVG_NUM_256,
	AVG_NUM_512,
	AVG_NUM_1024,
}AVG_NUM_E;


/*总线电压转换时间 us */
typedef enum{
	VBUS_CHANGE_TIME_140,
	VBUS_CHANGE_TIME_204,
	VBUS_CHANGE_TIME_332,
	VBUS_CHANGE_TIME_588,
	VBUS_CHANGE_TIME_1100, //缺省值
	VBUS_CHANGE_TIME_2116,
	VBUS_CHANGE_TIME_4156,
	VBUS_CHANGE_TIME_8244,
}VBUS_CHANGE_TIME_E;


/*分流电压转换时间 us */
typedef enum{
	VSH_CHANGE_TIME_140,
	VSH_CHANGE_TIME_204,
	VSH_CHANGE_TIME_332,
	VSH_CHANGE_TIME_588,
	VSH_CHANGE_TIME_1100, //缺省值
	VSH_CHANGE_TIME_2116,
	VSH_CHANGE_TIME_4156,
	VSH_CHANGE_TIME_8244,
}VSH_CHANGE_TIME_E;

/*操作模式*/
typedef enum{
	POWER_SAVE_MODE1,
	VSH_TRIG_MODE,
	VBUS_TRIG_MODE,
	VSH_VBUS_TRIG_MODE,
	POWER_SAVE_MODE2,
	VSH_CONTINUE_MODE,
	VBUS_CONTINUE_MODE,
	VSH_VBUS_CONTINUE_MODE,//缺省值
}OPERATION_MODE_E;

extern INA226_DATA ina226_data0;
extern INA226_DATA ina226_data1;

void delay_nms(uint16_t ms);
void delay_nus(uint16_t us);
void INA226_GPIO_Init(void);
void INA226_IIC_Start(void);
void INA226_IIC_Stop(void);
u8 INA226_Wait_Ack(void);
void INA226_Ack(void);
void INA226_NAck(void);
void INA226_Send_Byte(u8 tdata);
u8 INA226_Read_Byte(u8 ack);
u16 INA226_Read2Data(u8 reg_addr,u8 w_addr,u8 r_addr);
u8 INA226_WriteData(u8 reg_addr,u16 reg_data,u8 w_addr);
void Get_Bus_Vol(void);
void Get_Shunt_Vol(void);
void Get_Current(void);
void Get_Power(void);
void testINA226(void);
void INA226_Init(AVG_NUM_E avg_num,VBUS_CHANGE_TIME_E vbus_cnum,VSH_CHANGE_TIME_E vsh_cnum,OPERATION_MODE_E mode,u8 w_addr);
#endif