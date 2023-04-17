#include "ina226.h"

INA226_DATA ina226_data0,ina226_data1;

void delay_nms(uint16_t ms)  //毫秒
{
//根据不同芯片替换
}

void delay_nus(uint16_t us)//微秒
{
//根据不同芯片替换
}
void INA226_GPIO_Init(void){

    /*  
    HS_HAL_GPIO_INIT_PARAM_S initParam;
	
    initParam.pin = PIN_SCL;
    initParam.mode = HS_HAL_GPIO_DUALDIRECT;
    initParam.intType = HS_HAL_GPIO_INT_NONE;
    initParam.pull = HS_HAL_GPIO_NOPULL;
    initParam.func = GPIO;
    HS_HAL_GPIO_Init(&initParam);	


    initParam.pin = PIN_SDA;
    HS_HAL_GPIO_Init(&initParam);
    根据不同芯片替换 */
	
    IIC_SDA_H;
    IIC_SCL_H;  
    delay_nms(5);	
}


void INA226_IIC_Start(void){
	IIC_SDA_H;
	IIC_SCL_H;
	delay_nus(2);
	IIC_SDA_L;
	delay_nus(2);
	IIC_SCL_L;
}

void INA226_IIC_Stop(void){
	IIC_SCL_L;
	IIC_SDA_L;
	delay_nus(2);
	IIC_SCL_H;
	delay_nus(2);
	IIC_SDA_H;
}

u8 INA226_Wait_Ack(void){
	u8 ucErrTime=0;
	IIC_SDA_H;delay_nus(2);
	IIC_SCL_H;delay_nus(2);
	while(READ_SDA){
		ucErrTime++;
		if(ucErrTime > 250){
			INA226_IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;
	return 0;
}

void INA226_Ack(void){
	IIC_SCL_L;
	IIC_SDA_L;
	delay_nus(2);
	IIC_SCL_H;
	delay_nus(2);
	IIC_SCL_L;
}

void INA226_NAck(void){
	IIC_SCL_L;
	IIC_SDA_H;
	delay_nus(2);
	IIC_SCL_H;
	delay_nus(2);
	IIC_SCL_L;
}

void INA226_Send_Byte(u8 tdata){
	u8 t;
	IIC_SCL_L;
	for(t = 0;t < 8;t++){
		if((tdata&0x80)>>7)
			IIC_SDA_H;
		else
			IIC_SDA_L;
		tdata <<= 1;
		delay_nus(2);
		IIC_SCL_H;
		delay_nus(2);
		IIC_SCL_L;
		delay_nus(2);
	}
}

u8 INA226_Read_Byte(u8 ack){
	u8 i,res = 0;
	IIC_SDA_H;
	for(i = 0;i < 8;i++){
		IIC_SCL_L;
		delay_nus(2);
		IIC_SCL_H;
		res <<= 1;
		if(READ_SDA)res++;
		delay_nus(2);
	}
	if(!ack)
		INA226_NAck();
	else	
		INA226_Ack();
	return res;
}

u16 INA226_ReadData(u8 reg_addr,u8 w_addr,u8 r_addr)
{
	u16 reg_data=0;
	u16 temp=0;
	INA226_IIC_Start();
	INA226_Send_Byte(w_addr);
	if(INA226_Wait_Ack())return 0;
	INA226_Send_Byte(reg_addr);   
	if(INA226_Wait_Ack())return 0;
	INA226_IIC_Start();
	INA226_Send_Byte(r_addr);
	if(INA226_Wait_Ack())return 0;
	reg_data= INA226_Read_Byte(1);
	reg_data=(reg_data<<8)&0xFF00;
	temp=INA226_Read_Byte(0);
	INA226_IIC_Stop();
	reg_data|=temp;
	return reg_data;
}

u8 INA226_WriteData(u8 reg_addr,u16 reg_data,u8 w_addr)
{        
	u8 data_high=(u8)((reg_data&0xFF00)>>8);
	u8 data_low=(u8)reg_data&0x00FF;
	INA226_IIC_Start();
	INA226_Send_Byte(w_addr);   
	if(INA226_Wait_Ack())return 0;
	INA226_Send_Byte(reg_addr );    
	if(INA226_Wait_Ack())return 0;        
	INA226_Send_Byte(data_high);
	if(INA226_Wait_Ack())return 0;        
	INA226_Send_Byte(data_low);
	if(INA226_Wait_Ack())return 0;                 
	INA226_IIC_Stop();
	delay_nms(2);
	return 1;
}

void INA226_Init(AVG_NUM_E avg_num,VBUS_CHANGE_TIME_E vbus_cnum,VSH_CHANGE_TIME_E vsh_cnum,OPERATION_MODE_E mode,u8 w_addr){
	u16 config_reg_data = 0x4000;
	config_reg_data |= (avg_num << 9);
	config_reg_data |= (vbus_cnum << 6);
	config_reg_data |= (vsh_cnum << 3);
	config_reg_data |= mode;
	printf("config = %x calib = %x\r\n",config_reg_data,CAL);
	INA226_WriteData(CONFIG_REG, config_reg_data,w_addr);
	INA226_WriteData(CALIB_REG, CAL,w_addr);
}

//mV				
void Get_Bus_Vol(void){
	ina226_data0.bus_v = (float)INA226_ReadData(BUS_V_REG,CHIP1_WRITE_ADDR,CHIP1_READ_ADDR) * 1.25;
	ina226_data1.bus_v = (float)INA226_ReadData(BUS_V_REG,CHIP2_WRITE_ADDR,CHIP2_READ_ADDR) * 1.25;
}
//mV
void Get_Shunt_Vol(void){
	ina226_data0.shunt_v = (float)INA226_ReadData(SHUNT_V_REG,CHIP1_WRITE_ADDR,CHIP1_READ_ADDR) * 2.5 * 0.001;
	ina226_data1.shunt_v = (float)INA226_ReadData(SHUNT_V_REG,CHIP2_WRITE_ADDR,CHIP2_READ_ADDR) * 2.5 * 0.001 * 0.98;
	if(ina226_data0.shunt_v > 81.92) ina226_data0.shunt_v = 0;
	if(ina226_data1.shunt_v > 81.92) ina226_data1.shunt_v = 0;
}
//mA
void Get_Current(void){
	ina226_data0.cur = (float)INA226_ReadData(CURRENT_REG,CHIP1_WRITE_ADDR,CHIP1_READ_ADDR) * CURRENT_LSB / 1000;
	ina226_data1.cur = (float)INA226_ReadData(CURRENT_REG,CHIP2_WRITE_ADDR,CHIP2_READ_ADDR) * CURRENT_LSB / 1000 * 0.98 ; //0.98是校准
	if(ina226_data1.cur > 16384) ina226_data1.cur = 0;
	if(ina226_data0.cur > 16384) ina226_data0.cur = 0;
}
//mW
void Get_Power(void){
	ina226_data0.power = (float)INA226_ReadData(POWER_REG,CHIP1_WRITE_ADDR,CHIP1_READ_ADDR)* CURRENT_LSB / 1000 * 25;
	ina226_data1.power = (float)INA226_ReadData(POWER_REG,CHIP2_WRITE_ADDR,CHIP2_READ_ADDR)* CURRENT_LSB / 1000 * 25 * 0.98 ;
}

void testINA226(void){
	Get_Bus_Vol();
	Get_Shunt_Vol();
	Get_Current();
	Get_Power();
	printf("bus_in_vol = %.3fV  bus_out_vol = %.3fV\r\n",ina226_data0.bus_v/1000,ina226_data1.bus_v/1000);
	printf("shunt_in_vol = %.2fmV shunt_out_vol = %.2fmV \r\n",ina226_data0.shunt_v,ina226_data1.shunt_v);
	printf("cur_in = %.3fA  cur_out = %.3fA \r\n",ina226_data0.cur/1000,ina226_data1.cur/1000);
	printf("power_in = %.3fW power_out = %.3fW \r\n",ina226_data0.power/1000,ina226_data1.power/1000);
	printf("eff = %f\%\r\n",ina226_data1.power/ina226_data0.power);
}