/*
 * LPS22HB.c
 *
 *  Created on: 2017/09/05
 *      Author: feunoir
 */

/* Includes ------------------------------------------------------------------*/
#include "LPS22HB.h"

void LPS22HB_Set_Handle(LPS22HB_t* LPS22HB, I2C_HandleTypeDef* hi2cx)
{
	LPS22HB->hi2c = hi2cx;
}

void LPS22HB_Set_Address(LPS22HB_t* LPS22HB, LPS22HB_ADDR_t Address)
{
	LPS22HB->address = Address;
}

LPS22HB_RESULT_t LPS22HB_Init(LPS22HB_t* LPS22HB, LPS22HB_ODR_t ODR)
{
	uint8_t dt[1];


	if ( HAL_I2C_IsDeviceReady(LPS22HB->hi2c, LPS22HB->address, 3, 0xFFFF) != HAL_OK ) {
		return LPS22HB_NO_DEVICE;
	}

	//	Who am I check
	HAL_I2C_Mem_Read(LPS22HB->hi2c, LPS22HB->address, LPS22HB_WHO_AM_I, 1, dt, 1, 0xFFFF);

    if (dt[0] != I_AM_LPS22HB) {
    	return LPS22HB_FAIL;
    }
    if ( LPS22HB_Set_ODR(LPS22HB, ODR) != LPS22HB_OK ) {
    	return LPS22HB_FAIL;
    }
    LPS22HB->odr = ODR;

    return LPS22HB_OK;
}

LPS22HB_RESULT_t LPS22HB_Update_Data(LPS22HB_t* LPS22HB)
{
	HAL_StatusTypeDef res[2];
	uint8_t dt[3];

	// 	get pressure data
	res[0] = HAL_I2C_Mem_Read(LPS22HB->hi2c, LPS22HB->address, LPS22HB_PRESS_POUT_XL | 0x80, 1, dt, 3, 0xFFFF);
	LPS22HB->pressure_raw = dt[2] << 16 | dt[1] << 8 | dt[0];
	LPS22HB->pressure = (float)LPS22HB->pressure_raw / 4096.0f;

	res[1] = HAL_I2C_Mem_Read(LPS22HB->hi2c, LPS22HB->address, LPS22HB_TEMP_OUT_L | 0x80, 1, dt, 2, 0xFFFF);
	LPS22HB->temperature_raw = dt[1] << 8 | dt[0];
	LPS22HB->temperature = (float)LPS22HB->temperature_raw / 100.0f;

	if( res[0] != HAL_OK || res[1] != HAL_OK ) {
		return LPS22HB_FAIL;
	}

	return LPS22HB_OK;
}

float LPS22HB_Get_Pressure(LPS22HB_t* LPS22HB)
{
	return LPS22HB->pressure;
}

uint32_t LPS22HB_Get_PressureRaw(LPS22HB_t* LPS22HB)
{
	return LPS22HB->pressure_raw;
}

float LPS22HB_Get_Temperature(LPS22HB_t* LPS22HB)
{
	return LPS22HB->temperature;
}

int16_t LPS22HB_Get_TemperatureRaw(LPS22HB_t* LPS22HB)
{
	return LPS22HB->temperature_raw;
}

LPS22HB_RESULT_t LPS22HB_Set_ODR(LPS22HB_t* LPS22HB, LPS22HB_ODR_t odr)
{
	HAL_StatusTypeDef res[3];
	uint8_t dt[2];

	dt[0] = LPS22HB_CTRL_REG1;
	res[0] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);
	res[1] = HAL_I2C_Master_Receive(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);

	dt[1] = dt[0];
	dt[1] &= 0xff ^ 0x70;
	dt[1] |= odr;
	dt[0] =LPS22HB_CTRL_REG1;

	res[2] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);

	if( res[0] != HAL_OK || res[1] != HAL_OK || res[2] != HAL_OK ) {
		return LPS22HB_FAIL;
	}

	return LPS22HB_OK;
}

LPS22HB_RESULT_t LPS22HB_Set_LPF(LPS22HB_t* LPS22HB, LPS22HB_LPF_t lpf)
{
	HAL_StatusTypeDef res[3];
	uint8_t dt[2];

	dt[0] = LPS22HB_CTRL_REG1;
	res[0] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 1, 0xFFFF);
	res[1] = HAL_I2C_Master_Receive(LPS22HB->hi2c, LPS22HB->address, dt, 1, 0xFFFF);

	dt[1] = dt[0];
	dt[1] &= 0xff ^ 0x0c;
	dt[1] |= lpf;
	dt[0] = LPS22HB_CTRL_REG1;

	res[2] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 2, 0xFFFF);

	if( res[0] != HAL_OK || res[1] != HAL_OK || res[2] != HAL_OK ) {
		return LPS22HB_FAIL;
	}

	return LPS22HB_OK;
}

LPS22HB_RESULT_t LPS22HB_Set_DRDY(LPS22HB_t* LPS22HB, LPS22HB_DRDY_t drdy)
{
	HAL_StatusTypeDef res[3];
	uint8_t dt[2];

	dt[0] = LPS22HB_CTRL_REG1;
	res[0] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 1, 0xFFFF);
	res[1] = HAL_I2C_Master_Receive(LPS22HB->hi2c, LPS22HB->address, dt, 1, 0xFFFF);

	dt[1] = dt[0];
	dt[1] &= 0xff ^ 0x04;
	dt[1] |= drdy;

	dt[0] = LPS22HB_CTRL_REG1;
	res[2] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 2, 0xFFFF);

	if( res[0] != HAL_OK || res[1] != HAL_OK || res[2] != HAL_OK ) {
		return LPS22HB_FAIL;
	}

	return LPS22HB_OK;

}

LPS22HB_RESULT_t LPS22HB_Set_FIFO(LPS22HB_t* LPS22HB, LPS22HB_FIFO_EN_t fifo_en, LPS22HB_FIFO_MODE_t fifo_mode)
{
	LPS22HB_RESULT_t res[3];

	res[0] = LPS22HB_Write_Reg(LPS22HB, LPS22HB_CTRL_REG2, 0x40, fifo_en);
	res[1] = LPS22HB_Reset_FIFO(LPS22HB);
	res[2] = LPS22HB_Write_Reg(LPS22HB, LPS22HB_FIFO_CTRL, 0xE0, fifo_mode);

	if( res[0] != LPS22HB_OK || res[1] != LPS22HB_OK || res[2] != LPS22HB_OK ) {
		return LPS22HB_FAIL;
	}

	return LPS22HB_OK;
}

LPS22HB_RESULT_t LPS22HB_Reset_FIFO(LPS22HB_t* LPS22HB)
{
	LPS22HB_RESULT_t res;
	res = LPS22HB_Write_Reg(LPS22HB, LPS22HB_FIFO_CTRL, 0xE0, 0x00);

	if( res != LPS22HB_OK ) {
		return LPS22HB_FAIL;
	}

	return LPS22HB_OK;
}

LPS22HB_RESULT_t LPS22HB_Write_Reg(LPS22HB_t* LPS22HB, uint8_t reg_addr, uint8_t mask, uint8_t data)
{
	LPS22HB_RESULT_t read_res;
	HAL_StatusTypeDef write_res;
	uint8_t dt[2];

	read_res = LPS22HB_Read_Reg(LPS22HB, reg_addr, dt);

	dt[1] = dt[0];
	dt[1] &= 0xff ^ mask;
	dt[1] |= data;

	dt[0] = reg_addr;
	write_res = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 2, 0xFFFF);

	if( read_res != LPS22HB_OK || write_res != HAL_OK ) {
		return LPS22HB_FAIL;
	}

	return LPS22HB_OK;
}

LPS22HB_RESULT_t LPS22HB_Read_Reg(LPS22HB_t* LPS22HB, uint8_t reg_addr, uint8_t* p_dt)
{
	HAL_StatusTypeDef res;

	res = HAL_I2C_Mem_Read(LPS22HB->hi2c, LPS22HB->address, reg_addr, 1, p_dt, 1, 0xFFFF);

	if( res != HAL_OK ) {
		return LPS22HB_FAIL;
	}

	return LPS22HB_OK;
}
