/*
 * LPS22HB.c
 *
 *  Created on: 2017/09/05
 *      Author: feunoir
 */

/* Includes ------------------------------------------------------------------*/
#include "LPS22HB.h"

void LPS22HB_GetHandle(LPS22HB_t* LPS22HB, I2C_HandleTypeDef* hi2cx)
{
	LPS22HB->hi2c = hi2cx;
}

void LPS22HB_SetAddress(LPS22HB_t* LPS22HB, LPS22HB_Address_t Address)
{
	LPS22HB->address = Address;
}

LPS22HB_Result_t LPS22HB_Init(LPS22HB_t* LPS22HB, LPS22HB_ODR_t ODR)
{
	uint8_t dt[1];


	if ( HAL_I2C_IsDeviceReady(LPS22HB->hi2c, LPS22HB->address, 3, 0xFFFF) != HAL_OK ) {
		return LPS22HB_Result_DeviceNotConnected;
	}

	//	Who am I check
	HAL_I2C_Mem_Read(LPS22HB->hi2c, LPS22HB->address, LPS22HB_WHO_AM_I, 1, dt, 1, 0xFFFF);

    if (dt[0] != I_AM_LPS22HB) {
    	return LPS22HB_Result_Fail;
    }
    if ( LPS22HB_SetODR(LPS22HB, ODR) != LPS22HB_Result_Ok ) {
    	return LPS22HB_Result_Fail;
    }
    LPS22HB->odr = ODR;

    return LPS22HB_Result_Ok;
}

LPS22HB_Result_t LPS22HB_GetData(LPS22HB_t* LPS22HB)
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
		return LPS22HB_Result_Fail;
	}

	return LPS22HB_Result_Ok;
}

float LPS22HB_Pressure(LPS22HB_t* LPS22HB)
{
	return LPS22HB->pressure;
}

uint32_t LPS22HB_PressureRaw(LPS22HB_t* LPS22HB)
{
	return LPS22HB->pressure_raw;
}

float LPS22HB_Temperature(LPS22HB_t* LPS22HB)
{
	return LPS22HB->temperature;
}

int16_t LPS22HB_TemperatureRaw(LPS22HB_t* LPS22HB)
{
	return LPS22HB->temperature_raw;
}

LPS22HB_Result_t LPS22HB_SetODR(LPS22HB_t* LPS22HB, LPS22HB_ODR_t ODRConfig)
{
	HAL_StatusTypeDef res[3];
	uint8_t dt[2];

	dt[0] = LPS22HB_CTRL_REG1;
	res[0] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);
	res[1] = HAL_I2C_Master_Receive(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);

	dt[1] = dt[0];
	dt[1] &= 0xff ^ 0x70;
	dt[1] |= ODRConfig;
	dt[0] =LPS22HB_CTRL_REG1;

	res[2] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);

	if( res[0] != HAL_OK || res[1] != HAL_OK || res[2] != HAL_OK ) {
		return LPS22HB_Result_Fail;
	}

	return LPS22HB_Result_Ok;
}

LPS22HB_Result_t LPS22HB_SetLPF(LPS22HB_t* LPS22HB, LPS22HB_LPF_t LPFConfig)
{
	HAL_StatusTypeDef res[3];
	uint8_t dt[2];

	dt[0] = LPS22HB_CTRL_REG1;
	res[0] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 1, 0xFFFF);
	res[1] = HAL_I2C_Master_Receive(LPS22HB->hi2c, LPS22HB->address, dt, 1, 0xFFFF);

	dt[1] = dt[0];
	dt[1] &= 0xff ^ 0x0c;
	dt[1] |= LPFConfig;
	dt[0] = LPS22HB_CTRL_REG1;

	res[2] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 2, 0xFFFF);

	if( res[0] != HAL_OK || res[1] != HAL_OK || res[2] != HAL_OK ) {
		return LPS22HB_Result_Fail;
	}

	return LPS22HB_Result_Ok;
}

LPS22HB_Result_t LPS22HB_SetDRDY(LPS22HB_t* LPS22HB, LPS22HB_DRDY_t DRDYConfig)
{
	HAL_StatusTypeDef res[3];
	uint8_t dt[2];

	dt[0] = LPS22HB_CTRL_REG1;
	res[0] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 1, 0xFFFF);
	res[1] = HAL_I2C_Master_Receive(LPS22HB->hi2c, LPS22HB->address, dt, 1, 0xFFFF);

	dt[1] = dt[0];
	dt[1] &= 0xff ^ 0x04;
	dt[1] |= DRDYConfig;

	dt[0] = LPS22HB_CTRL_REG1;
	res[2] = HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, 2, 0xFFFF);

	if( res[0] != HAL_OK || res[1] != HAL_OK || res[2] != HAL_OK ) {
		return LPS22HB_Result_Fail;
	}

	return LPS22HB_Result_Ok;

}

uint8_t LPS22HB_ReadReg(LPS22HB_t* LPS22HB, uint8_t regaddr)
{
	uint8_t dt[1];

	dt[0] = regaddr;
	HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);
	HAL_I2C_Master_Receive(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);

	return dt[0];
}

void LPS22HB_WriteReg(LPS22HB_t* LPS22HB, uint8_t regaddr, uint8_t data){
	uint8_t dt[2];
	dt[0] = regaddr;
	dt[1] = data;

	HAL_I2C_Master_Transmit(LPS22HB->hi2c, LPS22HB->address, dt, sizeof(dt), 0xFFFF);
}
