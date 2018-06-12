/*
 * LPS22HB.h
 *
 *  Created on: 2017/09/05
 *      Author: feunoir
 */

#ifndef LPS22HB_H_
#define LPS22HB_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"


//  LPS22HB Address
//  7bit address = 0b101110x(0x5c or 0x5d depends on SA0/SDO)
#define LPS22HB_G_CHIP_ADDR  (0x5c << 1)    // SA0(=SDO pin) = Ground
#define LPS22HB_V_CHIP_ADDR  (0x5d << 1)    // SA0(=SDO pin) = Vdd

//   LPS22HB ID
#define I_AM_LPS22HB            0xb1

//  Register's definition
#define LPS22HB_WHO_AM_I        0x0f
#define LPS22HB_RES_CONF        0x1a

#define LPS22HB_CTRL_REG1       0x10
#define LPS22HB_CTRL_REG2       0x11
#define LPS22HB_CTRL_REG3       0x12

#define LPS22HB_STATUS_REG      0x27
#define LPS22HB_PRESS_POUT_XL   0x28
#define LPS22HB_PRESS_OUT_L     0x29
#define LPS22HB_PRESS_OUT_H     0x2a
#define LPS22HB_TEMP_OUT_L      0x2b
#define LPS22HB_TEMP_OUT_H      0x2c

#define LPS22HB_FIFO_CTRL       0x14
#define LPS22HB_FIFO_STATUS     0x26

typedef enum {
	LPS22HB_Address_L = (0x5c << 1),		/*!< SA0(=SDO pin) = Ground */
	LPS22HB_Address_H = (0x5d << 1)			/*!< SA0(=SDO Pin) = Vdd */
} LPS22HB_Address_t;

typedef enum {
	LPS22HB_Result_Ok = 0x00,            	/*!< Everything OK */
	LPS22HB_Result_DeviceNotConnected,   	/*!< Device is not connected to I2C */
	LPS22HB_Result_Fail
} LPS22HB_Result_t;

typedef enum {
	LPS22HB_PD       = 0x00,
	LPS22HB_ODR_1HZ  = 0x10,
	LPS22HB_ODR_10HZ = 0x20,
	LPS22HB_ODR_25HZ = 0x30,
	LPS22HB_ODR_50HZ = 0x40,
	LPS22HB_ODR_75HZ = 0x50
} LPS22HB_ODR_t;

typedef enum {
	LPS22HB_LPF_DISABLE = 0x00,
	LPS22HB_LPF_BW9     = 0x08,
	LPS22HB_LPF_BW20    = 0x0c
} LPS22HB_LPF_t;

typedef enum {
	LPS22HB_DRDY_DISABLE = 0x00,
	LPS22HB_DRDY_ENABLE  = 0x04
} LPS22HB_DRDY_t;


typedef struct {
	I2C_HandleTypeDef* hi2c;
	LPS22HB_Address_t address;
	float pressure;
	uint32_t pressure_raw;
	float temperature;
	int16_t temperature_raw;
	LPS22HB_ODR_t odr;
} LPS22HB_t;

void LPS22HB_GetHandle(LPS22HB_t* LPS22HB, I2C_HandleTypeDef* hi2cx);

void LPS22HB_SetAddress(LPS22HB_t* LPS22HB, LPS22HB_Address_t Address);

LPS22HB_Result_t LPS22HB_Init(LPS22HB_t* LPS22HB, LPS22HB_ODR_t ODR);

LPS22HB_Result_t LPS22HB_GetData(LPS22HB_t* LPS22HB);

float LPS22HB_Pressure(LPS22HB_t* LPS22HB);

uint32_t LPS22HB_PressureRaw(LPS22HB_t* LPS22HB);

float LPS22HB_Temperature(LPS22HB_t* LPS22HB);

int16_t LPS22HB_TemperatureRaw(LPS22HB_t* LPS22HB);

LPS22HB_Result_t LPS22HB_SetODR(LPS22HB_t* LPS22HB, LPS22HB_ODR_t ODRConfig);

LPS22HB_Result_t LPS22HB_SetLPF(LPS22HB_t* LPS22HB, LPS22HB_LPF_t LPFConfig);

LPS22HB_Result_t LPS22HB_SetDRDY(LPS22HB_t* LPS22HB, LPS22HB_DRDY_t DRDYConfig);

uint8_t LPS22HB_ReadReg(LPS22HB_t* LPS22HB, uint8_t regaddr);

void LPS22HB_WriteReg(LPS22HB_t* LPS22HB, uint8_t regaddr, uint8_t data);



#endif /* LPS22HB_H_ */
