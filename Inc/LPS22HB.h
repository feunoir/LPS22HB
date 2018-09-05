/*
 * LPS22HB.h
 *
 *  Created on: 2017/09/05
 *      Author: feunoir
 */

#ifndef LPS22HB_H_
#define LPS22HB_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"


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
	LPS22HB_ADDR_L = (0x5c << 1),		/*!< SA0(=SDO pin) = Ground */
	LPS22HB_ADDR_H = (0x5d << 1)			/*!< SA0(=SDO Pin) = Vdd */
} LPS22HB_ADDR_t;

typedef enum {
	LPS22HB_OK = 0x00,            	/*!< Everything OK */
	LPS22HB_NO_DEVICE,   	/*!< Device is not connected to I2C */
	LPS22HB_FAIL
} LPS22HB_RESULT_t;

typedef enum {
	LPS22HB_PWR_DOWN 	= 0x00,
	LPS22HB_ODR_1HZ  	= 0x10,
	LPS22HB_ODR_10HZ 	= 0x20,
	LPS22HB_ODR_25HZ 	= 0x30,
	LPS22HB_ODR_50HZ 	= 0x40,
	LPS22HB_ODR_75HZ	= 0x50
} LPS22HB_ODR_t;

typedef enum {
	LPS22HB_LPF_DISABLE	= 0x00,
	LPS22HB_LPF_BW9     = 0x08,
	LPS22HB_LPF_BW20    = 0x0c
} LPS22HB_LPF_t;

typedef enum {
	LPS22HB_DRDY_DISABLE 	= 0x00,
	LPS22HB_DRDY_ENABLE		= 0x04
} LPS22HB_DRDY_t;

typedef enum {
	LPS22HB_FIFO_DISABLE	= 0x00,
	LPS22HB_FIFO_ENABLE  	= 0x40
} LPS22HB_FIFO_EN_t;

typedef enum {
	LPS22HB_FIFOMODE_BYPASS 	= 0x00,
	LPS22HB_FIFOMODE_FIFO   	= 0x20,
	LPS22HB_FIFOMODE_STREAM    	= 0x40,
	LPS22HB_FIFOMODE_STR2FIFO 	= 0x60,
	LPS22HB_FIFOMODE_BYP2STR  	= 0x80,
	LPS22HB_FIFOMODE_DSTREAM   	= 0xC0,
	LPS22HB_FIFOMODE_BYP2FIFO 	= 0xE0,
} LPS22HB_FIFO_MODE_t;


typedef struct {
	I2C_HandleTypeDef* hi2c;
	LPS22HB_ADDR_t address;
	float pressure;
	uint32_t pressure_raw;
	float temperature;
	int16_t temperature_raw;
	LPS22HB_ODR_t odr;
} LPS22HB_t;

void LPS22HB_Set_Handle(LPS22HB_t* LPS22HB, I2C_HandleTypeDef* hi2cx);

void LPS22HB_Set_Address(LPS22HB_t* LPS22HB, LPS22HB_ADDR_t addr);

LPS22HB_RESULT_t LPS22HB_Init(LPS22HB_t* LPS22HB, LPS22HB_ODR_t odr);

LPS22HB_RESULT_t LPS22HB_Update_Data(LPS22HB_t* LPS22HB);

float LPS22HB_Get_Pressure(LPS22HB_t* LPS22HB);

uint32_t LPS22HB_Get_PressureRaw(LPS22HB_t* LPS22HB);

float LPS22HB_Get_Temperature(LPS22HB_t* LPS22HB);

int16_t LPS22HB_Get_TemperatureRaw(LPS22HB_t* LPS22HB);

LPS22HB_RESULT_t LPS22HB_Set_ODR(LPS22HB_t* LPS22HB, LPS22HB_ODR_t odr);

LPS22HB_RESULT_t LPS22HB_Set_LPF(LPS22HB_t* LPS22HB, LPS22HB_LPF_t lpf);

LPS22HB_RESULT_t LPS22HB_Set_DRDY(LPS22HB_t* LPS22HB, LPS22HB_DRDY_t drdy);

LPS22HB_RESULT_t LPS22HB_Set_FIFO(LPS22HB_t* LPS22HB, LPS22HB_FIFO_EN_t fifo_en, LPS22HB_FIFO_MODE_t fifo_mode);

LPS22HB_RESULT_t LPS22HB_Reset_FIFO(LPS22HB_t* LPS22HB);

LPS22HB_RESULT_t LPS22HB_Write_Reg(LPS22HB_t* LPS22HB, uint8_t reg_addr, uint8_t mask, uint8_t data);

LPS22HB_RESULT_t LPS22HB_Read_Reg(LPS22HB_t* LPS22HB, uint8_t reg_addr, uint8_t* p_dt);

#endif /* LPS22HB_H_ */
