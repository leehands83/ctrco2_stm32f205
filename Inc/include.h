/*
*********************************************************************************************************
*
*
*                          (c) Copyright 2019-2019, Namhun, LEE.
*                                           All Rights Reserved
* File : Includes.h
* By   : Namhun, LEE
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*   INCLUDE
*********************************************************************************************************
*/
#ifndef __INCLUDES_H
#define __INCLUDES_H

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>
#include <limits.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <intrinsics.h>

#include "stm32f2xx_hal.h"
#include "DataTypes.h"
#include "Ascii.h"
#include "Errors.h"
#include "main.h"
#include "hw_config.h"

#include "hdc1080.h"

/*
*********************************************************************************************************
*   GLOBAL DEFINE DECLARE
*********************************************************************************************************
*/
extern struct global_t* gP;


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern DMA_HandleTypeDef hdma_i2c3_rx;

/*
*********************************************************************************************************
*   FUNCTION PROTOTYPE DECLARE
*********************************************************************************************************
*/
//main.c
extern void Error_Handler(void);
//HW_CONFIG.C
void Time_Update(void);
void RdNowTime(struct time_t* t);
void ClrNowTime(struct time_t* t);
INT32 Now(void);
extern void hw_init(void);


extern TIM_HandleTypeDef htim1;
//Handle_AS1115.c
extern void HD_as1115(void);
extern void Init_as1115(void);

//Handle_HDC1080.c
extern void HD_hdc1080(void);
extern void Init_hdc1080(void);

//Handle_SDC30.c
extern void HD_sdc30(void);
extern void Init_sdc30(void);
//Handle_System.c
void HD_system(void);

//Handle_Usb.c
extern void HD_vcp(void);
extern void Init_Vcp(void);

extern void con_cmd_parser(TypeDef_Ser_t*, TypeDef_Shell_t *, bool);
extern void con_cmd_execute(TypeDef_Shell_t *, bool);

// Handle_LED.c
extern void led_init(void);
extern void led_control(void);

//Shell_mini.c
//extern void console_Init(void);

// DEBUG
//extern int DBG_PRINTF(const char *MSG, const char *format, ...);

#endif
