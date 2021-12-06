/*
*********************************************************************************************************
*
*
*                          (c) Copyright 2019-2019, Namhun, LEE.
*                                           All Rights Reserved
* File : hw_config.h
* By   : Namhun, LEE
*********************************************************************************************************
*/

#define I2C_DMA_TRANSIT



#define CFG_RST_RAM_ADDR          	0x2001FFF0   // NH
#define CFG_RST_ON									0xCAFECAFE   // NH
#define CFG_RST_OFF								  0xBEEFBEEF   // NH


#define CGF_DBG_LEVEL               1       // 0: Low Level View
                                            // 1:
                                            // 2:

/*
*********************************************************************************************************
*   SYSTEM TICK TIME DECLARE
*********************************************************************************************************
*/
#define ABSOLUTE_1_SECOND           1000
#define TICK_RESOLUTION             1               // Tick time resolution (ms)
#define ABS_1msec                   (1/TICK_RESOLUTION)
#define ABS_10msec                  (10/TICK_RESOLUTION)
#define ABS_100msec                 (100/TICK_RESOLUTION)
#define ABS_1000msec                (1000/TICK_RESOLUTION)

/*
*********************************************************************************************************
*   CONSTANTS for SHELL
*********************************************************************************************************
*/
#define CFG_CONSOLE_LENGTH          1024             // console receive buffer length

#define CFG_ARG_CNT                 28              // Total Argument Count
#define CFG_ARG_LENGTH              128             // command argument length

/*
*********************************************************************************************************
*   CONSTANTS for USB
*********************************************************************************************************
*/
#define CFG_DEV2USB_BUF_CNT         (1024*2)

/*
*********************************************************************************************************
*     LED
*********************************************************************************************************
*/
#define LED_STAT1_IDX       0
#define LED_STAT2_IDX       1
#define LED_STAT3_IDX       2

#define LED_FORCE_ON_TIME      2
#define LED_FORCE_OFF_TIME     2

#define LED_TOGGLE_TIME        30     // if 15ms, 15ms x 30 = 450ms

#define STAT1_LED_ON()      HAL_GPIO_WritePin(GPIOA, LED_STAT1_Pin, GPIO_PIN_RESET)
#define STAT1_LED_OFF()     HAL_GPIO_WritePin(GPIOA, LED_STAT1_Pin, GPIO_PIN_SET)
#define STAT1_LED_TGL()     HAL_GPIO_TogglePin(GPIOA, LED_STAT1_Pin)
  
#define STAT2_LED_ON()      HAL_GPIO_WritePin(GPIOA, LED_STAT2_Pin, GPIO_PIN_RESET)
#define STAT2_LED_OFF()     HAL_GPIO_WritePin(GPIOA, LED_STAT2_Pin, GPIO_PIN_SET)
#define STAT2_LED_TGL()     HAL_GPIO_TogglePin(GPIOA, LED_STAT2_Pin)
  
#define STAT3_LED_ON()      HAL_GPIO_WritePin(GPIOA, LED_STAT3_Pin, GPIO_PIN_RESET)
#define STAT3_LED_OFF()     HAL_GPIO_WritePin(GPIOA, LED_STAT3_Pin, GPIO_PIN_SET)
#define STAT3_LED_TGL()     HAL_GPIO_TogglePin(GPIOA, LED_STAT3_Pin)
          
