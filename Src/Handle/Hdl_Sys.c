/*
*********************************************************************************************************
*                                           HDC1080 PROGRAM
*
*                          (c) Copyright 2019-2019, Namhun LEE STEPBY
*                                           All Rights Reserved
* File : Handle_hdc1080.c
* By   : Namhun LEE
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*   INCLUDE
*********************************************************************************************************
*/
#include "includes.h"


#define DBG_MSG     "HDL_SYS"
#define DBG_LVL     2
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               GLOBAL VARIABLES
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                               LOCALL VARIABLES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               FUNCTION PROTOTYPES
*********************************************************************************************************
*/

typedef enum{
  ST_PWRON = 0,
  ST_INIT,
  ST_WORK,
}Sched_t;

void HD_system(void)
{
  static Sched_t status;
	static TypeDef_Time_t sched_t;
  
  // Time Scheduller.... START
  if((Now()-sched_t.t)>= 2*ABS_1000msec)
  {
    RdNowTime(&sched_t);
  }
  else return;
  // Time Scheduller .... END
  
  switch(status)
  {
		case ST_PWRON :
			status = ST_INIT;
			break;
		case ST_INIT :
			if(*(__IO uint32_t *)CFG_RST_RAM_ADDR != CFG_RST_OFF)      
			{
				*(__IO uint32_t *)CFG_RST_RAM_ADDR = CFG_RST_OFF;
				HAL_NVIC_SystemReset();
			}
			status = ST_WORK;
			break;
		case ST_WORK:
			asm("NOP");
			break;
		default:
			status = ST_PWRON;
			break;
  }
}
