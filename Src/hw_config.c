/*
*********************************************************************************************************
*                                                HW_CONFIG.C
*
*                          (c) Copyright 2019-2019, Namhun LEEH
*                                           All Rights Reserved
* Date : 2019-02-05
* File : hw_config.c
* By   : Namhun LEE
*
*********************************************************************************************************
*/
#include "includes.h"

static struct global_t gd;				// Global Data

struct global_t             *gP=&gd;
//static TypeDef_SDC30_t		  sdc30;

TypeDef_Dbg_t dbg;

volatile static int SystemTickTimer;

/*
*********************************************************************************************************
* SYSTEM TICK TIME UPDATE FUNCTION
* Arguments  : void
* Returns    : void
*********************************************************************************************************
*/
void Time_Update(void)
{
	SystemTickTimer++;
  gP->gTickTime++;
}
/*
*********************************************************************************************************
* READ NOW TIME FUNCTION
* Arguments  : struct time_t*
* Returns    : void
*********************************************************************************************************
*/
void RdNowTime(struct time_t* t)
{
	__disable_interrupt();
    t->s  = TRUE;
	t->t = SystemTickTimer ;
	__enable_interrupt();
}
/*
*********************************************************************************************************
* CLEAR NOW TIME FUNCTION
* Arguments  : struct time_t*
* Returns    : void
*********************************************************************************************************
*/
void ClrNowTime(struct time_t* t)
{
    t->s  = FALSE;
	t->t  = 0;
}
/*
*********************************************************************************************************
* CURRENT TICK TIME READ FUNCTION
* Arguments  : void
* Returns    : INT32
*********************************************************************************************************
*/
INT32 Now(void)
{
  UINT32 rtn;
  __disable_interrupt();
  rtn = SystemTickTimer ;
  __enable_interrupt();
  return rtn;
}


void hw_init(void)
{
  memset(&gd,  0, sizeof(struct global_t));
  
  gP = &gd;    
  gP->dbg = &dbg;  
	
	dbg.b.lvl  = CGF_DBG_LEVEL;
	
	gP->i2c->dev = &i2c_hdc1080;
	//gP->i2c->dev->init();
	
  Init_sdc30();
  Init_as1115();
  Init_hdc1080();
  Init_led();  
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);  
}
