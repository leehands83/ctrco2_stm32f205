/*
*********************************************************************************************************
*                                 LED HANDLE PROGRAM
*
*                          (c) Copyright 2002-2011, NHLEE. STEP BY
*                                           All Rights Reserved
* Date : 
* File : Hdl_led.c
* By   : 
*********************************************************************************************************
*/
#include "includes.h"


#define DBG_MSG     "HDL_LED"
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
*                                               LOCAL VARIABLES
*********************************************************************************************************
*/
static TypeDef_Led_t  led;
static TypeDef_Time_t sched_t;
/*
*********************************************************************************************************
*                                               FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static
void led_blink_time_set(uint32_t led_type, uint32_t cnt)
{
  switch(led_type)
  {
    case LED_STAT1_IDX:
      led.stat1_btime = !led.stat1_btime ? cnt : led.stat1_btime;
      break;
    case LED_STAT2_IDX:
      led.stat2_btime = !led.stat2_btime ? cnt : led.stat2_btime;
      break;
    default:// LED_STAT3_IDX:
      led.stat3_btime = !led.stat3_btime ? cnt : led.stat3_btime;
      break;
  }
}
/*
*********************************************************************************************************
* LED INIT FUNCTION
*********************************************************************************************************
*/
void Init_led(void)
{
  gP->led = &led;

  led.en        = true;
  led.btime_set = led_blink_time_set;
  RdNowTime(&sched_t);
  STAT1_LED_OFF();
  STAT2_LED_OFF();
  STAT3_LED_OFF();
  gP->led->btime_set(LED_STAT1_IDX, LED_FORCE_OFF_TIME);
}
/*
*********************************************************************************************************
* LED OUTPUT CONTROL FUNCTION
*********************************************************************************************************
*/
typedef enum{
  BLINK_COUNT_SET_WAIT,
  BLINK_DISCOUNTING,
  BLINK_FORCE_ON,
}TypeDef_LedBlink_t;

static TypeDef_LedBlink_t stat1_blink_state;

void HD_Led(void)
{
  static uint32_t stat1_toggle_time = LED_TOGGLE_TIME;
  ////////////////////////// STS LED  ///////////////////////////
  if((Now()-sched_t.t)>= 15*ABS_1msec)
    RdNowTime(&sched_t);
  else return;
  ///////////////////////////////////////////////////////////////
	
  // STAT1 led processing
  if(0)
  {
    static uint32_t stat1_force_on_time;

    switch(stat1_blink_state)
    {
      case BLINK_FORCE_ON :      
        if(!--stat1_force_on_time)
          stat1_blink_state = BLINK_COUNT_SET_WAIT;
        break;

      case BLINK_DISCOUNTING :
        if(!--led.stat1_btime)
        {
          stat1_blink_state   = BLINK_FORCE_ON;
          stat1_force_on_time = LED_FORCE_ON_TIME;
          STAT1_LED_ON();
        }
        break;
      default :
        if(led.stat1_btime)
        {
          STAT1_LED_OFF();
          stat1_blink_state = BLINK_DISCOUNTING;
        }
        else STAT1_LED_ON();
        break;
      }
  }
  else
  {
    if(!--stat1_toggle_time)
    {
      stat1_toggle_time = LED_TOGGLE_TIME;
      STAT1_LED_TGL();
    }
  }  
  if(gP->as1->ic_sel == 1)
  {
    STAT2_LED_OFF();     // HDC READ
  }
  else
  {
    STAT2_LED_ON();      // SDC30
  }
  if(gP->as1->fan_speed == 0)
  {
    STAT3_LED_OFF();
  }
  else
  {
    STAT3_LED_ON();
  }
}
/*
*********************************************************************************************************
*	                                               END
*********************************************************************************************************
*/

