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


#define DBG_MSG     "HDL_AS1115"
#define DBG_LVL     2
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/
#define CTR_SHUTDOWN	0x0C
#define CTR_DECODEEN	0x09
#define CTR_INTENSITY	0x0A
#define CTR_SCANLIMIT	0x0B
#define CTR_FEATURE		0x0E
#define CTR_ADDRESS		0x2D

#define CMD_KEY       0x1C

#define AS1115_SL     0x0A     //'-'
#define AS1115_H      0x0C
#define AS1115_P      0x0E
#define AS1115_C      0x0D
#define AS1115_O      0x00
#define AS1115_2      0x02
#define AS1115_BLANK  0x0F

#define LOOP_CNT      10000

#define					AS1115_ADDR							0x00
#define         HAL_I2C1_INSTANCE				hi2c1
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
TypeDef_AS1115_t as1115;


uint8_t Display(uint8_t *buf);
uint8_t Default_As1115(void);

/*
*********************************************************************************************************
*                                               FUNCTION PROTOTYPES
*********************************************************************************************************
*/


uint8_t Default_As1115(void)
{
	static uint8_t cnt =0U;
	static uint8_t sendbuf[6][3] = 
	{
		//{CTR_FEATURE,		0x02},	// Reset Enable
		{0x00,CTR_SHUTDOWN, 	0x81}, 	// Normal Operation (Reset Feature Register Unchanged )
		{0x00,CTR_INTENSITY, 	0x02},	// Global Type , Bright Level 0-F (0 : low , F : high )
		{0x00,CTR_ADDRESS, 	AS1115_ADDR},	// Global Type , Bright Level 0-F (0 : low , F : high )
		{AS1115_ADDR,CTR_FEATURE,		0x00},	// Reset Disable
		{AS1115_ADDR,CTR_SCANLIMIT,	0x07}, 	// Segment Enable Cnt 0-7 ( 7 all enable )
		{AS1115_ADDR,CTR_DECODEEN,	0xFF},	// Decode mode , refer to DS
	};
	uint8_t rtn = false;
	
	if(as1115.write(sendbuf[cnt],2) == true)
	{
		cnt += 1;
		if(cnt == 6U)
		{
			cnt = 0U;
			rtn = true;
		}
	}	
	return rtn;
}



typedef enum{
  R_SEND_CMD = 0,
	R_WAIT_CMD ,
	R_RECEIVE_DATA_REQUEST,
	R_RECEIVED_DATA,
}Read_Step_T;
uint8_t ReadAs1115_it(uint8_t* pbuf,uint8_t* pCmd, uint8_t siz)
{
	static Read_Step_T step;
	uint8_t rtn = false;
	
	switch(step)
	{
		case R_SEND_CMD:
			if(as1115.reg.TxReady != false)
			{
				as1115.reg.TxReady = false;
				if(HAL_I2C_Master_Transmit_IT(&hi2c1,AS1115_ADDR<<1,pCmd,1) != HAL_OK)	return rtn;
				step = R_WAIT_CMD;
			}
			break;
			
		case R_WAIT_CMD:
			if(as1115.reg.TxReady == true)
			{
				step = R_RECEIVE_DATA_REQUEST;
			}
			break;
		case R_RECEIVE_DATA_REQUEST:
			if(as1115.reg.RxReady != false)
			{
				as1115.reg.RxReady = false;
				if(HAL_I2C_Master_Receive_IT(&hi2c1,AS1115_ADDR<<1,pbuf/*as1115.Keyscan*/,siz/*as1115.reg.rx_len*/) != HAL_OK)	return rtn;
				step = R_RECEIVED_DATA;
			}
			break;
		case R_RECEIVED_DATA:
			if(as1115.reg.RxReady == true)
			{
				rtn = true;
				step = R_SEND_CMD;
			}
			break;
		default:
			step = R_SEND_CMD;
			break;
	}
	return rtn;
	
}
typedef enum{
  W_SEND_CMD = 0,
	W_WAIT_CMD ,
}Write_Step_T;
uint8_t WriteAs1115(uint8_t *buf, uint16_t len)
{
	uint8_t rtn = false;
	static Write_Step_T step;

	switch(step)
	{
		case W_SEND_CMD:
			if(as1115.reg.TxReady != false)
			{
				as1115.reg.TxReady = false;	
				if(HAL_I2C_Master_Transmit_IT(&HAL_I2C1_INSTANCE,((uint8_t)*buf)<<1,buf+1, len) != HAL_OK)	return rtn;
				step = W_WAIT_CMD;
			}
			break;
		case W_WAIT_CMD:
			if(as1115.reg.TxReady == true)
			{
				step = W_SEND_CMD;
				rtn = true;
			}
			break;
		default:
			step = W_SEND_CMD;
			break;
	}
	return rtn;
}


uint8_t DisplayControl(uint8_t mod)
{
	uint8_t uStringBuf[64];
	switch(mod)
	{
		case 1:
			// Upper Display
			uStringBuf[0] = 0x02;	 //'2'
			uStringBuf[1] = 0x8E;	 // P'
			uStringBuf[2] = 0x06;	// 'A'
			uStringBuf[3] = 0x0C;	// 'H'
			// Lower Display
			uStringBuf[4]=	0x0F;	
			uStringBuf[5]= 	0x0F;	
			uStringBuf[6]= 	0x0F;	
			uStringBuf[7]= as1115.fan_speed;	
			if(Display(uStringBuf) == true){
				return true;
			}
			break;
		case 2:
			// Upper Display
			uStringBuf[0] = 0x01;	 //'1'
			uStringBuf[1] = 0x8F;	 // '.'
			uStringBuf[2] = 0x01;	// 'I'
			uStringBuf[3] = 0x0D;	// 'C'
			// Lower Display
			uStringBuf[4] = 0x0F;	
			uStringBuf[5]	= 0x0F;	
			uStringBuf[6]	= 0x0F;	
			uStringBuf[7]	= as1115.ic_sel;
			if(Display(uStringBuf) == true)
			{
				return true;
			}
			break;
		default:
			//ERR
			break;
	}
	return false;
}
void Init_as1115(void)
{

  memset(&as1115,  0, sizeof(TypeDef_AS1115_t));
  gP->as1 = &as1115;

  as1115.mod = 0;
  as1115.ic_sel = 0;        // 0: HDC , 1: SDC
  as1115.fan_speed = 1;     // 1~5 STEP
	as1115.reg.TxReady = true;
	as1115.reg.RxReady = true;
	

  as1115.key_event = false;
  as1115.write = WriteAs1115;
  as1115.read = ReadAs1115_it /*ReadAs1115*/;
  as1115.set = Default_As1115;

  as1115.hi2c = &HAL_I2C1_INSTANCE;

}


typedef enum{
  R_CHK_KEYEVENT = 0,
	R_GET_KEYEVENT,
	R_RUN_KEYEVENT,
	R_WAT_KEYEVENT,
}Key_Step_T;
void check_key_event(void)
{
	static Key_Step_T step;
	static uint8_t cmd;
	static uint8_t keyvalue = 0xFF, oldkeyvalue = 0xFF;
	
	
	switch(step)
	{
		case R_CHK_KEYEVENT:
			if(HAL_GPIO_ReadPin(AS_IRQ_GPIO_Port,AS_IRQ_Pin) != GPIO_PIN_SET)
			{
				step = R_GET_KEYEVENT;
			}
			break;
		case R_GET_KEYEVENT:
			cmd = CMD_KEY;
			if(as1115.read(&keyvalue,&cmd,1) == true)
			{
				if(keyvalue != oldkeyvalue)
				{
					oldkeyvalue = keyvalue;
					step = R_RUN_KEYEVENT;					
				}
				else
				{
					step = R_WAT_KEYEVENT;
				}
			}
			break;
		case R_RUN_KEYEVENT:
			if(keyvalue == 0xDF){	// 1st button (upper)
				if(++as1115.mod > 2){
					as1115.mod = 0;
				}
			}
			if(keyvalue == 0xFE){	// 2nd button (Lower)
				switch(as1115.mod)
				{
					case 1:	// Fan Control
						if(as1115.fan_speed == 1){
							as1115.fan_speed = 0;
							HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
						}
						else{
							as1115.fan_speed = 1;
							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
						}
						break;
					case 2:	// Temp , Hum IC Sel
						if(++as1115.ic_sel > 1){
							as1115.ic_sel = 0;
						}
						break;
					default:
						break;
				}
			}
			step = R_CHK_KEYEVENT;
			break;
		case R_WAT_KEYEVENT:
			if(HAL_GPIO_ReadPin(AS_IRQ_GPIO_Port,AS_IRQ_Pin) == GPIO_PIN_SET){
				step = R_CHK_KEYEVENT;
			}				
			break;
		default:
			step = R_CHK_KEYEVENT;
			break;
	}
}

void ReadTemperature(uint8_t* buf,bool select)
{
	uint16_t ubuff;
	
	switch(select)
	{
		case 0:	// HDC
			ubuff = ((uint32_t)(gP->hdc->temperature/10.0f)) %  10;
			buf[0] = (uint8_t)ubuff;
			ubuff = ((uint32_t)(gP->hdc->temperature)) %  10;
			buf[1] = (uint8_t)ubuff| 0x80;
			ubuff = ((uint32_t)(gP->hdc->temperature * 10.0f)) %  10;
			buf[2] = (uint8_t)ubuff;
			ubuff = ((uint32_t)(gP->hdc->temperature * 100.0f)) %  10;
			buf[3] = (uint8_t)ubuff;

			ubuff = ((uint32_t)(gP->hdc->humidity / 10.0f)) %  10;
			buf[4] = (uint8_t)ubuff;
			ubuff = ((uint32_t)(gP->hdc->humidity)) %  10;
			buf[5] = (uint8_t)ubuff| 0x80;
			ubuff = ((uint32_t)(gP->hdc->humidity * 10.0f)) %  10;
			buf[6] = (uint8_t)ubuff;
			ubuff = ((uint32_t)(gP->hdc->humidity *100.0f)) %  10;
			buf[7] = (uint8_t)ubuff;
			break;
		case 1: //SCD
			ubuff = ((uint32_t)(*gP->sdc->pTemp /10.0f)) %  10;
			buf[0] = (uint8_t)ubuff;
			ubuff = ((uint32_t)(*gP->sdc->pTemp )) %  10;
			buf[1] = (uint8_t)ubuff | 0x80 ;
			ubuff = ((uint32_t)(*gP->sdc->pTemp  * 10.0f)) %  10;
			buf[2] = (uint8_t)ubuff;
			ubuff = ((uint32_t)(*gP->sdc->pTemp  * 100.0f)) %  10;
			buf[3] = (uint8_t)ubuff;

			ubuff = ((uint32_t)(*gP->sdc->pHumi / 10.0f)) %  10;
			buf[4] = (uint8_t)ubuff;
			ubuff = ((uint32_t)(*gP->sdc->pHumi)) %  10;
			buf[5] = (uint8_t)ubuff | 0x80;
			ubuff = ((uint32_t)(*gP->sdc->pHumi * 10.0f)) %  10;
			buf[6] = (uint8_t)ubuff;
			ubuff = ((uint32_t)(*gP->sdc->pHumi *100.0f)) %  10;
			buf[7] = (uint8_t)ubuff;
			break;
		default:
			//ERROR
			break;
	}
	
}
uint8_t Display(uint8_t *buf)
{
  static uint8_t sendbuf[8];
	static uint8_t cnt =0U;
	uint8_t rtn = false;

	sendbuf[0] = AS1115_ADDR;
	sendbuf[1] = cnt+1;
	sendbuf[2] = buf[cnt];
	if(as1115.write(sendbuf,2) == true)
	{
		cnt += 1;
		if(cnt > 7U)
		{
			cnt = 0U;
			rtn = true;
		}
	}	
	return rtn;
}
void ReadCO2(uint8_t *buf)
{
	uint8_t uCo2[4];
	
 	uCo2[3] = ((uint16_t)(*gP->sdc->pCo2 / 1000.0f)) % 10;
  uCo2[2] = (uint16_t)(*gP->sdc->pCo2 / 100.0f) % 10;
  uCo2[1] = (uint16_t)(*gP->sdc->pCo2 / 10.0f) % 10;
  uCo2[0] = (uint16_t)(*gP->sdc->pCo2 * 1.0f) % 10;

  buf[0] = AS1115_BLANK; 	// '-'
  buf[1] = AS1115_C;  		// 'C'
  buf[2] = AS1115_O;  		// 'O'
  buf[3] = AS1115_2; 			// '2'
 
	if(uCo2[3] != 0x00)   	buf[4] = uCo2[3];
  else									 	buf[4] = 0x0F;
	buf[5] = uCo2[2];
  buf[6] = uCo2[1];
  buf[7] = uCo2[0]| 0x80;
}
typedef enum{
	ST_INIT = 1,
	ST_RD_STB_TEMP,
	ST_RD_TEMP,
	ST_DIS_TEMP,
	ST_RD_STB_CO2,
	ST_RD_CO2,
	ST_DIS_CO2,
	ST_MENU,	
}Sched_t;

void HD_as1115(void)
{
  static Sched_t status;
	static TypeDef_Time_t sched_t;
	static TypeDef_Time_t display_t;	
	static uint8_t ubuf[32];
	static uint8_t bconnected = false;

	if(bconnected == true)  check_key_event();

	/////// Time Scheduller.... START
  if((Now()-sched_t.t)>= 5*ABS_1msec)
  {
    RdNowTime(&sched_t);
  }
  else return;
	/////// Time Scheduller.... END
	
	switch(status)
	{
		case ST_INIT:
			if(as1115.set() == true){
				status = ST_RD_TEMP;
				bconnected = true;
			}
			break;
		case ST_RD_TEMP:
			ReadTemperature(ubuf,as1115.ic_sel);
			
			if(as1115.mod != 0)	status = ST_MENU;
			else								status = ST_DIS_TEMP;
			
			break;
		case ST_DIS_TEMP:
			if(Display(ubuf) == true){
				RdNowTime(&display_t);
				if(gP->sdc->bconnected ==true){
					status = ST_RD_STB_CO2;
				}
				else{
					status = ST_RD_STB_TEMP;
				}					
			}
			break;
		case ST_RD_STB_CO2:
			if((Now()-display_t.t)>= 3*ABS_1000msec){	// 3s Delay
				status = ST_RD_CO2;
			}
			break;
		case ST_RD_CO2:
			ReadCO2(ubuf);
			status = ST_DIS_CO2;
			break;
		case ST_DIS_CO2:
			if(Display(ubuf) == true){
				status = ST_RD_STB_TEMP;
				RdNowTime(&display_t);
			}
			break;
		case ST_RD_STB_TEMP:
			if((Now()-display_t.t)>= 1*ABS_1000msec){	// 1s Delay
				status = ST_RD_TEMP;
			}
			break;
		case ST_MENU:
    	DisplayControl(as1115.mod);
			if(as1115.mod == 0)		status = ST_DIS_TEMP;
			break;			
		default:
			status = ST_INIT;
			break;
	}
}
