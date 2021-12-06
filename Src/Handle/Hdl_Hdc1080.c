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

#define DBG_MSG     "HDL_HDC"
#define DBG_LVL     2
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/



#define	HDC_1080_ADD							0x40
#define	HDC_1080_MANUFACTUREID			0xFE
#define	HDC_1080_DEVICEID					0xFF
#define	HDC_1080_CONFIGURATION			0x02
#define	HDC_1080_TEMPERATURE_REGI		0x00
#define	HDC_1080_HUMIDITY_REGI			0x01

#define         HAL_I2C3_INSTANCE                        hi2c3
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

static TypeDef_HDC1080_t		hdc1080;
uint8_t ManufactureIDString[32];         // read manufacture Id
/*
*********************************************************************************************************
*                                               FUNCTION PROTOTYPES
*********************************************************************************************************
*/

typedef enum{
  R_SEND_CMD = 0,
	R_WAIT_CMD ,
	R_RECEIVE_DATA_REQUEST,
	R_RECEIVED_DATA,
}Read_Step_T;
uint8_t ReadHdc1080_it(uint8_t* pbuf,uint8_t* cmd,uint8_t siz)
{
	static Read_Step_T step;
	uint8_t rtn = false;

	switch(step)
	{
		case R_SEND_CMD:
			if(hdc1080.reg.TxReady != false)
			{
				hdc1080.reg.TxReady = false;
				if(HAL_I2C_Master_Transmit_IT(&hi2c3,HDC_1080_ADD<<1,cmd,1) != HAL_OK)	return rtn;
				step = R_WAIT_CMD;
			}
			break;
		case R_WAIT_CMD:
			if(hdc1080.reg.TxReady == true)
			{
				step = R_RECEIVE_DATA_REQUEST;
			}
			break;
		case R_RECEIVE_DATA_REQUEST:
			if(hdc1080.reg.RxReady != false)
			{
				hdc1080.reg.RxReady = false;
				if(HAL_I2C_Master_Receive_IT(&hi2c3,HDC_1080_ADD<<1,pbuf,siz) != HAL_OK)	return rtn;
				step = R_RECEIVED_DATA;
			}
			break;
		case R_RECEIVED_DATA:
			if(hdc1080.reg.RxReady == true)
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

uint8_t WriteHdc1080(uint8_t *buf, uint16_t len)
{
	uint8_t rtn = false;
	static Write_Step_T step;

	switch(step)
	{
		case W_SEND_CMD:
			if(hdc1080.reg.TxReady != false)
			{
				hdc1080.reg.TxReady = false;
				if(HAL_I2C_Master_Transmit_IT(&hi2c3,HDC_1080_ADD<<1,buf,len) != HAL_OK)	return rtn;
				step = W_WAIT_CMD;
			}
			break;
		case W_WAIT_CMD:
			if(hdc1080.reg.TxReady == true)
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
void Init_hdc1080(void)
{
  memset(&hdc1080, 0, sizeof(TypeDef_HDC1080_t));

  gP->hdc = &hdc1080;
  hdc1080.hi2c = &HAL_I2C3_INSTANCE;
  hdc1080.write = WriteHdc1080;
	hdc1080.read = ReadHdc1080_it;
  hdc1080.reg.TxReady = true;
	hdc1080.reg.RxReady = true;
	hdc1080.pMfID = ManufactureIDString;
}


typedef enum{
  ST_PWRON = 0,
	ST_CHECKID,
  ST_CONFIG,
  ST_WORK,
	ST_NOWORK,
}Sched_t;


void HD_hdc1080(void)
{
  static Sched_t status;
  static TypeDef_Time_t sched_t;

	static uint8_t buff[32];
	static uint8_t cmd;

  uint16_t temp_x,humi_x;

	// Time Scheduller.... START
  if((Now()-sched_t.t)>= 5*ABS_100msec)
  {
    RdNowTime(&sched_t);
  }
  else return;
  // Time Scheduller .... END

  switch(status)
  {
    case ST_PWRON :
			hdc1080.reg.TxReady = true;
			status = ST_CHECKID;
      break;
		case ST_CHECKID:
			cmd = HDC_1080_MANUFACTUREID;
			if(hdc1080.read(hdc1080.pMfID,&cmd,2) == true){
				if(*hdc1080.pMfID == 'T'){
					status = ST_CONFIG;		// Init Complete Jodge
				}
				else{
					status = ST_NOWORK;
				}
			}
			break;
    case ST_CONFIG :
			buff[0] = HDC_1080_CONFIGURATION;
			buff[1] = 0x10;
			buff[2] = 0x00;
			if(hdc1080.write(buff,3) == true)
			{
				status = ST_WORK;
			}
      break;
    case ST_WORK:
			cmd = HDC_1080_TEMPERATURE_REGI;
			if(hdc1080.read(buff,&cmd,4) == true){
				temp_x =((buff[0]<<8)|buff[1]);
				humi_x =((buff[2]<<8)|buff[3]);
				hdc1080.temperature  = (float)((temp_x/65536.0f)*165.0f)-40.0f;
				hdc1080.humidity     = (float)((humi_x/65536.0f)*100.0f);
			}
			break;
		case ST_NOWORK:
			asm("NOP");
			// No Connected HDC1080 Device
			break;
    default:	// Init
      status = ST_PWRON;
      break;
  }
}
void hdc1080_rx_event(void)
{
}
void hdc1080_irq_rx(uint8_t buf)
{

}
static
void hdc1080_init(void)
{

}
TypeDef_Device_t i2c_hdc1080 = {
	(I2C_HandleTypeDef*)&hi2c3,	//hardware handle
	(I2C_TypeDef*)I2C3,					//hardware instance
	hdc1080_init,								//initialize function
	hdc1080_rx_event,						//control function
	NULL,												//external interrupt callback
	hdc1080_irq_rx,							//communication interrupt callback
};