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
#include "main.h"


#define DBG_MSG     "HDL_SCD30"
#define DBG_LVL     2


#define         HAL_I2C2_INSTANCE                        hi2c2

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/
#define SCD_I2C_ADDRESS											0x61
#define SCD_CMD_START_PERIODIC_MEASUREMENT  0x0010
#define SCD_CMD_STOP_PERIODIC_MEASUREMENT   0x0104
#define SCD_CMD_READ_MEASUREMENT            0x0300
#define SCD_CMD_SET_MEASUREMENT_INTERVAL    0x4600

#define SCD_CMD_GET_DATA_READY              0x0202
#define SCD_CMD_GET_VERSION	              	0xD100
#define SCD_CMD_SET_TEMPERATURE_OFFSET      0x5403
#define SCD_CMD_SET_ALTITUDE                0x5102
#define SCD_CMD_SET_FORCED_RECALIBRATION    0x5204
#define SCD_CMD_AUTO_SELF_CALIBRATION       0x5306

#define CRC8_POLYNOMIAL             0x31
#define CRC8_INIT                   0xFF
#define CRC8_LEN                    1


#define SCD_START_PERIODIC_MEASUREMENT_LEN		5
#define SCD_SETMEASUREMENTINTERVAL_LEN		5
#define SCD_GETREADYSTATUS_LEN		3
#define SCD_READMEASUREMENT_LEN		18


#define SCD_WORD_LEN     2
#define SCD_COMMAND_LEN  2
#define SCD_MAX_BUFFER_WORDS 24
#define SCD_CMD_SINGLE_WORD_BUF_LEN (SCD_COMMAND_LEN + SCD_WORD_LEN + CRC8_LEN)

#define be16_to_cpu(s) (((uint16_t)(s) << 8) | (0xff & ((uint16_t)(s)) >> 8))
#define be32_to_cpu(s) (((uint32_t)be16_to_cpu(s) << 16) | \
                        (0xffff & (be16_to_cpu((s) >> 16))))


/*
*********************************************************************************************************
*                                               GLOBAL VARIABLES
*********************************************************************************************************
*/
float co2_ppm, temperature, humidity;

/*
*********************************************************************************************************
*                                               LOCALL VARIABLES
*********************************************************************************************************
*/
TypeDef_SDC30_t sdc30;
float fCo2Value, fTempValue, fHumiValue;
/*
*********************************************************************************************************
*                                               FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count);
static void scd_fill_cmd_send_buf(uint8_t *buf, uint16_t cmd, const uint16_t *args, uint8_t num_args);


uint8_t Rxbuf[128];

typedef enum{
  R_SEND_CMD = 0,
	R_WAIT_CMD ,
	R_RECEIVE_DATA_REQUEST,
	R_RECEIVED_DATA,
}Read_Step_T;
uint8_t ReadScd30_it(uint8_t* pbuf,uint16_t* pCmd,uint8_t siz)
{
	static Read_Step_T step;
	static uint8_t cmd[2];
	uint8_t rtn = false;

	
	cmd[0] = (uint8_t)((*pCmd & 0xFF00)>>8);
	cmd[1] = (uint8_t)((*pCmd & 0x00FF)>>0);
	
	switch(step)
	{
		case R_SEND_CMD:
			if(sdc30.reg.TxReady != false)
			{
				sdc30.reg.TxReady = false;
				if(HAL_I2C_Master_Transmit_IT(&hi2c2,SCD_I2C_ADDRESS<<1,cmd,2) != HAL_OK)	return rtn;
				step = R_WAIT_CMD;
			}
			break;
		case R_WAIT_CMD:
			if(sdc30.reg.TxReady == true)
			{
				step = R_RECEIVE_DATA_REQUEST;
			}
			break;
		case R_RECEIVE_DATA_REQUEST:
			if(sdc30.reg.RxReady != false)
			{
				sdc30.reg.RxReady = false;
				if(HAL_I2C_Master_Receive_IT(&hi2c2,SCD_I2C_ADDRESS<<1,pbuf,siz) != HAL_OK)	return rtn;
				step = R_RECEIVED_DATA;
			}
			break;
		case R_RECEIVED_DATA:
			if(sdc30.reg.RxReady == true)
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

uint8_t WriteScd30(uint8_t *buf, uint16_t len)
{
	uint8_t rtn = false;
	static Write_Step_T step;

	switch(step)
	{
		case W_SEND_CMD:
			if(sdc30.reg.TxReady != false)
			{
				sdc30.reg.TxReady = false;
				if(HAL_I2C_Master_Transmit_IT(&hi2c2,SCD_I2C_ADDRESS<<1,buf,len) != HAL_OK)	return rtn;
				step = W_WAIT_CMD;
			}
			break;
		case W_WAIT_CMD:
			if(sdc30.reg.TxReady == true)
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

void Init_sdc30(void)
{
  
	memset(&sdc30, 0, sizeof(TypeDef_SDC30_t));
	
	gP->sdc = &sdc30;
  sdc30.hi2c 	= &HAL_I2C2_INSTANCE;
	sdc30.write = WriteScd30;
	sdc30.read 	= ReadScd30_it;
	sdc30.reg.TxReady = true;
	sdc30.reg.RxReady = true;
	
	sdc30.info.set_interval_sec = 2;
	sdc30.info.ambient_pressure_mbar = 0;
		
	sdc30.bconnected = false;
	sdc30.dev 	= &i2c_scd30;
	sdc30.pCo2 	= &fCo2Value;
	sdc30.pTemp = &fTempValue;
	sdc30.pHumi = &fHumiValue;
	
	i2c_scd30.pRxbuf = Rxbuf;
	i2c_scd30.address = SCD_I2C_ADDRESS<<1;
 
}
static uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count)
{
	uint16_t current_byte;
	uint8_t crc = CRC8_INIT;
	uint8_t crc_bit;

	/* calculates 8-Bit checksum with given polynomial */
	for (current_byte = 0; current_byte < count; ++current_byte)
	{
		crc ^= (data[current_byte]);
		for (crc_bit = 8; crc_bit > 0; --crc_bit)
		{
			if (crc & 0x80)
			{
				crc = (crc << 1) ^ CRC8_POLYNOMIAL;
			}
			else
			{
				crc = (crc << 1);
			}
		}
	}
	return crc;
}
static void scd_fill_cmd_send_buf(uint8_t *buf, uint16_t cmd, const uint16_t *args, uint8_t num_args)
{
    uint8_t crc;
    uint8_t i;
    uint8_t idx = 0;

    buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

    for (i = 0; i < num_args; ++i){
        crc = sensirion_common_generate_crc((uint8_t *)&args[i], SCD_WORD_LEN);

        buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);
        buf[idx++] = crc;
    }
}

uint8_t u8buff[18];
void Calculate_sdc30(uint8_t *pbuf)
{
	uint32_t u32buff;
	
	memcpy(&u8buff[0],pbuf ,18);
	
  u32buff = (uint32_t)((uint32_t)u8buff[0] << 24 )  |
											((uint32_t)u8buff[1] << 16 )  |
											((uint32_t)u8buff[3] << 8 )  	|
											((uint32_t)u8buff[4]);
	
  fCo2Value    = *(float*)&u32buff;
	
  
  u32buff = (uint32_t)((uint32_t)u8buff[6] << 24 ) |
											((uint32_t)u8buff[7] << 16 ) |
											((uint32_t)u8buff[9] << 8  )	|
											((uint32_t)u8buff[10]);
  fTempValue   = *(float*)&u32buff;
  
  u32buff = (uint32_t)((uint32_t)u8buff[12] << 24 )|
											((uint32_t)u8buff[13] << 16 ) |
											((uint32_t)u8buff[15] << 8 )  |
											((uint32_t)u8buff[16]);
	fHumiValue  = *(float*)&u32buff;
}

typedef enum{
  ST_PWRON = 0,
  ST_GET_VERION,
  ST_SET_MASURE_ITV,
  ST_GET_MASURE_ITV,
  ST_TRIG_CONTINUOUS_MEASURE,
  ST_READ_READY,
  ST_READ_MEASUMENT,
  ST_CALCUL,
  ST_WORK,
  ST_NOWORK,
}Sched_t;

uint8_t get_measure_itv[32];
void HD_sdc30(void)
{
  static Sched_t status;
	static uint16_t cmd;
	static uint8_t buff[32];
  static TypeDef_Time_t sched_t;
    
	sdc30.rdRDY = HAL_GPIO_ReadPin(GPIOC, SCD30_RDY_Pin)?true:false;
    
  // Time Scheduller.... START
  if((Now()-sched_t.t)>= 1*ABS_1msec)
  {
    RdNowTime(&sched_t);
  }
  else return;
  // Time Scheduller .... END
  switch(status)
  {
    case ST_PWRON :
      status = ST_GET_VERION;
      break;      
    case ST_GET_VERION :
			cmd = SCD_CMD_GET_VERSION;
			if(sdc30.read(sdc30.info.version,&cmd,3) == true){
				if(sdc30.info.version[0] != 0x03){
					status = ST_NOWORK;
				}					 
				else{
					sdc30.bconnected = true;
        	status = ST_SET_MASURE_ITV;
				}
			}
      break;      
    case ST_SET_MASURE_ITV :
			scd_fill_cmd_send_buf(buff, SCD_CMD_SET_MEASUREMENT_INTERVAL, &sdc30.info.set_interval_sec,sizeof(sdc30.info.set_interval_sec) / SCD_WORD_LEN);
			buff[2] = 0x00;
			buff[3] = 0x02;
			buff[4] = 0xE3;
			if(sdc30.write(buff,5) == true){
				status = ST_GET_MASURE_ITV;//ST_TRIG_CONTINUOUS_MEASURE;
			}
      break;  
		case ST_GET_MASURE_ITV:
				cmd = SCD_CMD_SET_MEASUREMENT_INTERVAL;
				if(sdc30.read(sdc30.info.get_interval_sec,&cmd,3) == true){
					status = ST_TRIG_CONTINUOUS_MEASURE;
				}
			break;			
		case ST_TRIG_CONTINUOUS_MEASURE :			
			scd_fill_cmd_send_buf(buff, SCD_CMD_START_PERIODIC_MEASUREMENT, &sdc30.info.ambient_pressure_mbar,sizeof(sdc30.info.ambient_pressure_mbar) / SCD_WORD_LEN);
			if(sdc30.write(buff,SCD_START_PERIODIC_MEASUREMENT_LEN) == true)
			{
        status = ST_READ_READY;
			}
      break;      
    case ST_READ_READY :
			if(sdc30.rdRDY == true){
      	status = ST_READ_MEASUMENT;				
			}			
      break;
		case ST_READ_MEASUMENT:
			cmd = SCD_CMD_READ_MEASUREMENT;
			if(sdc30.read(buff,&cmd,SCD_READMEASUREMENT_LEN ) == true){
				status = ST_CALCUL;
			}
      break;
    case ST_CALCUL:
      Calculate_sdc30(buff);
      status = ST_READ_READY;      
      break;
		case ST_NOWORK:
			asm("NOP");
			// No Connected SCD30 Device
			break;
    default:
      status = ST_PWRON;
      break;
  }
}
TypeDef_Device_t i2c_scd30 = {
	(I2C_HandleTypeDef*)&hi2c2,	//hardware handle
	(I2C_TypeDef*)I2C2,					//hardware instance
	NULL,								//initialize function
	NULL,						//control function
	NULL,												//external interrupt callback
	NULL,							//communication interrupt callback
};