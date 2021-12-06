/*
*********************************************************************************************************
*                                     DATA TYPE DEFINE
*
*                          (c) Copyright 2019-2019, Namhun LEE.
*                                           All Rights Reserved
* File : DataType.h
* By   : Namhun, LEE
*********************************************************************************************************
*/
#ifndef __DATATYPES_H__
#define __DATATYPES_H__


#include "hw_config.h"

/*
*********************************************************************************************************
*	Data Type 정의
*********************************************************************************************************
*/
#ifndef __BOOL__
#define __BOOL__
typedef	bool		BOOL;
#endif
#ifndef __BYTE__
#define __BYTE__
typedef uint8_t 	BYTE;
typedef uint8_t	 	uint8;
#endif
#ifndef __WORD__
#define __WORD__
typedef uint16_t	WORD;
typedef uint16_t	uint16;
#endif
#ifndef __DWORD__
#define __DWORD__
typedef uint32_t  	DWORD;
#endif
#ifndef __UINT8__
#define __UINT8__
typedef uint8_t   	UINT8;
#endif
#ifndef __UINT16__
#define __UINT16__
typedef uint16_t   UINT16;
#endif
#ifndef __UINT32__
#define __UINT32__
typedef uint32_t   UINT32;
#endif
#ifndef __INT8__
#define __INT8__
typedef int8_t		INT8;
#endif
#ifndef __INT16__
#define __INT16__
typedef int16_t    INT16;
#endif
#ifndef __INT32__
#define __INT32__
typedef int32_t	INT32;
#endif

#ifndef __UINT__
#define __UINT__
typedef uint16_t   UINT;
#endif

#define TRUE        true
#define FALSE       false

#define TRUE        true
#define FALSE       false

// 시간
typedef struct time_t{
    BOOL    s;     // timer store flag
    INT32   t;     // stored time
}TypeDef_Time_t;

typedef struct I2C_REG_t{
  uint8_t RxReady , TxReady;
  uint8_t rx_buf[10];
  uint8_t rx_len;
  uint8_t tx_buf[10];
}TypeDef_I2C_REG_t;

// FOR HDC1080
typedef struct HDC1080_t{
  TypeDef_I2C_REG_t reg;
  I2C_HandleTypeDef *hi2c;
	
	uint8_t *pMfID, *pDbID;
	float temperature;
  float humidity;

  void(*init)(void);
  uint8_t(*write)(uint8_t *buf, uint16_t len);
  uint8_t(*read)(uint8_t* pbuf,uint8_t* cmd,uint8_t siz);

}TypeDef_HDC1080_t;

typedef struct {
  char *name;           // name
  void (*func)(void);   // functions

}TypeDef_Switch_t;

typedef struct AS1115_t{

  uint8_t mod;
  uint8_t ic_sel;
  uint8_t input;
  uint8_t fan_speed;


  uint8_t dis_mode;

  bool key_event;
  uint32_t acs_err;
  uint8_t Keyscan[2];
  TypeDef_I2C_REG_t reg;

  void (*init)(void);
  uint8_t (*write)(uint8_t *buf, uint16_t len);
  uint8_t(*read)(uint8_t* pbuf,uint8_t* pCmd,uint8_t siz);
  uint8_t (*set)(void);

  I2C_HandleTypeDef *hi2c;

}TypeDef_AS1115_t;



// USB VCP

// For Serial Comm
// Console 을 위해
typedef struct console_t{
    uint8_t rbuf_hal;
    char *prompt;
    struct shell_t *sh;
    void (*prompt_update)(void);
    void (*comm_init)(uint32_t);
    void (*putstr)(uint8_t*);
    int  (*printf)(const char *_Restrict, ...);
    void (*con2dev)(char, bool, bool);
}TypeDef_Console_t;

typedef union debug_t {
  volatile uint32_t d32;
  struct
  {
  uint32_t    on:      1;
  uint32_t    lvl:     4;
  uint32_t    echo:    1;
  uint32_t    prompt:  1;
  uint32_t    bkp:     1;
  uint32_t    key:     1;

  uint32_t    usb:     1;
  uint32_t    vcp:     1;
  uint32_t    led:     1;
  }
  b;
}TypeDef_Dbg_t;

// led를 위해
typedef struct led_t{

  bool    en;     // enable flag
  uint32_t stat1_btime;
  uint32_t stat2_btime;
  uint32_t stat3_btime;

  void (*btime_set)(uint32_t, uint32_t);

}TypeDef_Led_t;



typedef struct __Device_t
{
	void *Handle;           // hardware Handle   type
	void *Instance;         // Instance type

	void (*init)(void);     			// device init
	void (*ctrl)(void);     			// device control
	void (*irq_cb_evt)(void); 		// irq callback event
	void (*irq_cb_rx)(uint8_t);   // irq callback receive data

  uint8_t *pRxbuf;
	uint8_t *pTxbuf;

	bool bRx_ok, bTx_ok;
	uint8_t uRx_len, uTx_len;
	uint8_t uRx_idx, uTx_idx;
	uint32_t uEvt_wt_over, uNum_wt_over;

	uint32_t address;
}TypeDef_Device_t;

typedef struct I2C_t{
	TypeDef_Device_t *dev;
}TypeDef_I2C_t;

typedef struct SCD30_info_t{
	uint16 set_interval_sec;
	uint8 get_interval_sec[3];
	uint16 ambient_pressure_mbar;
	uint8 version[3];
	
}TypeDef_SCD30_info_t;
// FOR SDC30
typedef struct SDC30_t{
  TypeDef_I2C_REG_t reg;
	TypeDef_SCD30_info_t info;
	I2C_HandleTypeDef *hi2c;
		
  uint8_t(*write)(uint8_t *buf, uint16_t len);
  uint8_t(*read)(uint8_t* pbuf,uint16_t* cmd,uint8_t siz);
	
	uint8_t bconnected;
  uint8_t buff[18];
  float *pCo2;
  float *pTemp;
  float *pHumi;
  bool rdRDY;
  //uint8_t rdPWM;  // Not support PWM

  void (*init)(void);
	TypeDef_Device_t* dev;
}TypeDef_SDC30_t;
typedef struct global_t
{
	int gTickTime;
	uint8_t uDeviceMode;
	
  TypeDef_HDC1080_t *hdc;
  TypeDef_SDC30_t *sdc;
  TypeDef_AS1115_t *as1;

  TypeDef_Console_t *console;
  TypeDef_Dbg_t     *dbg;
  TypeDef_Led_t     *led;

	TypeDef_I2C_t	*i2c;

}TypeDef_Global_t;


typedef struct ser_t{
    UINT32 in;
    UINT32 out;
    char *buf;
}TypeDef_Ser_t;
typedef struct shell_t{
    BOOL    event;
    UINT8   argc;
    char    argv[CFG_ARG_CNT][CFG_ARG_LENGTH];
}TypeDef_Shell_t;

typedef struct cmd_list{            /* cmd_table[] entries prototype    */
	char *cmd;
	INT8 (*func)(TypeDef_Shell_t*);
	char *help;
	UINT8  arg_cnt;

}CMD_LIST;



#endif //__DATATYPES_H__
