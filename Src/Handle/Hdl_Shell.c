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
#include "usbd_cdc_if.h"
#include "main.h"

//#include "stm32f2xx_hal_pcd.h"

#include <stdarg.h>   /* 가변인자 처리 */
#include <string.h>   /* 문자열 비교   */


#define DBG_MSG     "HDL_SHELL"
#define DBG_LVL     2

#define PAD_RIGHT 	1
#define PAD_ZERO 		2

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

#define WRITE_PROTOTYPE	int _write(int32_t file, uint8_t *ptr, int32_t len)

uint8_t CDC_Input(uint8_t buf);	
static char   buffer[CFG_CONSOLE_LENGTH]={0};
static struct ser_t con;
void ChkReceiveData(USBD_HandleTypeDef *pdev);
extern USBD_HandleTypeDef hUsbDeviceHS;

int DBG_PRINTF(uint8_t level, const char *MSG, const char *format, ...);
static int print(char **out, const char *format, va_list args );

uint8_t cdcText[1000];
uint8_t* pcdcText = &cdcText[0];

typedef enum{
  ST_PWRON = 0,
  ST_INIT,
  ST_WORK,
}Sched_t;

void HD_Shell(void)
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
			con.in  = 0;
			con.out = 0;
			con.buf = buffer;
	
//			printf("***************************** \n\r");
//			printf("** Welcon to CTRco2 System ** \n\r");
//			printf("***************************** \n\r");
      status = ST_WORK;
      break;
    case ST_WORK:
					
			//ChkReceiveData(&hUsbDeviceHS);
			
			//DBG_PRINTF(DBG_LVL, DBG_MSG, " TEST \r\n",__FUNCTION__);
			break;
    default:
      status = ST_PWRON;
      break;
  }
}
void ChkReceiveData(USBD_HandleTypeDef *pdev)
{
	//pdev->pClassData->
}

int DBG_PRINTF(uint8_t level, const char *MSG, const char *format, ...)
{
	if(gP->dbg->b.lvl > level)	return 1;

	printf("[%010ld:%s] ",gP->gTickTime, MSG);
	printf(">");

	va_list args;
	va_start( args, format );

	return print( 0, format, args );
}

static void printchar(char **str, int c)
{
	extern int putchar(int c);

	if(str)
	{
		**str = c;
		++(*str);
	}
	else
	{
		(void)putchar(c);
  }
}
static int prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';

	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		printchar (out, padchar);
		++pc;
	}

	return pc;
}
#define PRINT_BUF_LEN 12

static
int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			printchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + prints (out, s, width, pad);
}

static int print(char **out, const char *format, va_list args )
{
	register int width, pad;
	register int pc = 0;
	
	char scr[2];

	for (; *format != 0; ++format){
		if (*format == '%'){
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				register char *s = (char *)va_arg( args, int );
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, va_arg( args, int ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (out, va_arg( args, int ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
            if( *format == 'p' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
		}
		else {
		out:
			printchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}

PUTCHAR_PROTOTYPE
{
	uint8_t temp;
	temp = (uint8_t)ch;
	while(CDC_Transmit_HS(&temp,1) == USBD_BUSY);
	return ch;
}




uint8_t CDC_Input(uint8_t buf)
{
	if(buf == ESC)
	{
  	return (USBD_OK);
	}
	switch(buf)
	{
		case BS:
			if(con.in)
			{
				con.buf[--con.in] = ' ';
				//printf((char*)&buf);
				//printf(" ");
				//printf((char*)&buf);
			}
			break;
			
		case LF:
			//printf(buf);
			break;
			
		case CR_F:
			//printf(buf);
			break;
			
		default:
			if(con.in < CFG_CONSOLE_LENGTH - 1)
			{
				con.buf[con.in++] = buf;
				printf((char*)&buf);
			}
			break;
	}
	return USBD_OK;
}

static
INT8 do_reset(struct shell_t *p)
{
	UINT8 rtn = NO_ERROR;
	printf("%s > Now system s/w reset !!!\n", __FUNCTION__);
	HAL_Delay(500U);
	NVIC_SystemReset();
	return rtn;
}


/*
*********************************************************************************************************
* COMMANDS
*********************************************************************************************************
*/
static CMD_LIST cmd_tbl[]={
	{"res",  do_reset,      	"s/w reset"},
	NULL,
};