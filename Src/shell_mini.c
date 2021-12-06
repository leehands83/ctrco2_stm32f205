/*
*********************************************************************************************************
*                                                SHELL_MINI.C
*
*                          (c) Copyright 2002-2011, Yongjik Park. OPENFRAME
*                                           All Rights Reserved
* Date : 2011-01-07
* File : shell_mini.c
* By   : Yongjik Park
*
*********************************************************************************************************
*/
#include "include.h"
#include "stm32f2xx_hal.h"
/*
*********************************************************************************************************
*                                               DEBUG
*********************************************************************************************************
*/
#define DBG_MSG     "SHELL"
#define DBG_EN      gP->dbg->b.on
#define DBG_LVL     0

#define CON_S_PRINTF(usb,fmt,args...)  {sprintf(cbuf,fmt,##args); con_send(cbuf, usb);}
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/
#define ARGC_HELP_CALL      255


#define USE_BITBAND_CMD     0
#define USE_MEMORY_CMD      1
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
static TypeDef_Shell_t shell ;

static TypeDef_Console_t console;
static TypeDef_Ser_t con;

static char prompt[20];
static char buffer[CFG_CONSOLE_LENGTH]={0};
static char cbuf[256];

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
/*
*********************************************************************************************************
* CONSOLE PUTCHAR
*********************************************************************************************************
*/

/*
*********************************************************************************************************
* PROMPT UPDATE
*********************************************************************************************************
*/
static
void prompt_update(void)
{
  
}
/*
*********************************************************************************************************
* VCP OR CONSOLE SEND
*********************************************************************************************************
*/
static
void con_send(char *buf, bool usb)
{
    if(!usb)
        {printf("%s", buf);}
    else
        {gP->usb->puts((uint8_t *)buf);}
}
/*
*********************************************************************************************************
* CONSOLE DEBUG
*********************************************************************************************************
*/
static
void con2dev(char buf, bool cmd, bool usb)
{
    static TypeDef_Time_t chk_t;
    static uint8_t exit_cnt;

    if(!cmd){
        exit_cnt = 0;
        RdNowTime(&chk_t);
        return;
    }

    if(buf == '@'){
        if(!exit_cnt){
            if((Now()-chk_t.t)>= 1*ABS_1000msec){
                RdNowTime(&chk_t);
                exit_cnt++;
            }
        }
        else{
            if((Now()-chk_t.t)>= 1*ABS_1000msec){
                exit_cnt = 0;
            }
            else
            if(++exit_cnt>2){
#if 0
                if(gP->dbg->b.c2gps){
                    gP->dbg->b.c2gps = FALSE;
                }
                else
                if(gP->dbg->b.c2btm){
                    gP->dbg->b.c2btm = FALSE;
                }
#endif
                CON_S_PRINTF(usb,"\n[Direct con <-> device mode exit]\n");

                return;
            }
        }
    }
    else{
        exit_cnt = 0;
        RdNowTime(&chk_t);
    }
}
/*
*********************************************************************************************************
* COMMAND PARSER FUNCTION
*********************************************************************************************************
*/
void con_cmd_parser(TypeDef_Ser_t *rx, TypeDef_Shell_t *shP, bool usb)
{
    char  buf;
    uint8_t buf_cnt=0;
    char  c_buf[CFG_ARG_LENGTH];

    shP->argc = 0;

    // ������ ����Ÿ�� �о� ���δ�.
    while(rx->in-rx->out){

        buf = rx->buf[rx->out++];

        switch(buf){
        case SPACE :    // Argument ������
            if(!buf_cnt) break; // �ƹ��͵� ���� space �� �ִٸ� ������ �о� ���δ�

            if(buf_cnt<CFG_ARG_LENGTH)  c_buf[buf_cnt++] = '\0';   // ���ڿ��� ������ �˸��� ����

            if(shP->argc<CFG_ARG_CNT)  strcpy(shP->argv[shP->argc++], c_buf);

//printfshell.argv[shell.argc-1]);
            buf_cnt = 0;
            break;

        case '\0'  :    // ��� ��ü�� ����
            if(!buf_cnt && !shP->argc) return; // �˼����� �Է��̶�� ����

            if(buf_cnt<CFG_ARG_LENGTH)  c_buf[buf_cnt++] = '\0';   // ���ڿ��� ������ �˸��� ����

            if(shP->argc<CFG_ARG_CNT)  strcpy(shP->argv[shP->argc++], c_buf);
//printf(shell.argv[shell.argc-1]);

            if(!usb)
                shP->event = true;  // ��� �Է� �̺�Ʈ �߻� �÷��� ��
            else
                gP->usb->sh->event = true;

            /////////////////////////////////////////////////////////////////////////////
            // ���ó�� �Լ��� ȣ���Ϸ��� ���⼭ ���Ѵ�. ������ Call Depth�� ��� ����.//
            /////////////////////////////////////////////////////////////////////////////
            break;

        default    :
            if(buf_cnt<CFG_ARG_LENGTH) c_buf[buf_cnt++] = buf;
            break;
        }
    }
}
/*
*********************************************************************************************************
* CONSOLE INIT FUNCTION
*********************************************************************************************************
*/
void console_Init(void)
{
    con.in  = 0;
    con.out = 0;
    con.buf = buffer;

    prompt_update();  // �⺻ console set

    ////////////////////////////////////////////
    gP->console       = &console;

    console.prompt    = prompt;
    console.printf    = printf;
    console.con2dev   = con2dev;

    console.prompt_update = prompt_update;
    console.sh        = &shell;
}
/*
*********************************************************************************************************
* CONSOLE CONTROL FUNCTION
* Arguments  : struct shell_t *
* Returns    : void
*********************************************************************************************************
*/
void console_control(void)
{
  //console_receive();

  if(shell.event)
    con_cmd_execute(&shell, 0);

  return;
}
/*
*********************************************************************************************************
*                                                   END
*********************************************************************************************************
*/