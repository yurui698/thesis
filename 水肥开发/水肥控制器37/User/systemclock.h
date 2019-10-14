#ifndef __systemclock_H
#define __systemclock_H

#include "stm32f10x.h"

/*事件标志位宏定义*/
#define SYS_INIT_EVT        0x00000001
#define RX2_DELAY_EVT       0x00000002
#define RX2_TIMEOUT_EVT     0x00000004
#define MY_SNDBOTM_EVT	    0x00000008
#define IO_CTRL_CMD_EVT     0x00000010
#define MY_REVISION2_EVT    0x00000020
#define FLASH_CTRL_EVT      0x00000040
#define SENSOR_DATA_EVT		  0x00000080
#define MY_DELAY_EVT        0x00000100
#define RX1_DELAY_EVT       0x00000200
#define RX1_TIMEOUT_EVT     0x00000400
#define WX_CMD_EVT			    0x00000800
#define WX_RECEIVE_EVT			0x00001000
#define CHECK_EACHEVT_EVT   0x00002000
#define TX3_SEND_EVT        0x00004000
#define RX3_DELAY_EVT       0x00008000
#define RX3_TIMEOUT_EVT     0x00010000
#define PID_CTRL_CMD_EVT    0x00020000
#define MOTOR1_STOP_EVT      	0x00040000
#define MOTOR2_STOP_EVT     	0x00080000
#define MOTOR_CTR_EVT    		0x00100000

typedef struct timer_recall {
    u32 eventflag;   // 设定时间标志
    u16 timeout;     // 设定触发时间
    u16 reload_timeout;
    struct timer_recall *next;
} TimerRec;



#ifdef __cplusplus
extern "C"
{
#endif

void SysClock(void);
void GetSysTime(u32 *Day,u32 *mSec);
void TimerInit(void);
u8   Start_timerEx(u32 eventflag,u16 timeout);
u8   Start_reload_timer(u32 eventflag,u16 reload_timeout);
u8   Stop_timerEx(u32 eventflag);
u16  Get_timeoutEx(u32 eventflag);
u8   Timer_Num_Active(void);
void Set_Event(u32 eventflag);
void Clear_Event(u32 eventflag);
void Delayus(u32 dwTime);

#ifdef __cplusplus
}
#endif

#endif
