#include "systemclock.h"
#include "HandleTask.h"
#include <stdlib.h>

TimerRec *timerhead;                            //单向链表表头

static u32  Events             = 0x00000000;	//事件处理标志

static u32  sysclockday        = 0         ;
static u32  sysclockcnt        = 0         ;
static u32  sysclockcnt_old    = 0         ;

const u8 fac_us = 9;


static TimerRec *FindTimer(u32 eventflag);
static TimerRec *AddTimer(u32 eventflag,u16 timeout);
static void DeleteTimer(TimerRec *rmTimer);
static void  TimerUpdate(u32 updateTime);
static void  SysHandleEvent(void);



void SysClock(void)
{
	u32 sysclockcnt_current=0,updateTime=0;

	__set_PRIMASK(1); 
	sysclockcnt_current=SysTick->VAL;
	sysclockcnt_current=sysclockcnt_current/72000;

	if(sysclockcnt_current<sysclockcnt_old)
	{
		updateTime=sysclockcnt_old-sysclockcnt_current;
		sysclockcnt+=updateTime;		
	}

	else if(sysclockcnt_current>sysclockcnt_old)
	{
		updateTime=(0x00FFFFFF-sysclockcnt_current*72000)/72000+sysclockcnt_old;
		sysclockcnt+=updateTime;
	}
	__set_PRIMASK(0);

	if(updateTime!=0)
		TimerUpdate(updateTime);

	__set_PRIMASK(1);
	sysclockcnt_old=sysclockcnt_current;

	if(sysclockcnt>=86400000)
	{
		sysclockcnt-=86400000;
		sysclockday+=1;
	}
	__set_PRIMASK(0);

	SysHandleEvent();
	
	Events&=0x00000000;
}



void GetSysTime(u32 *Day,u32 *mSec)
{
	 *Day=sysclockday;
	 *mSec=sysclockcnt;
}



void TimerInit(void)
{
	timerhead=NULL;
}


static TimerRec *FindTimer(u32 eventflag)
{
	TimerRec *srchTimer;
	srchTimer=timerhead;

	while(srchTimer)
	{
		if(srchTimer->eventflag==eventflag)
			break;

		srchTimer=srchTimer->next;
	}

	return (srchTimer);
}



static TimerRec *AddTimer(u32 eventflag,u16 timeout)
{
	TimerRec *newTimer,*srchTimer;	
	newTimer=FindTimer(eventflag);
	
	if(newTimer)
	{
		newTimer->timeout=timeout;
		return (newTimer);
	}
	
	else
	{
		newTimer=(TimerRec *) malloc(sizeof(TimerRec));

		if(newTimer)
		{
			newTimer->eventflag=eventflag;
			newTimer->timeout=timeout;
			newTimer->reload_timeout=0;
			newTimer->next=NULL;

			if(timerhead==NULL)
				timerhead=newTimer;
	
			else
			{
				srchTimer=timerhead;
	
				while(srchTimer->next)
					srchTimer=srchTimer->next;
	
				srchTimer->next=newTimer;
			}

			return (newTimer);
		}

		else
			return ((TimerRec *) NULL);
	}	
}



static void DeleteTimer(TimerRec *rmTimer)
{
	if(rmTimer)
		rmTimer->eventflag=0;
}



u8 Start_timerEx(u32 eventflag,u16 timeout)
{
	TimerRec *newTimer;

	__set_PRIMASK(1); 
	newTimer=AddTimer(eventflag,timeout);
	__set_PRIMASK(0); 

	return ((newTimer!=NULL)? 1:0);
}



u8 Start_reload_timer(u32 eventflag,u16 reload_timeout)
{
	TimerRec *newTimer;

	__set_PRIMASK(1); 
	newTimer=AddTimer(eventflag,reload_timeout);
	if(newTimer)
		newTimer->reload_timeout=reload_timeout;
	__set_PRIMASK(0); 

	return ((newTimer!=NULL)? 1:0);
}



u8 Stop_timerEx(u32 eventflag)
{
	TimerRec *foundTimer;

	__set_PRIMASK(1); 
  	foundTimer = FindTimer(eventflag);
  	if(foundTimer)
   		DeleteTimer(foundTimer);
	__set_PRIMASK(0);
	 
	return ((foundTimer!=NULL)? 1:0);
}



u16 Get_timeoutEx(u32 eventflag)
{
	u16 rtrn=0;
	TimerRec *tmr;

	__set_PRIMASK(1); 
	tmr=FindTimer(eventflag);
	if(tmr)
		rtrn=tmr->timeout;
	__set_PRIMASK(0);
	 
	return rtrn;
}



u8 Timer_Num_Active(void)
{
	u8 num_timers=0;
	TimerRec *srchTimer;

	__set_PRIMASK(1); 
	srchTimer=timerhead;
	while(srchTimer!=NULL)
	{
		num_timers++;
		srchTimer=srchTimer->next;
	}
	__set_PRIMASK(0);
	 
	return num_timers;
}



void Set_Event(u32 eventflag)
{
	Events|=eventflag;	
}



void Clear_Event(u32 eventflag)
{
	Events&=~eventflag;
}



static void TimerUpdate(u32 updateTime)
{
  	TimerRec *srchTimer;
  	TimerRec *prevTimer;

 	__set_PRIMASK(1);
	//Look for open timer slot
  	if(timerhead!= NULL)
  	{
		//Add it to the end of the timer list
    	srchTimer=timerhead;
    	prevTimer=NULL;
		//Look for open timer slot
    	while( srchTimer)
    	{
      		TimerRec *freeTimer=NULL;
      
      		if(srchTimer->timeout<=updateTime)
        		srchTimer->timeout=0;
      		else
        		srchTimer->timeout=srchTimer->timeout-updateTime;

      		// Check for reloading
      		if((srchTimer->timeout==0)&&(srchTimer->reload_timeout)&&(srchTimer->eventflag))
      		{
  				// Notify the task of a timeout
        		Set_Event(srchTimer->eventflag);
				// Reload the timer timeout value       
        		srchTimer->timeout=srchTimer->reload_timeout;
      		}
      
      		//When timeout or delete (event_flag == 0)
      		if (srchTimer->timeout==0||srchTimer->eventflag==0)
      		{   
				//Take out of list    		
        		if (prevTimer==NULL)							   
          			timerhead=srchTimer->next;

        		else
          			prevTimer->next=srchTimer->next;
        		
				// Setup to free memory
        		freeTimer=srchTimer;
        		// Next
        		srchTimer=srchTimer->next;
     		}

     		else
      		{
        		// Get next
        		prevTimer=srchTimer;
        		srchTimer=srchTimer->next;
      		}

      		if(freeTimer)
      		{
        		if(freeTimer->timeout==0)
          			Set_Event(freeTimer->eventflag);
        		free(freeTimer);
      		}
    	}
  	}
	__set_PRIMASK(0);
}



static void SysHandleEvent(void)
{
	if(Events)
		Period_Events_Handle(Events);
    Scan_Events_Handle();
}
