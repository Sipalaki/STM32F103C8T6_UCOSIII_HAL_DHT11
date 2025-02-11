#include "delay.h"
#include "os.h"
#include "tim.h"
#if SYSTEM_SUPPORT_OS
#include "includes.h"			
#endif

static uint8_t  fac_us=0;								
	
	
#if SYSTEM_SUPPORT_OS							

#ifdef 	OS_CRITICAL_METHOD						//OS_CRITICAL_METHOD??????,???????UCOSII				
#define delay_osrunning		OSRunning			//OS?????????,0,??????;1,??????
#define delay_ostickspersec	OS_TICKS_PER_SEC	//OS??????,??�????????
#define delay_osintnesting 	OSIntNesting		//?????????,???????????
#endif

//???UCOSIII
#ifdef 	CPU_CFG_CRITICAL_METHOD					//CPU_CFG_CRITICAL_METHOD??????,???????UCOSIII	
#define delay_osrunning		OSRunning			//OS?????????,0,??????;1,??????
#define delay_ostickspersec	OSCfg_TickRate_Hz	//OS??????,??�????????
#define delay_osintnesting 	OSIntNestingCtr		//?????????,???????????
#endif


//us??????,??????????(??????us?????)
void delay_osschedlock(void)
{
#ifdef CPU_CFG_CRITICAL_METHOD   				//'??UCOSIII
	OS_ERR err; 
	OSSchedLock(&err);							//UCOSIII?k??,??????????????us???
#else											//????UCOSII
	OSSchedLock();								//UCOSII?k??,??????????????us???
#endif
}

//us??????,??????????
void delay_osschedunlock(void)
{	
#ifdef CPU_CFG_CRITICAL_METHOD   				//'??UCOSIII
	OS_ERR err; 
	OSSchedUnlock(&err);						//UCOSIII?k??,???????
#else											//????UCOSII
	OSSchedUnlock();							//UCOSII?k??,???????
#endif
}

//????OS???????????????
//ticks:????L?????
void delay_ostimedly(u32 ticks)
{
#ifdef CPU_CFG_CRITICAL_METHOD
	OS_ERR err; 
	OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);	//UCOSIII???????????g?
#else
	OSTimeDly(ticks);							//UCOSII???
#endif 
}
 
//systick????????,'??ucos??�?
void SysTick_Handler(void)
{	
	if(delay_osrunning==1)						//OS??'????,??????????j??????
	{
		OSIntEnter();							//???????
		OSTimeTick();       					//????ucos???????????               
		OSIntExit();       	 					//????????????????
	}
}
#endif

			   
//??'????????
//??'??OS?????,????????'??OS????????
//SYSTICK????????HCLK????1/8
//SYSCLK:?????
void delay_init()
{
	fac_us = 72;
}								    

#if SYSTEM_SUPPORT_OS  							//?????????OS.
//???nus
//nus???????us??.		    								   
void delay_us(uint32_t nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;					//LOAD???	    	 
	ticks=nus*fac_us; 							//????L?????	  		 
	tcnt=0;
	delay_osschedlock();						//???OS???????????us???
	told=SysTick->VAL;        					//???????l??????
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;		//???????h??SYSTICK??h??????l????????????.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;				//?????/????????????,?????.
		}  
	};
	delay_osschedunlock();						//???OS????									    
}
//???nms
//nms:??????ms??
void delay_ms(uint16_t nms)
{	
	if(delay_osrunning&&delay_osintnesting==0)	//???OS?????????,????????????????(????????????????)	    
	{		 
		if(nms>=fac_ms)							//???????????OS????????????? 
		{ 
   			delay_ostimedly(nms/fac_ms);		//OS???
		}
		nms%=fac_ms;							//OS??????????�????????,?????????????    
	}
	delay_us((u32)(nms*1000));					//?????????  
}
#else //????OS?
//???nus
//nus???????us??.		    								   
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD?? 
	ticks=nus*fac_us; 						//?????? 
	OS_ERR err; 
	OSSchedLock(&err);					//??OS??,????us??
	told=SysTick->VAL;        				//?????????
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//??????SYSTICK?????????????.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//????/????????,???.
		}  
	};
	OSSchedUnlock(&err);					//??OS?? 
}

void Delay_us(uint16_t us)
{     
	uint16_t differ = 0xffff-us-5;				
	__HAL_TIM_SET_COUNTER(&htim1,differ);	//??TIM1??????
	HAL_TIM_Base_Start(&htim1);		//?????	
	
	while(differ < 0xffff-5)//??
	{	
		differ = __HAL_TIM_GET_COUNTER(&htim1);		//?????????
	}
	HAL_TIM_Base_Stop(&htim1);//?????
}

//???nms
//???nms?k??
//SysTick->LOAD?24??J???,????,???????:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK????Hz,nms????ms
//??72M??????,nms<=1864 
void delay_ms(uint16_t nms)
{	
	uint32_t i;
	for(i=0;i<nms;i++)
	{
		delay_us(1000);
	}
}
#endif 



