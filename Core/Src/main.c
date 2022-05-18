/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "DHT11.h"
#include "delay.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <includes.h>
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ????? */
#define START_TASK_PRIO		10
#define LED0_TASK_PRIO		4
#define OLED_TASK_PRIO		5
#define MSG_TASK_PRIO		  6

/* ??????	*/
#define START_STK_SIZE 		64
#define LED0_STK_SIZE 		80
#define OLED_TASK_SIZE 		64
#define MSG_STK_SIZE 		  80 //??????????,????????

/* ??? */	
CPU_STK START_TASK_STK[START_STK_SIZE];
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
CPU_STK OLED_TASK_STK[OLED_TASK_SIZE];
CPU_STK MSG_TASK_STK[MSG_STK_SIZE];
/* ????? */
OS_TCB StartTaskTCB;
OS_TCB Led0TaskTCB;
OS_TCB OLEDTASKTCB;
OS_TCB MsgTaskTCB;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ?????? */
void start_task(void *p_arg);
static  void  AppTaskCreate(void);
static  void  AppObjCreate(void);
static  void  led_pc0(void *p_arg);
static  void  oled_task(void *p_arg);
static  void  send_msg(void *p_arg);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
OS_MUTEX mutex_dht11;		
OS_MUTEX mutex_oled;		
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	OS_ERR err;
	OSInit(&err);
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_USART1_UART_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_RTC_Init();
//  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//	OLED_DISPLAY_8x16_BUFFER(0,"Hello World");
	
//	printf("OLED init\n");
//	OSTimeDlyHMSM(0, 0, 0, 100,OS_OPT_TIME_HMSM_STRICT,&err);
//	OLED0561_Init();
//	OLED_DISPLAY_CLEAR();
//	printf("oled ok\n");
//	OLED_DISPLAY_8x16_BUFFER(0,"Hello World");
//	printf("cls ok\n");
	



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  OSTaskCreate((OS_TCB     *)&StartTaskTCB,                /* Create the start task                                */
				 (CPU_CHAR   *)"start task",
				 (OS_TASK_PTR ) start_task,
				 (void       *) 0,
				 (OS_PRIO     ) START_TASK_PRIO,
				 (CPU_STK    *)&START_TASK_STK[0],
				 (CPU_STK_SIZE) START_STK_SIZE/10,
				 (CPU_STK_SIZE) START_STK_SIZE,
				 (OS_MSG_QTY  ) 0,
				 (OS_TICK     ) 0,
				 (void       *) 0,
				 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				 (OS_ERR     *)&err);
	OSStart(&err);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */


/* USER CODE BEGIN 4 */
/**
  * ????: ??????
  * ????: p_arg ?????????????
  * ? ? ?: ?
  * ?    ?:?
  */
	


	void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */

/* USER CODE BEGIN 4 */
/**
  * ????: ??????
  * ????: p_arg ?????????????
  * ? ? ?: ?
  * ?    ?:?
  */
	



void start_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
  BSP_Init();                                                   /* Initialize BSP functions */

	//CPU_Init();
  //Mem_Init();                                                 /* Initialize Memory Management Module */
	
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  		//????                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN			//?????????????
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  		//???????????
	 //???????????,??????1???????,?1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	OSMutexCreate ( &mutex_dht11,"mutex_dht11",&err);
	OSMutexCreate ( &mutex_oled,"mutex_oled",&err);
	/* ??LED0?? */
	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
				 (CPU_CHAR	* )"led_pc0", 		
                 (OS_TASK_PTR )led_pc0, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK   * )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TMR_PERIODIC,
                 (OS_ERR 	* )&err);				
	
//	OSTaskCreate((OS_TCB 	* )&OLEDTASKTCB,		
//				 (CPU_CHAR	* )"oled_task", 		
//                 (OS_TASK_PTR )oled_task, 			
//                 (void		* )0,					
//                 (OS_PRIO	  )OLED_TASK_PRIO,     
//                 (CPU_STK   * )&OLED_TASK_STK[0],	
//                 (CPU_STK_SIZE)OLED_TASK_SIZE/10,	
//                 (CPU_STK_SIZE)OLED_TASK_SIZE,		
//                 (OS_MSG_QTY  )0,					
//                 (OS_TICK	  )0,					
//                 (void   	* )0,					
//                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//                 (OS_ERR 	* )&err);		
	/* ??LED1?? */
	OSTaskCreate((OS_TCB 	* )&MsgTaskTCB,		
				 (CPU_CHAR	* )"send_msg", 		
                 (OS_TASK_PTR )send_msg, 			
                 (void		* )0,					
                 (OS_PRIO	  )MSG_TASK_PRIO,     	
                 (CPU_STK   * )&MSG_TASK_STK[0],	
                 (CPU_STK_SIZE)MSG_STK_SIZE/10,	
                 (CPU_STK_SIZE)MSG_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TMR_PERIODIC, 
                 (OS_ERR 	* )&err);
								 
}
/**
  * ????: ????????
  * ????: p_arg ?????????????
  * ? ? ?: ?
  * ?    ?:?
  */
static  void  led_pc0 (void *p_arg)
{
  OS_ERR      err;

  (void)p_arg;

  BSP_Init();                                                 /* Initialize BSP functions                             */
  CPU_Init();

  Mem_Init();                                                 /* Initialize Memory Management Module                  */

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

  CPU_IntDisMeasMaxCurReset();

  AppTaskCreate();                                            /* Create Application Tasks                             */

  AppObjCreate();                                             /* Create Application Objects                           */

  while (DEF_TRUE)
  {
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
		OSTimeDlyHMSM(0, 0, 0, 500,OS_OPT_TIME_HMSM_STRICT,&err);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
		OSTimeDlyHMSM(0, 0, 0, 500,OS_OPT_TIME_HMSM_STRICT,&err);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

static  void  oled_task (void *p_arg)
{
  OS_ERR      err;
	CPU_SR_ALLOC();
  (void)p_arg;
  BSP_Init();                                                 /* Initialize BSP functions                             */
  CPU_Init();
  Mem_Init();                                                 /* Initialize Memory Management Module                  */
	MX_GPIO_Init();
	MX_I2C1_Init();
	OSTimeDlyHMSM(0, 0, 0, 100,OS_OPT_TIME_HMSM_STRICT,&err);
	OLED0561_Init();
	OLED_DISPLAY_CLEAR();

	
#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif
  CPU_IntDisMeasMaxCurReset();
  AppTaskCreate();                                            /* Create Application Tasks                             */
  AppObjCreate();                                             /* Create Application Objects                           */
	while (DEF_TRUE)
  {	
		CPU_CRITICAL_ENTER();
		CPU_CRITICAL_EXIT();
		OSTimeDlyHMSM(0, 0, 0, 1000,OS_OPT_TIME_HMSM_STRICT,&err);
//		OLED_DISPLAY_8x16_BUFFER(0,"Hello World");
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
//		OSTimeDlyHMSM(0, 0, 0, 500,OS_OPT_TIME_HMSM_STRICT,&err);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
//		OSTimeDlyHMSM(0, 0, 0, 500,OS_OPT_TIME_HMSM_STRICT,&err);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



static  void  send_msg (void *p_arg)
{
  OS_ERR      err;
	uint8_t b[2] = {0,0};
	uint8_t rt=0;
	uint8_t re = 0;
  (void)p_arg;
  BSP_Init();                                                 /* Initialize BSP functions                             */
  CPU_Init();
  Mem_Init();	/* Initialize Memory Management Module                  */
	delay_init();
	re = DHT11_Init();	
	if(re)
	{
		printf("DHT11 Init Err!\r\n");
	}else
	{
		printf("OLED DHT11 Init Success!!\r\n");
	}
#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

  CPU_IntDisMeasMaxCurReset();

  AppTaskCreate();                                            /* Create Application Tasks                             */

  AppObjCreate();                                             /* Create Application Objects                           */

  while (DEF_TRUE)
  {
//		OSMutexPend(&mutex_dht11,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		OSTimeDly(1000,OS_OPT_TIME_DLY,&err);
//		CPU_SR_ALLOC();
//		OS_CRITICAL_ENTER();
		OSIntEnter();	
		if(DHT11_ReadData(b)==0){
			//oled?????
//			OSMutexPend(&mutex_oled,0,OS_OPT_PEND_BLOCKING,NULL,&err);	//????oled???
			printf("h:%d\r\n t:%d\r\n",b[0],b[1]);
			
//			OLED_DISPLAY_8x16(0,0,(temp/10)+'0');
//			OLED_DISPLAY_8x16(0,8,(temp%10)+'0');
//			OLED_DISPLAY_8x16(2,0,(humi/10)+'0');
//			OLED_DISPLAY_8x16(2,8,(humi%10)+'0');


						//??oled???

		}else{
		printf("Error");}
		OSIntExit();
//		OS_CRITICAL_EXIT();	
//		OSMutexPost (&mutex_oled,OS_OPT_POST_NONE,&err);			
//		
//		OSMutexPost (&mutex_dht11,OS_OPT_POST_NONE,&err);
		OSTimeDly(4000,OS_OPT_TIME_DLY,&err);
//		OSTimeDlyHMSM(0, 0, 0, 4000,OS_OPT_TIME_HMSM_STRICT,&err);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

static  void  AppTaskCreate (void)
{
  
}


/**
  * ????: uCOSIII??????
  * ????: ?
  * ? ? ?: ?
  * ?    ?:?
  */
static  void  AppObjCreate (void)
{

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
