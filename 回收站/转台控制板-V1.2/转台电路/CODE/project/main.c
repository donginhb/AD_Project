/**
  ******************************************************************************
  * @file    main.c 
  * @author  chenyao
  * @version V1.0
  * @date    2014.11.3
  * @brief   Main program body.      2015.10.13 最终版本
  ******************************************************************************
*/ 


/* Include---------------------------------------------------------------*/

#include "stm32f0xx.h"
#include "USARTCMD.h"
#include "main.h"
#include "motordriver.h" 

/****************************************************
	函数名:	fputc
	功能: 	printf重定向
	作者:		liyao 2015年9月8日14:10:51
****************************************************/
int fputc(int ch,FILE *f);


int fputc(int ch,FILE *f)
{
		USART_ClearFlag(USART1, USART_FLAG_TC);
		USART_SendData(USART1, ch);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET) ;
}


volatile uint32_t timeTick = 0;

uint32_t timeTickLast=0;
uint32_t timerLast=0;


typedef void (* FUNtype)(void);

typedef struct{
	uint32_t timer_div_last;	
	void (*handle)(void);
}FUNCTION;


void Task(uint32_t timer, uint32_t interval, FUNCTION *func){
	uint32_t timer_mod = 0;
	uint32_t timer_div = 0;
	timer_mod = timer % interval;
	timer_div = timer / interval;

	if((timer_mod < (interval>>2)) && func->timer_div_last != timer_div){
		func->timer_div_last = timer_div;
		func->handle();
	}
}

void ExeMotor1(void){
	MotorRunning(&Motor1);
}
void DealKey(MOTOR_PARM* motor){
	uint8_t PinStatus = 0;
	PinStatus = GPIO_ReadInputDataBit(motor->GPIO_port, motor->GPIO_Pin);
//	printf("PinStatus:%d\n",PinStatus);
	if(PinStatus == 1)motor->angle=0;
}
/* Private function prototypes -----------------------------------------------*/

extern void STM32_DevInit(void);

FUNCTION motor_func;
FUNCTION DealUsartCMD_func;
FUNCTION UpLoadState_func;
FUNCTION Buff_To_Uart_func;

int main(void)
{
	uint32_t i=0;
//驱动板初始化
	STM32_DevInit();
	//系统初始化
	MotorInit();//电机初始化	
	Motor1.motorFlag = 0;
//Motor1.motorDirCMD = 0;
	Motor1.motorSpeedCMD = 1000;
	Motor1.detaAngleCMD = 0;
//	TurnAngle(&Motor1, 10000, 0, 1000000);
	motor_func.handle = ExeMotor1;
	motor_func.timer_div_last = 0;	
	while(1){
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
		GPIO_SetBits(GPIOA, GPIO_Pin_6);
		GPIO_SetBits(GPIOA, GPIO_Pin_7);
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
		GPIO_ResetBits(GPIOA, GPIO_Pin_6);
		GPIO_ResetBits(GPIOA, GPIO_Pin_7);
		/*
	while(1){	  
	
		
	}	
	
	
	*/
	}
	while(1){
		for(i=0;i<48000000*0.1;i++);
		DealKey(&Motor1);
		DealKey(&Motor2);
		//TIM_CtrlPWMOutputs(TIM15, DISABLE);		
		printf("DirCMD:%d\n",Motor1.motorDirCMD);
		printf("angleCMD:%d\n",Motor1.angleCMD);
		printf("Angle:%d\n",Motor1.angle);
		printf("speed%d\n",Motor1.motorSpeedCMD);
		
		printf("DirCMD:%d\n",Motor2.motorDirCMD);
		printf("angleCMD:%d\n",Motor2.angleCMD);
		printf("Angle:%d\n",Motor2.angle);
		printf("speed%d\n",Motor2.motorSpeedCMD);
		//for(i=0;i<48000000*0.1;i++);

		//TIM_CtrlPWMOutputs(TIM15, ENABLE);
//		Task(timeTick, Motor1.motorSpeedCMD, &motor_func);	
//		Task(timeTick, Motor1.motorSpeedCMD, &motor_func);	
		
//		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//		USART_ClearFlag(USART1, USART_FLAG_TC); 
//		USART_SendData(USART1,0xfd);
		
	}	
}
 

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

