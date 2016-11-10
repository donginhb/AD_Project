/******************** (C) COPYRIGHT 2009 Chen Yao ******************************
* File Name          : STM32_DevInit.c
* Author             : Chen Yao
* Version            : V1.0
* Date               : 11/6/2008
* Description        : STM32 Hardware Init
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "main.h"
#include "motordriver.h" 

/* Private typedef -----------------------------------------------------------*/
//DMA_InitTypeDef	DMA_InitStructure;
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address	0x40012440			//ADC数据地址，给DMA用
#define DAC_DHR12R1		((u32)0x40007408)		//DAC数据地址，给DMA用


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SysTick_Configuration(void);

//#ifdef __STM32F0XX_ADC_H 
//void ADC_Configuration(void);
//#endif

//#ifdef __STM32F0XX_DAC_H
//void DAC_Configuration(void);
//#endif

//#ifdef __STM32F0XX_DMA_H
//void DMA_Configuration(void);
//#endif

//#ifdef __STM32F0XX_EXTI_H
//void EXTI_Configuration(void);
//#endif

void GPIO_Configuration(void);

//#ifdef __STM32F0XX_I2C_H
//void I2C_Configuration(void);
//#endif

void NVIC_Configuration(void);

//#ifdef __STM32F0XX_SPI_H
//void SPI_Configuration(void);
//#endif

void TIM_Configuration(void);

void USART_Configuration(void);

//#ifdef __STM32F0XX_WWDG_H
//void WDG_Configuration(void);                //看门狗初始化
//#endif
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : STM32_DevInit
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STM32_DevInit(void)
{
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_TIM1           
							|RCC_APB2Periph_SYSCFG         
//							|RCC_APB2Periph_USART6         
//							|RCC_APB2Periph_USART7         
//							|RCC_APB2Periph_USART8         
							|RCC_APB2Periph_ADC1           
//							|RCC_APB2Periph_SPI1           
							|RCC_APB2Periph_USART1         
							|RCC_APB2Periph_TIM15          
//							|RCC_APB2Periph_DBGMCU        
							|RCC_APB2Periph_TIM16          
							|RCC_APB2Periph_TIM17,	ENABLE);          

	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_TIM3   
//							|RCC_APB1Periph_TIM6   
							|RCC_APB1Periph_TIM14  
							|RCC_APB1Periph_WWDG   
//							|RCC_APB1Periph_SPI2   
//							|RCC_APB1Periph_USART2 
							|RCC_APB1Periph_I2C1,	ENABLE);   
//							|RCC_APB1Periph_I2C2   
//							|RCC_APB1Periph_PWR

	RCC_AHBPeriphClockCmd(	RCC_AHBPeriph_GPIOA 
							|RCC_AHBPeriph_GPIOB  
							|RCC_AHBPeriph_GPIOC
//							|RCC_AHBPeriph_GPIOD 
							|RCC_AHBPeriph_GPIOF 
//							|RCC_AHBPeriph_TS    
//							|RCC_AHBPeriph_CRC   
//							|RCC_AHBPeriph_FLITF 
//							|RCC_AHBPeriph_SRAM  
							|RCC_AHBPeriph_DMA1 , 	ENABLE);   
//							|RCC_AHBPeriph_DMA2
	SysTick_Configuration();
	


	USART_Configuration();

//	EXTI_Configuration();

	NVIC_Configuration();

	TIM_Configuration();
	GPIO_Configuration();

//#ifdef __STM32F0XX_WWDG_H
////	 WDG_Configuration();                //看门狗初始化
//#endif
}
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	
/*// 在开发板上用来驱动LED		IO*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_13;                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	
	//DIR OUT  PA1:DIR_1 PA2:DIR_2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
		//Reset	IN	PA3: RES_1   PA3: RES_1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 	
	//PWM OUT 			PA6/7			AF
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//UART1  			 			PA9/10					AF
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_0);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configure the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
  /*NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);*/
	 /* Enable the USART Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
//	 /* Enable the TIM16 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	////	 /* Enable the TIM14 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM15_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//	 /* Enable the TIM17 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

/* Enable and set EXTI4_15 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	

}

/*******************************************************************************
* Function Name  : SysTick_Configuration
* Description    : Configure the SysTick 0.05ms次中断
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Configuration(void)
{
	if (SysTick_Config(SYSTICKCOUNT))
	{ 
		while (1);
	}
}

/*******************************************************************************
* Function Name  : TIM_Configuration
* Description    : Configure the TIM
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//通用定时器设置==============================================================
	// Time Base configuration  
//	TIM_TimeBaseStructure.TIM_Prescaler = 36;		// 1M
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
///	TIM_TimeBaseStructure.TIM_Period = 5000;		// 5ms
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//	TIM_Cmd(TIM2, ENABLE);   


// 编码器捕捉模式TIM1 TIM3===========================================================================
/* Timer configuration in Encoder mode */
	 TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

//	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 240;			//0.5ms  计数100
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//	
//	/* Prescaler configuration */
//	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
//	
//	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = 3;
//	TIM_ICInit(TIM1, &TIM_ICInitStructure);	 
//	TIM_ICInit(TIM3, &TIM_ICInitStructure);	 
//  
//	/* TIM Interrupts enable */
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//	TIM_SetCounter(TIM1, ENCODER_CON_RES);
//	TIM_SetCounter(TIM3, ENCODER_CON_RES);
//	/* TIM3 enable counter */
	TIM_Cmd(TIM1, ENABLE);
//	TIM_Cmd(TIM3, ENABLE);

// PWM模式===========================================================================
// Time base configuration
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;				//		x		us一个单位
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_Period = PERIOD<<1;					// 		x		k频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM15, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
  	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM15, &TIM_OCInitStructure);
//	TIM_OC1PreloadConfig(TIM15, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM15, &TIM_OCInitStructure);
//	TIM_OC2PreloadConfig(TIM15, TIM_OCPreload_Enable);	
	
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
//	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);	

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM16, &TIM_OCInitStructure);
//	TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM17, &TIM_OCInitStructure);
//	TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM14, ENABLE);	
	TIM_ARRPreloadConfig(TIM15, ENABLE);	
	TIM_ARRPreloadConfig(TIM16, ENABLE);	
	TIM_ARRPreloadConfig(TIM17, ENABLE);
	
//	TIM_ITConfig(TIM15, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM17, TIM_IT_CC1, ENABLE);
	
//	TIM_CtrlPWMOutputs(TIM15, ENABLE);
//	TIM_CtrlPWMOutputs(TIM16, ENABLE);
//	TIM_CtrlPWMOutputs(TIM17, ENABLE);

	TIM_SetCompare1(TIM15, PERIOD);
	TIM_SetCompare1(TIM14, PERIOD);
	TIM_SetCompare1(TIM16, PERIOD);
	TIM_SetCompare1(TIM17, PERIOD);

	TIM_Cmd(TIM14, ENABLE);
	TIM_Cmd(TIM15, ENABLE);
	TIM_Cmd(TIM16, ENABLE);
	TIM_Cmd(TIM17, ENABLE);
	
}

/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configures USART
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;				 			//9600波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//数据位为8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;				//停止位为1
	USART_InitStructure.USART_Parity = USART_Parity_No;					//没有奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 	//硬件流控制失能 ???
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//发送使能接受使能

	USART_Init(USART1, &USART_InitStructure);				//初始化USART1
//	USART_Init(USART2, &USART_InitStructure);				//初始化USART2
//	USART_Init(USART3, &USART_InitStructure);				//初始化USART3
//	USART_Init(UART4, &USART_InitStructure);				//初始化USART1
//	USART_Init(UART5, &USART_InitStructure);				//初始化USART1

// Enable USART Receive and Transmit interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启接受中断
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//开启接受中断
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			//开启接受中断
//	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);			//开启接受中断
//	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);			//开启接受中断
  
// Enable the USART
	USART_Cmd(USART1, ENABLE);
//	USART_Cmd(USART2, ENABLE);		//485
//	USART_Cmd(USART3, ENABLE);		//GPS
//	USART_Cmd(UART4, ENABLE);		//电机驱动
//	USART_Cmd(UART5, ENABLE);		//上位机通信，接在视频服务器上
}


/******************* (C) COPYRIGHT 2008 CY *****END OF FILE****/
