/******************** (C) COPYRIGHT 2009 Chen Yao ******************************
* File Name          : main.h
* Author             : Chen Yao
* Version            : V1.0
* Date               : 05/02/2009
* Description        : This file contains the headers of main.c
********************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f0xx.h"
#include <stdio.h>

#define SYSTICK_STEP 5 //��λ��US  ��SysTick_Handler�ж�ִ�м����
#define HZ 1000000 / SYSTICK_STEP //Ƶ��
#define SYSTICKCOUNT (SystemCoreClock/(HZ)) //SysTick_Config(SYSTICKCOUNT)


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern volatile uint32_t timeTick;
extern uint32_t timeTickLast;
extern int fputc(int ch,FILE *f);

/* Private function prototypes -----------------------------------------------*/

#endif
