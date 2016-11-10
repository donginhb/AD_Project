#include <stm32f0xx.h>
#include "motordriver.h" 
#include <stdio.h>

void MotorRunning(MOTOR_PARM* motor);

MOTOR_PARM Motor1;
MOTOR_PARM Motor2;

void MotorInit(void);
void MotorReset(MOTOR_PARM* motor);
void setAngle(MOTOR_PARM* motor, uint16_t speed, uint32_t angle);
void setSpeed(MOTOR_PARM* motor, uint16_t speed, uint8_t dir);
void TurnAngle(MOTOR_PARM* motor, uint16_t speed, uint8_t dir, uint32_t angle);
void resetSys(MOTOR_PARM* motor);

void MotorInit(){
	//Motor1
	MotorReset(&Motor1);
	Motor1.motorID = 1;
	Motor1.GPIO_port = GPIOA;
	Motor1.GPIO_Pin = GPIO_Pin_3;
	Motor1.Dir_Pin = GPIO_Pin_1;	
	Motor1.Timer = TIM16;
	//Motor2
	MotorReset(&Motor2);
	Motor2.motorID = 2;
	Motor2.GPIO_port = GPIOA;
	Motor2.GPIO_Pin = GPIO_Pin_5;
	Motor2.Dir_Pin = GPIO_Pin_2;	
	Motor2.Timer = TIM17;
}

void MotorRunning(MOTOR_PARM* motor){	
	if(motor->motorFlag){
			if(motor->detaAngle == motor->detaAngleCMD){
				motor->detaAngle = 0;
				motor->detaAngleCMD = 0;
				motor->motorFlag = 0;
				TIM_CtrlPWMOutputs(motor->Timer, DISABLE);
				printf("over,angle:%d\n",(motor->angle));
			}else{
				TIM_CtrlPWMOutputs(motor->Timer, ENABLE);				
				//速度脉冲控制
				TIM_SetCounter(motor->Timer,(PERIOD + motor->motorSpeedCMD));				
				TIM_SetCompare1(motor->Timer, (PERIOD - motor->motorSpeedCMD));
				//方向控制		
				if(motor->motorDirCMD==2){
					GPIO_SetBits(motor->GPIO_port, motor->Dir_Pin);
					motor->angle++;
				}else if(motor->motorDirCMD==0){
					GPIO_ResetBits(motor->GPIO_port, motor->Dir_Pin);
					motor->angle--;
				}
				motor->detaAngle++;
		}
	}
}

void MotorReset(MOTOR_PARM* motor)
{

	motor->motorFlag = 0;
	motor->motorDir=0;			//电机当前速度
	motor->motorSpeed=0;			//当前速度
	motor->motorDirCMD=0;		//此处存有上位机发来的方向指令，1和2
	motor->motorSpeedCMD=0;		//此处存有上位机发来未执行的速度	0-100		复位为0xff，在启动电机前先要稳定DIR一段时间
	
//	motor->angle=ANGLEZERO;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
//	motor->angleLast=ANGLEZERO;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
//	motor->angleCMD=ANGLEZERO;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	motor->angle=0;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	motor->angleLast=0;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	motor->angleCMD=0;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	motor->detaAngleCMD=0;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	motor->detaAngle=0;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值	
	TIM_CtrlPWMOutputs(motor->Timer, DISABLE);
}

void setAngle(MOTOR_PARM* motor, uint16_t speed, uint32_t angle){	
	uint8_t dir = 0;
	int32_t deltaAngle = 0;
	motor->angleCMD = angle;
	deltaAngle = motor->angleCMD - motor->angle;
	if(deltaAngle>0){
		dir = 2;
	}else{
		deltaAngle = -deltaAngle;
		dir = 0;
	}
	TurnAngle(motor, speed, dir, deltaAngle);
}

void resetSys(MOTOR_PARM* motor){	
	motor->motorSpeedCMD = 0;
	motor->motorDirCMD = 0;
	motor->motorDir = 0;
	motor->detaAngleCMD = 0;
	motor->detaAngle = 0;
	motor->angle = 0;
	motor->angleCMD = 0;
	MotorReset(motor);
}

void setSpeed(MOTOR_PARM* motor, uint16_t speed, uint8_t dir){
	motor->motorSpeedCMD = speed;
	motor->motorDirCMD = dir;
	motor->detaAngleCMD = 0xffffffff;
}

void TurnAngle(MOTOR_PARM* motor, uint16_t speed, uint8_t dir, uint32_t angle){
	if(speed>PERIOD)speed=PERIOD;
	motor->motorSpeedCMD = speed;
	motor->motorDirCMD = dir;
	motor->detaAngleCMD = angle;	
	motor->detaAngle = 0;	
}
