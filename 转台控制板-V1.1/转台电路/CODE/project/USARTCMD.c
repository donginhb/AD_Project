//###########################################################################
//    处理上位机发来的指令
//###########################################################################

#include <stm32f0xx.h>
#include "main.h"
#include "motordriver.h" 

void DealUsartCMD(uint8_t *buf);

//###################################协议校验函数区###################################
/*****************************************************************
函数名:DealUsartCMD
备注: 处理上位机串口命令总函数，单个电机运动、任务模式、获取信息等
******************************************************************/
void DealUsartCMD(uint8_t *buf)
{
	int32_t angle=0;
	uint16_t speed=0;
	uint8_t dir=0;
	if(buf[1]==1){//转相对角度
		dir = buf[3];//方向
		speed = (buf[4]<<8) + buf[5];//
		angle = (buf[6]<<24) + (buf[7]<<16) + (buf[8]<<8) + buf[9];
		if (buf[2] == 1){
			TurnAngle(&Motor1, speed, dir, angle);
			Motor1.motorFlag = 1;
//			printf("speed:%d\n",Motor1.motorSpeedCMD);
//			printf("dir:%d\n",Motor1.motorDirCMD);
//			printf("angle:%d\n",Motor1.detaAngleCMD);
		}
		if (buf[2] == 2){
			Motor1.motorFlag = 1;		
		}
	}else if(buf[1]==2){//转绝对角度
		dir = buf[3];
		speed = (buf[4]<<8) + buf[5];
		angle = (buf[6]<<24) + (buf[7]<<16) + (buf[8]<<8) + buf[9];
		if(dir==0)angle = -angle;
		if (buf[2] == 1){
			setAngle(&Motor1, speed, angle);
			Motor1.motorFlag = 1;
//			printf("speed:%d\n",Motor1.motorSpeedCMD);
//			printf("dir:%d\n",Motor1.motorDirCMD);
//			printf("angle:%d\n",Motor1.detaAngleCMD);
		}
		if (buf[2] == 2){
			Motor1.motorFlag = 1;		
		}
	}else if(buf[1]==0){
		resetSys(&Motor1);
	}
}

/****************************************
函数名:	UpLoadState
备注: 上传状态机器人当前状态，每500ms开始以(1字节/5ms)的频率发送一组数据
****************************************/
void UpLoadState(void)
{

}

