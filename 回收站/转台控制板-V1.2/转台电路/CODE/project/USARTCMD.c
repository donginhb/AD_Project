//###########################################################################
//    ������λ��������ָ��
//###########################################################################

#include <stm32f0xx.h>
#include "main.h"
#include "motordriver.h" 

void DealUsartCMD(uint8_t *buf);

//###################################Э��У�麯����###################################
/*****************************************************************
������:DealUsartCMD
��ע: ������λ�����������ܺ�������������˶�������ģʽ����ȡ��Ϣ��
******************************************************************/
void DealUsartCMD(uint8_t *buf)
{
	int32_t angle=0;
	uint16_t speed=0;
	uint8_t dir=0;
	if(buf[1]==1){//ת��ԽǶ�
		dir = buf[3];//����
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
	}else if(buf[1]==2){//ת���ԽǶ�
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
������:	UpLoadState
��ע: �ϴ�״̬�����˵�ǰ״̬��ÿ500ms��ʼ��(1�ֽ�/5ms)��Ƶ�ʷ���һ������
****************************************/
void UpLoadState(void)
{

}

