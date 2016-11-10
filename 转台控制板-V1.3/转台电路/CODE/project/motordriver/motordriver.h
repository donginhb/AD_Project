#include <stm32f0xx.h>
#define ANGLEZERO 0x7fffffff
/*******************************************
�ṹ�嶨��
*******************************************/
//typedef struct _MOTOR_PM  MOTOR_PARM;
typedef struct 						//����Ĳ���������Ҫ����һ�����õ�
{
	uint8_t	motorID;			//���ID 1-2	
	
	uint8_t   motorFlag;
	uint8_t		motorDir;			//�����ǰ�ٶ�
	uint32_t	motorSpeed;			//��ǰ�ٶ�
	uint8_t		motorDirCMD;		//�˴�������λ�������ķ���ָ�1��2
	uint32_t	motorSpeedCMD;		//�˴�������λ������δִ�е��ٶ�	0-100		��λΪ0xff�����������ǰ��Ҫ�ȶ�DIRһ��ʱ��
	uint32_t	angleEncode;			//��ʼ����0x7fffffff���ٴ˻��������Ӽ�����ÿ���ϴ���ָ���ʼֵ
	
	int32_t	angle;			//��ʼ����0���ٴ˻��������Ӽ�����ÿ���ϴ���ָ���ʼֵ
	int32_t	angleLast;			//��ʼ����0x7fffffff���ٴ˻��������Ӽ�����ÿ���ϴ���ָ���ʼֵ
	int32_t	angleCMD;			//��ʼ����0x7fffffff���ٴ˻��������Ӽ�����ÿ���ϴ���ָ���ʼֵ
	uint32_t	detaAngleCMD;			//��ʼ����0x7fffffff���ٴ˻��������Ӽ�����ÿ���ϴ���ָ���ʼֵ
	uint32_t	detaAngle;			//��ʼ����0x7fffffff���ٴ˻��������Ӽ�����ÿ���ϴ���ָ���ʼֵ
	
	uint16_t  Dir_Pin;
	uint16_t  GPIO_Pin;//��λ����

	GPIO_TypeDef  *GPIO_port;
	TIM_TypeDef   *Timer;
	
}MOTOR_PARM;
#define PERIOD 20000

extern MOTOR_PARM Motor1;
extern MOTOR_PARM Motor2;
extern void MotorInit(void);
extern void setAngle(MOTOR_PARM* motor, uint16_t speed, uint32_t angle);
extern void setSpeed(MOTOR_PARM* motor, uint16_t speed, uint8_t dir);
extern void TurnAngle(MOTOR_PARM* motor, uint16_t speed, uint8_t dir, uint32_t angle);
extern void MotorRunning(MOTOR_PARM* motor);
extern void resetSys(MOTOR_PARM* motor);

