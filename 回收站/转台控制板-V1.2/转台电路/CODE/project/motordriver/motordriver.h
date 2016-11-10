#include <stm32f0xx.h>
#define ANGLEZERO 0x7fffffff
/*******************************************
结构体定义
*******************************************/
//typedef struct _MOTOR_PM  MOTOR_PARM;
typedef struct 						//里面的参数根据需要，不一定都用到
{
	uint8_t	motorID;			//电机ID 1-2	
	
	uint8_t   motorFlag;
	uint8_t		motorDir;			//电机当前速度
	uint32_t	motorSpeed;			//当前速度
	uint8_t		motorDirCMD;		//此处存有上位机发来的方向指令，1和2
	uint32_t	motorSpeedCMD;		//此处存有上位机发来未执行的速度	0-100		复位为0xff，在启动电机前先要稳定DIR一段时间
	uint32_t	angleEncode;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	
	int32_t	angle;			//初始数据0，再此基础上做加减，在每次上传后恢复初始值
	int32_t	angleLast;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	int32_t	angleCMD;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	uint32_t	detaAngleCMD;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	uint32_t	detaAngle;			//初始数据0x7fffffff，再此基础上做加减，在每次上传后恢复初始值
	
	uint16_t  Dir_Pin;
	uint16_t  GPIO_Pin;//复位输入

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

