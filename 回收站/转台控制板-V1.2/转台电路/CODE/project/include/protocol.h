#ifndef __PROTOCOL__H
#define __PROTOCOL__H
#include <stm32f0xx.h>
#include <stdio.h>
#include <string.h>
#include "tool.h"
/*-----字符转义-----
FD->FE 7D
F8->FE 78
FE->FE 7E		*/

#define RECV_PROTOCOL_NUM 1	//接收协议实体数量
#define RECV_PROTOCOL_BUFSIZE 20
#define RECV_BUFSIZE 50	//接收缓冲区（能够接收一个协议大小的缓冲区）
#define SEND_BUFSIZE 300	//发送缓冲区（发送协议和打印信息时的缓冲区）

/*接收协议类型枚举*/
typedef enum {
ANGLE_PROTOCOL = 0x01
}RECV_PROTO_TYPE;
/*发送协议类型枚举*/
typedef enum {
STATE_PROTOCOL = 0x01,
ASK_PROTOCOL = 0x02
}SEND_PROTO_TYPE;

//###################################发送协议类###################################
/****************************************************
	结构体名:	STATE_PROTOCOL_T
	功能: 驱动板状态协议实体
	作者：liyao 2015年9月8日14:10:51
****************************************************/
typedef struct{
	uint8_t head;
	uint8_t type;
	uint8_t para1;//编码器1方向（左轮）
	uint8_t para2;//左轮增量
	uint8_t para3;//编码器2方向（右轮）
	uint8_t para4;//右轮增量
	uint8_t para5;//头部角度
	uint8_t para6;//左翅角度
	uint8_t para7;//右翅角度
	uint8_t para8;//左轮电流
	uint8_t para9;//右轮电流
	uint8_t para10;//头部电流
	uint8_t para11;//双翅电流
	uint8_t para12;//Reserved
	uint8_t para13;//Reserved
	uint8_t para14;//错误信息按位 置1
	uint8_t checksum;
	uint8_t tail;
}STATE_PROTOCOL_T;



/****************************************************
	结构体名:	ASK_PROTOCOL_T
	功能: 应答协议实体
	作者：liyao 2015年9月8日14:10:51
****************************************************/
typedef struct{
	uint8_t head;
	uint8_t type;
	uint8_t para1;//序号
	uint8_t checksum;
	uint8_t tail;
}ASK_PROTOCOL_T;

/****************************************************
	结构体名:	RESPOND_PROTOCOL_T
	功能: 执行结果反馈协议
	作者：liyao 2015年10月16日16:33:41
****************************************************/
typedef struct{
	uint8_t head;
	uint8_t type;
	uint8_t para1;//序号
	uint8_t para2;//上位机命令序列
	uint8_t para3;//上位机帧类型
	uint8_t para4;//指令是否完成
	uint8_t checksum;
	uint8_t tail;
}RESPOND_PROTOCOL_T;

//###################################接收协议类###################################
/****************************************************
	结构体名:	RUN_PROTOCOL_T
	功能:	双轮控制命令协议实体
	作者：liyao 2015年9月8日14:10:51
	例:		fd 01 02 00 10 00 00 10 01 01 01 f8
****************************************************/
typedef struct{
	uint8_t head;
	uint8_t type;
	uint8_t para1;//motorID
	uint8_t para2;//方向
	uint8_t para3;//速度高8位
	uint8_t para4;//速度低8位
	uint8_t para5;//度数
	uint8_t para6;//度数
	uint8_t para7;//度数
	uint8_t para8;//度数
	uint8_t para9;//序号
	uint8_t checksum;
	uint8_t tail;
}ANGLE_PROTOCOL_T;

typedef union 
{	
	STATE_PROTOCOL_T					state_protocol;	
	ANGLE_PROTOCOL_T	angle_protocol;
	ASK_PROTOCOL_T	ask_protocol;
}PROTOCOL_T;
/****************************************************
	结构体名:	PROTOCOL_INFO_T
	功能: 协议信息描述
	作者：liyao 2015年9月8日14:10:51
****************************************************/
typedef struct _PROTOCOL_INFO_T PROTOCOL_INFO_T ;
struct _PROTOCOL_INFO_T{
	int8_t len;
	int8_t type;
	PROTOCOL_T protocol;
	void (*handle)(PROTOCOL_INFO_T*);
	int8_t (*check)(void*);
};


//###################################声明区###################################
extern QUEUE_T* Recv_Protocol_Queue;
extern int8_t heart_flag;
extern PROTOCOL_INFO_T recv_protocol_infos[RECV_PROTOCOL_NUM];
extern void Protocol_Init(void);
extern PROTOCOL_INFO_T Create_Protocol_Info(int8_t len,SEND_PROTO_TYPE type,void (*handle)(PROTOCOL_INFO_T*),int8_t (*check)(void*));
extern int8_t put_byte(uint16_t data);
//int8_t _Send_To_Uart(PROTOCOL_INFO_T* protocol_info);
int8_t Send_To_Buff(PROTOCOL_INFO_T* protocol_info);
extern void ask_send(uint8_t serial_number);
extern void state_protocol_send(PROTOCOL_INFO_T* protocol_info);
extern void Buff_To_Uart(void);
#endif
