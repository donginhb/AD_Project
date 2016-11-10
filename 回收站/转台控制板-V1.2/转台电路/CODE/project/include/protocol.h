#ifndef __PROTOCOL__H
#define __PROTOCOL__H
#include <stm32f0xx.h>
#include <stdio.h>
#include <string.h>
#include "tool.h"
/*-----�ַ�ת��-----
FD->FE 7D
F8->FE 78
FE->FE 7E		*/

#define RECV_PROTOCOL_NUM 1	//����Э��ʵ������
#define RECV_PROTOCOL_BUFSIZE 20
#define RECV_BUFSIZE 50	//���ջ��������ܹ�����һ��Э���С�Ļ�������
#define SEND_BUFSIZE 300	//���ͻ�����������Э��ʹ�ӡ��Ϣʱ�Ļ�������

/*����Э������ö��*/
typedef enum {
ANGLE_PROTOCOL = 0x01
}RECV_PROTO_TYPE;
/*����Э������ö��*/
typedef enum {
STATE_PROTOCOL = 0x01,
ASK_PROTOCOL = 0x02
}SEND_PROTO_TYPE;

//###################################����Э����###################################
/****************************************************
	�ṹ����:	STATE_PROTOCOL_T
	����: ������״̬Э��ʵ��
	���ߣ�liyao 2015��9��8��14:10:51
****************************************************/
typedef struct{
	uint8_t head;
	uint8_t type;
	uint8_t para1;//������1�������֣�
	uint8_t para2;//��������
	uint8_t para3;//������2�������֣�
	uint8_t para4;//��������
	uint8_t para5;//ͷ���Ƕ�
	uint8_t para6;//���Ƕ�
	uint8_t para7;//�ҳ�Ƕ�
	uint8_t para8;//���ֵ���
	uint8_t para9;//���ֵ���
	uint8_t para10;//ͷ������
	uint8_t para11;//˫�����
	uint8_t para12;//Reserved
	uint8_t para13;//Reserved
	uint8_t para14;//������Ϣ��λ ��1
	uint8_t checksum;
	uint8_t tail;
}STATE_PROTOCOL_T;



/****************************************************
	�ṹ����:	ASK_PROTOCOL_T
	����: Ӧ��Э��ʵ��
	���ߣ�liyao 2015��9��8��14:10:51
****************************************************/
typedef struct{
	uint8_t head;
	uint8_t type;
	uint8_t para1;//���
	uint8_t checksum;
	uint8_t tail;
}ASK_PROTOCOL_T;

/****************************************************
	�ṹ����:	RESPOND_PROTOCOL_T
	����: ִ�н������Э��
	���ߣ�liyao 2015��10��16��16:33:41
****************************************************/
typedef struct{
	uint8_t head;
	uint8_t type;
	uint8_t para1;//���
	uint8_t para2;//��λ����������
	uint8_t para3;//��λ��֡����
	uint8_t para4;//ָ���Ƿ����
	uint8_t checksum;
	uint8_t tail;
}RESPOND_PROTOCOL_T;

//###################################����Э����###################################
/****************************************************
	�ṹ����:	RUN_PROTOCOL_T
	����:	˫�ֿ�������Э��ʵ��
	���ߣ�liyao 2015��9��8��14:10:51
	��:		fd 01 02 00 10 00 00 10 01 01 01 f8
****************************************************/
typedef struct{
	uint8_t head;
	uint8_t type;
	uint8_t para1;//motorID
	uint8_t para2;//����
	uint8_t para3;//�ٶȸ�8λ
	uint8_t para4;//�ٶȵ�8λ
	uint8_t para5;//����
	uint8_t para6;//����
	uint8_t para7;//����
	uint8_t para8;//����
	uint8_t para9;//���
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
	�ṹ����:	PROTOCOL_INFO_T
	����: Э����Ϣ����
	���ߣ�liyao 2015��9��8��14:10:51
****************************************************/
typedef struct _PROTOCOL_INFO_T PROTOCOL_INFO_T ;
struct _PROTOCOL_INFO_T{
	int8_t len;
	int8_t type;
	PROTOCOL_T protocol;
	void (*handle)(PROTOCOL_INFO_T*);
	int8_t (*check)(void*);
};


//###################################������###################################
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
