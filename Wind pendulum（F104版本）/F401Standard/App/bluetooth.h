#ifndef __APP_BT_H__
#define __APP_BT_H__


#include "struct_typedef.h"
#include "fifo.h"

#define BT_TX_LENGTH 20
#define BT_RX_LENGTH 4

typedef struct
{
	uint8_t dataFlag;	//���ݸ��±�־λ
	uint8_t buttonRes;	//button���
	float 	float1;		//������1
	uint8_t usemode;
} btDataType;

typedef struct
{
	float 	float1;		//������1
	float 	float2;		//������2
	float 	float3;		//������3
} btDataType_tx;

extern btDataType btData;
void bt_SendData(uint32_t data1 ,uint32_t data2);
void bt_DataProcess(fifo_s_t * p_fifo);

#endif

