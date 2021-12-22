#ifndef __MOTOR_PID_H
#define __MOTOR_PID_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*------------------------------------------
 				PID�ṹ��				
------------------------------------------*/
#define staticx 0.9
#define staticy 2.86


typedef struct
{
	float  SetPoint; 	//  �趨Ŀ�� Desired Value 
	double  SumError;		//	����ۼ� 
		
	float  Proportion;      //  �������� Proportional Const 
	float  Integral;        //  ���ֳ��� Integral Const
	float  Derivative;      //  ΢�ֳ��� Derivative Const

	float LastError;     //  Error[-1]
	float PrevError;     //  Error[-2]

}PIDTypdDef;

int32_t  PID_M1_PosLocCalc(float NextPoint);

void     PID_M1_Init(void);
void 	 PID_M1_SetKd(float dKdd);
void 	 PID_M1_SetKi(float dKii);
void 	 PID_M1_SetKp(float dKpp);
void 	 PID_M1_SetPoint(float setpoint);

int32_t  PID_M2_PosLocCalc(float NextPoint);
void     PID_M2_Init(void);
void 	 PID_M2_SetKd(float dKdd);
void 	 PID_M2_SetKi(float dKii);
void 	 PID_M2_SetKp(float dKpp);
void 	 PID_M2_SetPoint(float setpoint);

#endif

