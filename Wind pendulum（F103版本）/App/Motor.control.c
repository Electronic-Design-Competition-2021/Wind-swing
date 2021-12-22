#include "Motor.control.h"
#include "stdio.h"
#include "math.h"
//#include "6050.h"
#include "motor.pwm.h"
#include "pid.h"
#include "stdlib.h"

M1TypeDef M1;
M2TypeDef M2;

extern PIDTypdDef M1PID;
extern PIDTypdDef M2PID;
uint8_t RoundDir = 0; 				 //����ת����
extern float Angle1[3];

float R = 30.0; 					 //�뾶����(cm)
float angle = 30.0;					 //�ڶ��Ƕ�����(��)

//uint8_t RoundDir = 0; 				 //����ת����

float set_x = 0.0;
float A = 0.0;
/*------------------------------------------
 ��������:�����������λ
 ����˵��:ǿ�Ƹ�λ			
------------------------------------------*/
void MCU_Reset(void) 
{  
	__set_FAULTMASK(1);   // �ر������ж�
 	NVIC_SystemReset();   // ��λ
}

/*------------------------------------------
 ��������:��ʼ��M1�ṹ�����
 ����˵��:			
------------------------------------------*/
void M1TypeDef_Init(void)
{
	M1.CurPos    = 0.0;
	M1.PrevPos   = 0.0;
	M1.CurAcc    = 0.0;
	M1.PrevSpeed = 0.0;
 	M1.Offset    = 0.1;   //����ƫ����
	M1.CurSpeed  = 0.0;  //��ǰ�ٶ�ʸ��
	M1.PWM = 0;	         //PWM
}

/*------------------------------------------
 ��������:��ʼ��M2�ṹ�����
 ����˵��:			
------------------------------------------*/
void M2TypeDef_Init(void)
{
	M2.CurPos    = 0.0;
	M2.PrevPos   = 0.0;
	M2.CurAcc    = 0.0;
	M2.PrevSpeed = 0.0;
 	M2.Offset    = 0.1;   //����ƫ����
	M2.CurSpeed  = 0.0;  //��ǰ�ٶ�ʸ��
	M2.PWM = 0;	         //PWM		
}

void Mode_0(void)
{

	static uint32_t MoveTimeCnt = 0;
	
	float set_x = 0.0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;	
	
	MoveTimeCnt += 5;							 //ÿ5ms����1��         

	PID_M1_SetKd(3000);
	PID_M2_SetKd(3000);

	PID_M1_SetPoint(0);	//X����PID����Ŀ��ֵ0
	PID_M2_SetPoint(0);		//Y����PID��λĿ��ֵ0
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos-staticx);	//X����PID����
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos-staticy);  //Y����PID����
	

	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);//��Ϊ���İ����������������᷽��ƫ����45�ȣ�����Ҫ��PID�����ֵ��x��y����ֽ�
}


/*------------------------------------------
 ��������:��2��PID����
 ����˵��:
------------------------------------------*/
void Mode_2(void)
{
	const float priod = 1269.4;  //��������(����)
	static uint32_t MoveTimeCnt = 0;
	
	 
	float Normalization = 0.0;
	float Omega = 0.0;	
	A = 0.0;	
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������
	A = atan((R/53.0f))*57.2958f;//���ݰڷ�����Ƕ�A,63Ϊ�ڸ���ظ߶�
	set_x = 20*sin(Omega);                        //�������ǰ�ڽ� 			
	PID_M1_SetPoint(set_x);	//X����PID����Ŀ��ֵsin
	PID_M1_SetKp(0);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(3000);	
	
	
	PID_M2_SetPoint(0);		//Y����PID��λĿ��ֵ0
	PID_M2_SetKp(0);	
	PID_M2_SetKi(0);	 
	PID_M2_SetKd(3000);	
	
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos-staticx);	//X����PID����
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos-staticy);  //Y����PID����	
	if(M1.PWM > POWER_MAX) M1.PWM  =  POWER_MAX;//����޷�
	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX; 	
	if(M2.PWM > POWER_MAX) M2.PWM  =  POWER_MAX;
	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;			
	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);
}




void Mode_4(void)
{	
	if(abs(M1.CurPos)<45.0 && abs(M2.CurPos)<45.0)	//С��45�ȲŽ����ƶ�
	{		
		PID_M1_SetPoint(0);	  //X����PID��λĿ��ֵ0

		PID_M2_SetPoint(0);	  //Y����PID��λĿ��ֵ0

			
		M1.PWM = PID_M1_PosLocCalc(M1.CurPos); //Pitch
		M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Roll
		
		if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
		if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;

		if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
		if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;
	}
	else	
	{
	 	M1.PWM = 0;
		M2.PWM = 0;	
	}
	
	MotorMove(M1.PWM,M2.PWM);
}


void Mode_5(void)
{
	const float priod = 1269.4;  //��������(����)
	static uint32_t MoveTimeCnt = 0;
	float set_x = 0.0;
	float set_y = 0.0;
	float A = 0.0;
	float phase = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;
	
	MoveTimeCnt += 5;							 //ÿ5ms����1��
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ��
	Omega = 2.0*3.14159*Normalization;			 //��2�н��й�һ������				
	A = atan((R/88.0f))*57.2958f;    //���ݰ뾶�����Ӧ�����A
	
	if(RoundDir == 0)       	  
		phase = 3.141592/2.0;		 //��ʱ����ת��λ��90�� 
	else if(RoundDir == 1)  
		phase = (3.0*3.141592)/2.0;	 //˳ʱ����ת��λ��270��
	
	set_x = A*sin(Omega);			 //�����X����ǰ�ڽ�
	set_y = A*sin(Omega+phase); 	 //�����Y����ǰ�ڽ�
	 
	PID_M1_SetPoint(set_x);	//X����PID����Ŀ��ֵsin
//	PID_M1_SetKp(60);	
//	PID_M1_SetKi(0.79);	 
	PID_M1_SetKd(3000);

	PID_M2_SetPoint(set_y);	//Y����PID����Ŀ��ֵcos
//	PID_M2_SetKp(60);    
//	PID_M2_SetKi(0.79);		
	PID_M2_SetKd(3000); 		 
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos); //Pitch
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Roll
	
//	if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
//	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;
//			 	
//	if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
//	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;		

	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);

	
}

/*------------------------------------------
 ��������:����ײ���������
 ����˵��:

��MOS�ܴ����˵�����������ֻ��һ��������ת��ͨ��������Ҷ���߸ı���������ʹ���������ȷ
------------------------------------------*/
void MotorMove(int32_t pwm1,int32_t pwm2)
{
	if(pwm1>=0)
	{
		Motor1(pwm1);
		Motor4(0);
	}
	if(pwm1<0)
	{
		Motor1(0);
		Motor4(-pwm1);
	}
	
	
	if(pwm2>=0)
	{
		Motor3(0);
		Motor2(pwm2);
	}
	if(pwm2<0)
	{
		Motor2(0);
		Motor3(-pwm2);
	}
	
	
	
}

