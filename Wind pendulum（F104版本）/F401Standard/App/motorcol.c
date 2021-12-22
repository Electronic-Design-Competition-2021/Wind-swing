#include "motorcol.h"
#include "stdio.h"
#include "math.h"
#include "pid.h"
#include "stdlib.h"
#include "main.h"
#include "tim.h"
#include "bsp_delay.h"
#include "bsp_pwm.h"
#include "6050.h"
#include "usart.h"
#include "bluetooth.h"

M1TypeDef M1;
M2TypeDef M2;
extern struct SAngle 	stcAngle;

extern int32_t appcount;
pid_type_def M1PID;
pid_type_def M2PID;
uint8_t RoundDir = 0; 				 //����ת����
extern float Angle1[3];
extern btDataType mode;

float R = 30.0; 					 //�뾶����(cm)
float angle = 30.0;					 //�ڶ��Ƕ�����(��)
 fp32 pid[3];

float set_x = 0.0;
float A = 0.0;



void USER_Mode(uint32_t modenum)//ģʽѡ��
{
	if(modenum == 0)
	{
		User_PWM_Set1(0);
		User_PWM_Set2(0);
		User_PWM_Set3(0);
		User_PWM_Set4(0);
	}
	if(modenum == 1)
	{
	static uint32_t MoveTimeCnt = 0;
	
	float set_x = 0.0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;	
	
	MoveTimeCnt += 5;							 //ÿ5ms����1��         	
		
	M1.PWM = PID_calc(&M1PID,M1.CurPos,0);	//X����PID����
	M2.PWM = PID_calc(&M2PID,M2.CurPos,0);  //Y����PID����	

	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);//��Ϊ���İ����������������᷽��ƫ����45�ȣ�����Ҫ��PID�����ֵ��x��y����ֽ�
	}
	
	if(modenum == 2)//��԰
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
	
		M1.PWM = PID_calc(&M1PID,M1.CurPos,set_x); //Pitch
		M2.PWM = PID_calc(&M2PID,M2.CurPos,set_y); //Roll
		
		MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);
	}
	
	if(modenum == 3)
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
	set_x = 20*sin(Omega); 		//�������ǰ�ڽ� 
			
	M1.PWM = PID_calc(&M1PID,M1.CurPos,set_x); //Pitch
	M2.PWM = PID_calc(&M2PID,M2.CurPos,0); //Roll
		
	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);
	}
}


/*------------------------------------------
 ��������:�����ڳ�ʼ��
 ����˵��:			
------------------------------------------*/
void Windpendulum_Init(void)
{
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(led,GPIO_PIN_13,GPIO_PIN_SET);

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	
	
	mode.usemode = 0;
	pid[0] = 0;
	pid[1] = 0;
	pid[2] = 0;
	
	PID_init(&M1PID,PID_POSITION,pid,10000,10000);
	PID_init(&M2PID,PID_POSITION,pid,10000,10000);
	
	HAL_Delay(5000);//��ֹ�ϵ�֮������
}


/*------------------------------------------
 ��������:������
 ����˵��:��main�����
------------------------------------------*/
void Windpendulum_Run(void)
{
	if(User_GetTick() - appcount >= 5)
	{
		appcount = User_GetTick();
		
		if(mode.dataFlag == 1)//����PID����
		{
			mode.dataFlag = 0;
			if(mode.usemode == 0)//�ض�
			{
				pid[0] = 0;
				pid[1] = 0;
				pid[2] = 0;
			}
			else if(mode.usemode == 1)//����0λ��
			{
				pid[0] = 0;
				pid[1] = 0;
				pid[2] = 4000;
			}
			else if(mode.usemode == 2)//��Բ
			{
				pid[0] = 0;
				pid[1] = 0;
				pid[2] = 4000;
			}
			else if(mode.usemode == 3)//��ֱ��
			{
				pid[0] = 0;
				pid[1] = 0;
				pid[2] = 4000;
			}
			PID_init(&M1PID,PID_POSITION,pid,10000,10000);
			PID_init(&M2PID,PID_POSITION,pid,10000,10000);
		}
		
		Angle1[0] = (float)stcAngle.Angle[0]/32768*180;
		Angle1[1] = (float)stcAngle.Angle[1]/32768*180;
		Angle1[2] = (float)stcAngle.Angle[2]/32768*180; 
		
		M1.CurPos = Angle1[1]-staticsY; 
		M2.CurPos = Angle1[0]-staticsX;
		
		USER_Mode(mode.usemode);
		
		
	}
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


/*------------------------------------------
 ��������:����������

------------------------------------------*/

void Motor1(int v)//��
{
	User_PWM_Set1(v);
	}
	
void Motor2(int v)//��
{
	User_PWM_Set2(v);
	}

	
void Motor3(int v)//��
{
	User_PWM_Set3(v);
	}

void Motor4(int v)//��
{
	User_PWM_Set4(v);
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

