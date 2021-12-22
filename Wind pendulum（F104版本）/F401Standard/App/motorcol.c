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
uint8_t RoundDir = 0; 				 //正反转控制
extern float Angle1[3];
extern btDataType mode;

float R = 30.0; 					 //半径设置(cm)
float angle = 30.0;					 //摆动角度设置(°)
 fp32 pid[3];

float set_x = 0.0;
float A = 0.0;



void USER_Mode(uint32_t modenum)//模式选择
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
	
	MoveTimeCnt += 5;							 //每5ms运算1次         	
		
	M1.PWM = PID_calc(&M1PID,M1.CurPos,0);	//X方向PID计算
	M2.PWM = PID_calc(&M2PID,M2.CurPos,0);  //Y方向PID计算	

	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);//因为画的板子陀螺仪与坐标轴方向偏离了45度，所以要把PID输出的值在x，y方向分解
	}
	
	if(modenum == 2)//画园
	{
		const float priod = 1269.4;  //单摆周期(毫秒)
		static uint32_t MoveTimeCnt = 0;
		float set_x = 0.0;
		float set_y = 0.0;
		float A = 0.0;
		float phase = 0.0;
		float Normalization = 0.0;
		float Omega = 0.0;
	
		MoveTimeCnt += 5;							 //每5ms运算1次
		Normalization = (float)MoveTimeCnt / priod;	 //对单摆周期归一化
		Omega = 2.0*3.14159*Normalization;			 //对2π进行归一化处理				
		A = atan((R/88.0f))*57.2958f;    //根据半径求出对应的振幅A
		
		if(RoundDir == 0)       	  
			phase = 3.141592/2.0;		 //逆时针旋转相位差90° 
		else if(RoundDir == 1)  
			phase = (3.0*3.141592)/2.0;	 //顺时针旋转相位差270°
		
		set_x = A*sin(Omega);			 //计算出X方向当前摆角
		set_y = A*sin(Omega+phase); 	 //计算出Y方向当前摆角
	
		M1.PWM = PID_calc(&M1PID,M1.CurPos,set_x); //Pitch
		M2.PWM = PID_calc(&M2PID,M2.CurPos,set_y); //Roll
		
		MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);
	}
	
	if(modenum == 3)
	{
		const float priod = 1269.4;  //单摆周期(毫秒)
	static uint32_t MoveTimeCnt = 0;
	
	 
	float Normalization = 0.0;
	float Omega = 0.0;	
	A = 0.0;	
	MoveTimeCnt += 5;							 //每5ms运算1次
	Normalization = (float)MoveTimeCnt / priod;	 //对单摆周期归一化
	Omega = 2.0*3.14159*Normalization;			 //对2π进行归一化处理
	A = atan((R/53.0f))*57.2958f;//根据摆幅求出角度A,63为摆杆离地高度
	set_x = 20*sin(Omega); 		//计算出当前摆角 
			
	M1.PWM = PID_calc(&M1PID,M1.CurPos,set_x); //Pitch
	M2.PWM = PID_calc(&M2PID,M2.CurPos,0); //Roll
		
	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);
	}
}


/*------------------------------------------
 函数功能:风力摆初始化
 函数说明:			
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
	
	HAL_Delay(5000);//防止上电之后被削手
}


/*------------------------------------------
 函数功能:风力摆
 函数说明:在main里调用
------------------------------------------*/
void Windpendulum_Run(void)
{
	if(User_GetTick() - appcount >= 5)
	{
		appcount = User_GetTick();
		
		if(mode.dataFlag == 1)//设置PID参数
		{
			mode.dataFlag = 0;
			if(mode.usemode == 0)//关断
			{
				pid[0] = 0;
				pid[1] = 0;
				pid[2] = 0;
			}
			else if(mode.usemode == 1)//定点0位置
			{
				pid[0] = 0;
				pid[1] = 0;
				pid[2] = 4000;
			}
			else if(mode.usemode == 2)//画圆
			{
				pid[0] = 0;
				pid[1] = 0;
				pid[2] = 4000;
			}
			else if(mode.usemode == 3)//画直线
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
 函数功能:初始化M1结构体参数
 函数说明:			
------------------------------------------*/
void M1TypeDef_Init(void)
{
	M1.CurPos    = 0.0;
	M1.PrevPos   = 0.0;
	M1.CurAcc    = 0.0;
	M1.PrevSpeed = 0.0;
 	M1.Offset    = 0.1;   //允许偏差量
	M1.CurSpeed  = 0.0;  //当前速度矢量
	M1.PWM = 0;	         //PWM
}

/*------------------------------------------
 函数功能:初始化M2结构体参数
 函数说明:			
------------------------------------------*/
void M2TypeDef_Init(void)
{
	M2.CurPos    = 0.0;
	M2.PrevPos   = 0.0;
	M2.CurAcc    = 0.0;
	M2.PrevSpeed = 0.0;
 	M2.Offset    = 0.1;   //允许偏差量
	M2.CurSpeed  = 0.0;  //当前速度矢量
	M2.PWM = 0;	         //PWM		
}


/*------------------------------------------
 函数功能:电机方向控制

------------------------------------------*/

void Motor1(int v)//西
{
	User_PWM_Set1(v);
	}
	
void Motor2(int v)//西
{
	User_PWM_Set2(v);
	}

	
void Motor3(int v)//西
{
	User_PWM_Set3(v);
	}

void Motor4(int v)//西
{
	User_PWM_Set4(v);
	}


/*------------------------------------------
 函数功能:电机底层驱动函数
 函数说明:

用MOS管代替了电机驱动，电机只能一个方向旋转，通过更换桨叶或者改变电机正负极使电机方向正确
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

