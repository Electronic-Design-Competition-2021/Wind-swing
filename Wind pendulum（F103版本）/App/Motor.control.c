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
uint8_t RoundDir = 0; 				 //正反转控制
extern float Angle1[3];

float R = 30.0; 					 //半径设置(cm)
float angle = 30.0;					 //摆动角度设置(°)

//uint8_t RoundDir = 0; 				 //正反转控制

float set_x = 0.0;
float A = 0.0;
/*------------------------------------------
 函数功能:控制器软件复位
 函数说明:强制复位			
------------------------------------------*/
void MCU_Reset(void) 
{  
	__set_FAULTMASK(1);   // 关闭所有中断
 	NVIC_SystemReset();   // 复位
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

void Mode_0(void)
{

	static uint32_t MoveTimeCnt = 0;
	
	float set_x = 0.0;
	float A = 0.0;
	float Normalization = 0.0;
	float Omega = 0.0;	
	
	MoveTimeCnt += 5;							 //每5ms运算1次         

	PID_M1_SetKd(3000);
	PID_M2_SetKd(3000);

	PID_M1_SetPoint(0);	//X方向PID跟踪目标值0
	PID_M2_SetPoint(0);		//Y方向PID定位目标值0
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos-staticx);	//X方向PID计算
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos-staticy);  //Y方向PID计算
	

	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);//因为画的板子陀螺仪与坐标轴方向偏离了45度，所以要把PID输出的值在x，y方向分解
}


/*------------------------------------------
 函数功能:第2问PID计算
 函数说明:
------------------------------------------*/
void Mode_2(void)
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
	set_x = 20*sin(Omega);                        //计算出当前摆角 			
	PID_M1_SetPoint(set_x);	//X方向PID跟踪目标值sin
	PID_M1_SetKp(0);	
	PID_M1_SetKi(0);	 
	PID_M1_SetKd(3000);	
	
	
	PID_M2_SetPoint(0);		//Y方向PID定位目标值0
	PID_M2_SetKp(0);	
	PID_M2_SetKi(0);	 
	PID_M2_SetKd(3000);	
	
	
	M1.PWM = PID_M1_PosLocCalc(M1.CurPos-staticx);	//X方向PID计算
	M2.PWM = PID_M2_PosLocCalc(M2.CurPos-staticy);  //Y方向PID计算	
	if(M1.PWM > POWER_MAX) M1.PWM  =  POWER_MAX;//输出限幅
	if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX; 	
	if(M2.PWM > POWER_MAX) M2.PWM  =  POWER_MAX;
	if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;			
	MotorMove(0.707*M1.PWM+0.707*M2.PWM,0.707*M2.PWM-0.707*M1.PWM);
}




void Mode_4(void)
{	
	if(abs(M1.CurPos)<45.0 && abs(M2.CurPos)<45.0)	//小于45度才进行制动
	{		
		PID_M1_SetPoint(0);	  //X方向PID定位目标值0

		PID_M2_SetPoint(0);	  //Y方向PID定位目标值0

			
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
	 
	PID_M1_SetPoint(set_x);	//X方向PID跟踪目标值sin
//	PID_M1_SetKp(60);	
//	PID_M1_SetKi(0.79);	 
	PID_M1_SetKd(3000);

	PID_M2_SetPoint(set_y);	//Y方向PID跟踪目标值cos
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

