#include "motor.pwm.h"



void Motor1(int v)//��
{
  TIM4->CCR1 = v;
	}
	
void Motor2(int v)//��
{
  TIM4->CCR2 = v;
	}

	
void Motor3(int v)//��
{
  TIM4->CCR3 = v;
	}

void Motor4(int v)//��
{
  TIM4->CCR4 = v;
	}
	


