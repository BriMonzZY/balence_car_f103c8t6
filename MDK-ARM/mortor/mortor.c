#include "mortor.h"



void Set_Pwm(int moto1,int moto2)
{
	moto1 = -moto1;
	moto2 = -moto2;
	if(moto1 < 0) {
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	}
	
	if(moto2 < 0) {
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	}
	
	if(moto1 == 0) {
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
	}
	if(moto2 == 0) {
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
	}
	
	/* 限幅 */
	if(moto1 > 7200) moto1 = 7200;
	if(moto2 > 7200) moto2 = 7200;
	if(moto1 < -7200) moto1 = -7200;
	if(moto2 < -7200) moto2 = -7200;
	
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, myabs(moto1)); /* PWMA */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, myabs(moto2)); /* PWMB */
}



int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}

void Limit_PWM(void)
{
	/* PWM满幅是7200 限制在7200 */
	if(Moto1<-7200 ) Moto1=-7200 ;
	if(Moto1>7200 )  Moto1=7200 ;
	if(Moto2<-7200 ) Moto2=-7200 ;
	if(Moto2>7200 )  Moto2=7200 ;
}

/* 异常关闭电机 */
void Turn_Off(float angle)
{
	if(angle<-35 || angle > 35) {																	 
		Moto1=0;
		Moto2=0;
	}		
}
