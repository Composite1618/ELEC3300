/**
  ******************************************************************************
  * @file   main.c
  * @author LYC
  * @version V1.0
  * @date   2014-04-22
  * @brief   MPU6050
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "./lcd/lcd.h"
#include "./mpu6050/mpu6050.h"
#include "./i2c/bsp_i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536

#define M_PI 3.14159265359	    

#define dt 0.01

float roll=0, pitch=0;
float roll_sum=0, pitch_sum=0;
float roll_normal, pitch_normal;

float roll_error, pitch_error;
float p_value=1.3,d_value=18;//Adjust these values
int pid_roll_sum=0,pid_pitch_sum=0;
float pid_i_roll=0,pid_i_pitch=0, i_value=0.03;
float last_roll_error=0,last_pitch_error=0;

void ComplementaryFilter(short accData[3], short gyrData[3], float *roll, float *pitch);
void Delayus(int duration);
void ctrl(float * rollp, float * pitchp, int * pid_roll_p, int * pid_pitch_p);

GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
USART_InitTypeDef USART_InitStructure;
void GPIO_Config(void);
void USART_Config(void);
void Delayus(int duration);
void Four_Clocks(void);//maps clocks PWM in pins
void All_Timer_Pulse_Change(int pulse);
void Pulse_Balance(void);

int stop = 1000;
int mid = 1100;
int up = 1100;
int pulse2,pulse3,pulse4,pulse5;
int base_throttle=1000;
uint16_t rxdata;
char AnglesChar[70];
int main(void)
{
	short Accel[3];
	short Gyro[3];
	int i;
	base_throttle=mid;

	i2c_GPIO_Config();
	MPU6050_Init();
	LCD_INIT();
	GPIO_Config();
	USART_Config();
	Four_Clocks();
	
	for (i = 0; i < 200; ++i)
	{
		Delayus(10000);
		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
	}
	
	All_Timer_Pulse_Change(1200);
	
	for (i = 0; i < 200; ++i)
	{
		Delayus(10000);
		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
	}
	
	All_Timer_Pulse_Change(stop);
	
	for (i = 0; i < 200; ++i)
	{
		Delayus(10000);
		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
		roll_sum+=roll;
		pitch_sum+=pitch;
	}
	roll_normal=roll_sum/200;
	pitch_normal=pitch_sum/200;
	
	
	while(1)
	{
		
		Delayus(10000);
		
		if (USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET) {
			rxdata = USART_ReceiveData(USART1);	
			if (rxdata == 'w') {
				LCD_DrawString(0,100,"WWWWW");
			}	
			else if (rxdata == 'a') {
				LCD_DrawString(0,100,"AAAAA");
			}	
			else if (rxdata == 's') {
				LCD_DrawString(0,100,"SSSSS");
			}	
			else if (rxdata == 'd') {
				LCD_DrawString(0,100,"DDDDD");
			}	
			else if (rxdata == 'f') {
				LCD_DrawString(0,100,"FFFFF");
			}	
			else if (rxdata == 'b') {
				LCD_DrawString(0,100,"BBBBB");
			}	
			else if (rxdata == '1') {
				All_Timer_Pulse_Change(1000);
				return -1;
			}	
		}
		
		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
		
		ctrl(&roll, &pitch, &pid_roll_sum, &pid_pitch_sum);
		
		sprintf(AnglesChar,"Roll:%6.3f Pitch:%6.3f",roll,pitch);
		LCD_DrawString(0,0,AnglesChar);
		
		sprintf(AnglesChar,"RN:%6.3f PN:%6.3f",roll_normal,pitch_normal);
		LCD_DrawString(0,20,AnglesChar);
		
		sprintf(AnglesChar,"RPID:%i PPID:%i",pid_roll_sum,pid_pitch_sum);
		LCD_DrawString(0,40,AnglesChar);
		
		Pulse_Balance();
		
		sprintf(AnglesChar,"pulse2:%i pulse3:%i",pulse2,pulse3);
		LCD_DrawString(0,60,AnglesChar);
		
		sprintf(AnglesChar,"pulse4:%i pulse5:%i",pulse4,pulse5);
		LCD_DrawString(0,80,AnglesChar);
	}
}

void ComplementaryFilter(short accData[3], short gyrData[3], float *roll, float *pitch) {
	float pitchAcc, rollAcc;
	int forceMagnitudeApprox ;         

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
	// Turning around the X axis results in a vector on the Y-axis
    	pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    	*pitch = *pitch * 0.98 + pitchAcc * 0.02;

	// Turning around the Y axis results in a vector on the X-axis
    	rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
    	*roll = *roll * 0.98 + rollAcc * 0.02;
    }
    
	//Ripped this code online
}

void ctrl(float * rollp, float * pitchp, int * pid_roll_p, int * pid_pitch_p) {
	roll_error = *rollp - roll_normal;
	pitch_error = *pitchp - pitch_normal;
	
	pid_i_roll += i_value*roll_error;
	pid_i_pitch += i_value*pitch_error;
	
	*pid_roll_p = p_value*roll_error+pid_i_roll+d_value*(roll_error-last_roll_error);
	*pid_pitch_p = p_value*pitch_error+pid_i_pitch+d_value*(pitch_error-last_pitch_error);
	
	last_roll_error = roll_error;
	last_pitch_error = pitch_error;
}

void GPIO_Config (void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //PA2

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //PA3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //PA6
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //PB8
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //Connect this pin to Bluetooth RX

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //Connect this pin to Bluetooth TX
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BSRR = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7; //LOW serve as ground
}

void USART_Config (void) {
	// Baud rate:9600, Word length: 8, Stop Bit: 1, Parity Bit: None

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_Cmd(USART1, ENABLE); 
}

void Four_Clocks (void)
{	
		//Initialize clocks TIM2-TIM5
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
		//Generate 4 50hz pwm signals
	TIM_TimeBaseStructure.TIM_Period = 19999;
	TIM_TimeBaseStructure.TIM_Prescaler = 72;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
		//Assign duty cycle to signals
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = stop;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);//PA2
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);//PA6
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);//PB8
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);//PA3

	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}

void USART1_SendString(const char * pStr) {
	while (*pStr != '\0') {
		USART_SendData(USART1, *pStr);
		pStr++;
	}
}

void All_Timer_Pulse_Change(int pulse) {
	TIM_OCInitStructure.TIM_Pulse = pulse;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);//PA2
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);//PA6
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);//PB8
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);//PA3
}

void Pulse_Balance(void) {
	//a6 a3 roll + a2 b8 roll - a2 a6 pitch+ b8 a3 pitch- 
	
	pulse2 = base_throttle - pid_roll_sum + pid_pitch_sum;
	pulse3 = base_throttle + pid_roll_sum + pid_pitch_sum;
	pulse4 = base_throttle - pid_roll_sum - pid_pitch_sum;
	pulse5 = base_throttle + pid_roll_sum - pid_pitch_sum;
	
	if (pulse2 > 2000)
		pulse2 = 2000;
	if (pulse3 > 2000)
		pulse3 = 2000;
	if (pulse4 > 2000)
		pulse4 = 2000;
	if (pulse5 > 2000)
		pulse5 = 2000;

	if (pulse2 < 1000)
		pulse2 = 1000;
	if (pulse3 < 1000)
		pulse3 = 1000;
	if (pulse4 < 1000)
		pulse4 = 1000;
	if (pulse5 < 1000)
		pulse5 = 1000;
	
	TIM_OCInitStructure.TIM_Pulse = pulse2;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = pulse3;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = pulse4;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = pulse5;
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
}

void Delayus(int duration) {
	while(duration--) 
	{
		int i=0x02;       
		while(i--)
			__asm("nop");
	}
}
