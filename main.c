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
#include <stdbool.h>
#include <math.h>

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536

#define M_PI 3.14159265359	    

#define dt 0.005

float roll=0, pitch=0;
float roll_sum=0, pitch_sum=0;
float roll_normal, pitch_normal;

float roll_error, pitch_error;
float last_roll_error=0,last_pitch_error=0;

int pid_roll_25_sum=0,pid_pitch_25_sum=0;
int pid_roll_34_sum=0,pid_pitch_34_sum=0;

float p_25_value=3.1,d_25_value=13.5, i_25_value=0.027;//Adjust these values
float pid_i_roll_25=0,pid_i_pitch_25=0;

float p_34_value=3.1,d_34_value=13.5, i_34_value=0.027;//Adjust these values
float pid_i_roll_34=0,pid_i_pitch_34=0;

void ComplementaryFilter(short accData[3], short gyrData[3], float *roll, float *pitch);
void Delayus(int duration);
void ctrl(float * rollp, float * pitchp, int * pid_roll_p_25, int * pid_pitch_p_25, int * pid_roll_p_34, int * pid_pitch_p_34);

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

int stop = 1100;
int down = 1300;
int up = 1475;
int pulse2,pulse3,pulse4,pulse5;
int base_throttle;
uint16_t rxdata;
char AnglesChar[70];
bool increase=false;
bool decrease=false;
int flyf=false;

int main(void)
{
	short Accel[3];
	short Gyro[3];
	int i;

	i2c_GPIO_Config();
	MPU6050_Init();
	LCD_INIT();
	GPIO_Config();
	USART_Config();
	Four_Clocks();

	for (i = 0; i < 400; ++i)
	{
		Delayus(5000);
		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
	}

	for (i = 0; i < 400; ++i)
	{
		Delayus(5000);
		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
	}

	for (i = 0; i < 400; ++i)
	{
		Delayus(5000);
		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
		roll_sum+=roll;
		pitch_sum+=pitch;
	}
	roll_normal=roll_sum/200;
	pitch_normal=pitch_sum/200;

	
	base_throttle=stop;

	while(1)
	{

		Delayus(5000);

		if (USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET) {
			rxdata = USART_ReceiveData(USART1);
			switch (rxdata) {
				case 'w':
				d_25_value+=1;
				sprintf(AnglesChar,"D:%6.3f",d_25_value);
				break;
				case 's':
				d_25_value-=1;
				sprintf(AnglesChar,"D:%6.3f",d_25_value);
				break;
				case 'a':
				d_25_value-=0.1;
				sprintf(AnglesChar,"D:%6.3f",d_25_value);
				break;
				case 'd':
				d_25_value+=0.1;
				sprintf(AnglesChar,"D:%6.3f",d_25_value);
				break;
				case 'f':
				p_25_value-=0.1;
				sprintf(AnglesChar,"P:%6.3f",p_25_value);
				break;
				case 'b':
				p_25_value+=0.1;
				sprintf(AnglesChar,"P:%6.3f",p_25_value);
				break;
				case '0':
				flyf=0;
				if (base_throttle<down)
					increase = true;
				if (base_throttle>down)
					decrease = true;
				break;
				case '1':
				flyf=1;
				if (base_throttle<up)
					increase = true;
				if (base_throttle>up)
					decrease = true;
				break;
				case '3':
				i_25_value+=0.001;
				sprintf(AnglesChar,"I:%6.3f",i_25_value);
				break;
				case '4':
				i_25_value-=0.001;
				sprintf(AnglesChar,"I:%6.3f",i_25_value);
				break;
				case '5':
				All_Timer_Pulse_Change(1000);
				return -1;
				default:
				LCD_DrawString(0,120,AnglesChar);
				break;
			}
		}

			MPU6050ReadAcc(Accel);
			MPU6050ReadGyro(Gyro);
			ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);

			ctrl(&roll, &pitch, &pid_roll_25_sum, &pid_pitch_25_sum, &pid_roll_34_sum, &pid_pitch_34_sum);
			
			Pulse_Balance();

			sprintf(AnglesChar,"Roll:%6.3f Pitch:%6.3f",roll,pitch);
			LCD_DrawString(0,0,AnglesChar);
			
			switch (flyf) {
				case 0:
				if (base_throttle>=down)
					increase = false;
				if (base_throttle<=down)
					decrease = false;
				break;
				case 1:
				if (base_throttle>=up)
					increase = false;
				if (base_throttle<=up)
					decrease = false;
				break;
			}
			
			if (increase == true) {
				base_throttle++;
			}
			
			if (decrease == true) {
				base_throttle--;
			}

			sprintf(AnglesChar,"RPID:%i PPID:%i",pid_roll_25_sum,pid_pitch_25_sum);
			LCD_DrawString(0,40,AnglesChar);

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

void ctrl(float * rollp, float * pitchp, int * pid_roll_p_25, int * pid_pitch_p_25, int * pid_roll_p_34, int * pid_pitch_p_34) {
	roll_error = *rollp - roll_normal;
	pitch_error = *pitchp - pitch_normal;

	pid_i_roll_25 += i_25_value*roll_error;
	pid_i_pitch_25 += i_25_value*pitch_error;

	*pid_roll_p_25 = p_25_value*roll_error+pid_i_roll_25+d_25_value*(roll_error-last_roll_error);
	*pid_pitch_p_25 = p_25_value*pitch_error+pid_i_pitch_25+d_25_value*(pitch_error-last_pitch_error);

	pid_i_roll_34 += i_34_value*roll_error;
	pid_i_pitch_34 += i_34_value*pitch_error;

	*pid_roll_p_34 = p_34_value*roll_error+pid_i_roll_34+d_34_value*(roll_error-last_roll_error);
	*pid_pitch_p_34 = p_34_value*pitch_error+pid_i_pitch_34+d_34_value*(pitch_error-last_pitch_error);
	
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
	TIM_OCInitStructure.TIM_Pulse = 1000;
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
//a6 a3 roll + a2 b8 roll - a2 a6 pitch + b8 a3 pitch - 

	pulse2 = base_throttle - pid_roll_25_sum + pid_pitch_25_sum;
	pulse3 = base_throttle + pid_roll_34_sum + pid_pitch_34_sum;
	pulse4 = base_throttle - pid_roll_34_sum - pid_pitch_34_sum;
	pulse5 = base_throttle + pid_roll_25_sum - pid_pitch_25_sum;

	if (pulse2 > 2000)
		pulse2 = 2000;
	if (pulse3 > 2000)
		pulse3 = 2000;
	if (pulse4 > 2000)
		pulse4 = 2000;
	if (pulse5 > 2000)
		pulse5 = 2000;

	if (pulse2 < 1100)
		pulse2 = 1100;
	if (pulse3 < 1100)
		pulse3 = 1100;
	if (pulse4 < 1100)
		pulse4 = 1100;
	if (pulse5 < 1100)
		pulse5 = 1100;

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
