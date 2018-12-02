//Alternate design
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "./lcd/lcd.h"
#include "./mpu6050/mpu6050.h"
#include "./i2c/bsp_i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 16.4

#define M_PI 3.14159265359	    

#define alpha 0.9

#define dt 0.0025
#define accel_dt 0.002
#define period 2500
#define accel_period 1000

float roll=0, pitch=0;
float roll_sum=0, pitch_sum=0;
float roll_normal, pitch_normal;

float roll_rate, pitch_rate;
float roll_rate_normal=0,roll_rate_sum,pitch_rate_normal=0,pitch_rate_sum;

float roll_error, pitch_error;
float last_roll_error=0,last_pitch_error=0;

float roll_rate_error, pitch_rate_error;
float last_roll_rate_error=0,last_pitch_rate_error=0;

float pid_roll_rate_sum=0,pid_pitch_rate_sum=0;

float p_value=5,d_value=30, i_value=0.1;//Adjust these values
float pid_i_roll=0,pid_i_pitch=0;

float p_value2=3, d_value2=10, i_value2=0.01;//Adjust these values
float pid_i_roll_rate=0,pid_i_pitch_rate=0;

float pid_roll, pid_pitch;

float last_roll, last_pitch;

void ComplementaryFilter(short accData[3], short gyrData[3], float *roll, float *pitch);
void Delayus(int duration);
void ctrl(void);
void ctrl2(void);

GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
USART_InitTypeDef USART_InitStructure;
void GPIO_Config(void);
void USART_Config(void);
void Four_Clocks(void);//maps clocks PWM in pins
void All_Timer_Pulse_Change(int pulse);
void Pulse_Balance(void);

void UU_PutNumber(USART_TypeDef* USARTx, int x);
void UU_PutString(USART_TypeDef* USARTx, char* message);

int stop = 7000;
int down = 9000;
int up = 8500;
int max = 13000;
int pulse;
int pulse2,pulse3,pulse4,pulse5;
int base_throttle;
uint16_t rxdata;
char AnglesChar[70];
bool increase=false;
bool decrease=false;
int flyf=false;
short Accel[3];
short Gyro[3];
char AChar[10];

int main(void)
{
	int i;
	pulse = stop;

	i2c_GPIO_Config();
	MPU6050_Init();
	LCD_INIT();
	GPIO_Config();
	USART_Config();
	Four_Clocks();

	
	Delayus(5000000);

	for (i = 0; i < (1/accel_dt); ++i)
	{
		MPU6050ReadGyro(Gyro);
		MPU6050ReadAcc(Accel);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
		roll_sum+=roll;
		pitch_sum+=pitch;
		Delayus(accel_period);
	}
	roll_normal=roll_sum/(1/accel_dt);
	pitch_normal=pitch_sum/(1/accel_dt);

	base_throttle=down;
	i=0;
	while(1)
	{
		i++;
		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);
		
		Delayus(accel_period);

		if (USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET) {
			rxdata = USART_ReceiveData(USART1);
			switch (rxdata) {
				case '0':
				base_throttle+=100;
				sprintf(AnglesChar,"D:%6.3f",d_value);
				break;
				case '1':
				base_throttle+=100;
				sprintf(AnglesChar,"D:%6.3f",d_value);
				break;
				case '2':
				d_value-=5;
				sprintf(AnglesChar,"D:%6.3f",d_value);
				break;
				case '3':
				d_value+=5;
				sprintf(AnglesChar,"D:%6.3f",d_value);
				break;
				case '4':
				p_value-=1;
				sprintf(AnglesChar,"D:%6.3f",p_value);
				break;
				case '5':
				p_value+=1;
				sprintf(AnglesChar,"D:%6.3f",p_value);
				break;
				case '6':
				p_value-=3;
				sprintf(AnglesChar,"P:%6.3f",p_value);
				break;
				case '7':
				p_value+=3;
				sprintf(AnglesChar,"P:%6.3f",p_value);
				break;
				case '8':
				i_value-=0.05;
				sprintf(AnglesChar,"I:%6.3f",i_value);
				break;
				case '9':
				i_value+=0.05;
				sprintf(AnglesChar,"I:%6.3f",i_value);
				break;
				case 'a':
				i_value-=0.15;
				sprintf(AnglesChar,"I:%6.3f",i_value);
				break;
				case 'b':
				i_value+=0.15;
				sprintf(AnglesChar,"I:%6.3f",i_value);
				break;
				case 'c':
					pid_i_roll=0;
					pid_i_pitch=0;
				break;
				case 'd':
				All_Timer_Pulse_Change(stop);
				return -1;
			}
			LCD_DrawString(0,60,AnglesChar);
		}

		MPU6050ReadAcc(Accel);
		MPU6050ReadGyro(Gyro);
		ComplementaryFilter(&Accel[0], &Gyro[0], &roll, &pitch);

		last_roll=roll;
		last_pitch=pitch;
		
		Delayus(accel_period);

		sprintf(AnglesChar,"RE:%6.1f PE:%6.1f",roll_rate_error,pitch_rate_error);
		LCD_DrawString(0,0,AnglesChar);
		
		sprintf(AnglesChar,"R:%6.1f P:%6.1f",roll_error,pitch_error);
		LCD_DrawString(0,20,AnglesChar);
		
		sprintf(AnglesChar,"R1:%6.1f P1:%6.1f",pid_roll_rate_sum,pid_pitch_rate_sum);
		LCD_DrawString(0,60,AnglesChar);

		sprintf(AnglesChar,"R2:%6.1f P2:%6.1f",pid_roll,pid_pitch);
		LCD_DrawString(0,80,AnglesChar);

		sprintf(AnglesChar,"p2:%5i p3:%5i",pulse2,pulse3);
		LCD_DrawString(0,100,AnglesChar);

		sprintf(AnglesChar,"p4:%5i p5:%5i",pulse4,pulse5);
		LCD_DrawString(0,120,AnglesChar);
		
		roll_rate=roll-last_roll;
		pitch_rate=pitch-last_pitch;
		
		sprintf(AChar,"%i,",(int)roll_rate_error);
		
		UU_PutString(USART1, AChar);
		
		ctrl();
		ctrl2();
		
		if (i==100) {
			UU_PutString(USART1, "\n");
			i=0;
		}
		
		Pulse_Balance();
		
		Delayus(period-2*accel_period);
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
	*pitch = *pitch * alpha + pitchAcc * (1-alpha);

// Turning around the Y axis results in a vector on the X-axis
	rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
	*roll = *roll * alpha + rollAcc * (1-alpha);
}

//Ripped this code online
}

void ctrl(void) {
	roll_error = roll - roll_normal;
	pitch_error = pitch - pitch_normal;

	pid_i_roll += i_value*roll_error;
	pid_i_pitch += i_value*pitch_error;
	
	pid_roll_rate_sum = p_value*roll_error+pid_i_roll+d_value*(roll_error-last_roll_error);
	pid_pitch_rate_sum = p_value*pitch_error+pid_i_pitch+d_value*(pitch_error-last_pitch_error);
	
	last_roll_error = roll_error;
	last_pitch_error = pitch_error;
}

void ctrl2(void) {
	roll_rate_error = roll_rate - pid_roll_rate_sum - roll_rate_normal;
	pitch_rate_error = pitch_rate - pid_pitch_rate_sum - pitch_rate_normal;

	pid_i_roll_rate += i_value2*roll_rate_error;
	pid_i_pitch_rate += i_value2*pitch_rate_error;

	pid_roll = p_value2*roll_rate_error+pid_i_roll_rate+d_value2*(roll_rate_error-last_roll_rate_error);
	pid_pitch = p_value2*pitch_rate_error+pid_i_pitch_rate+d_value2*(pitch_rate_error-last_pitch_rate_error);
	
	last_roll_rate_error = roll_rate_error;
	last_pitch_rate_error = pitch_rate_error;
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

//Generate 4 400hz pwm signals
	TIM_TimeBaseStructure.TIM_Period = 19999;
	TIM_TimeBaseStructure.TIM_Prescaler = 9;
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

void All_Timer_Pulse_Change(int pulse) {
	TIM_OCInitStructure.TIM_Pulse = pulse;
TIM_OC3Init(TIM2, &TIM_OCInitStructure);//PA2
TIM_OC1Init(TIM3, &TIM_OCInitStructure);//PA6
TIM_OC3Init(TIM4, &TIM_OCInitStructure);//PB8
TIM_OC4Init(TIM5, &TIM_OCInitStructure);//PA3
}

void Pulse_Balance(void) {
//a6 a2 roll + a3 b8 roll - 2 4 pitch + 3 5 pitch - 

	pulse2 = base_throttle + pid_roll - pid_pitch;
	pulse3 = base_throttle - pid_roll - pid_pitch;
	pulse4 = base_throttle + pid_roll + pid_pitch;
	pulse5 = base_throttle - pid_roll + pid_pitch;

	if (pulse2 > max)
		pulse2 = max;
	if (pulse3 > max)
		pulse3 = max;
	if (pulse4 > max)
		pulse4 = max;
	if (pulse5 > max)
		pulse5 = max;

	if (pulse2 < stop)
		pulse2 = stop;
	if (pulse3 < stop)
		pulse3 = stop;
	if (pulse4 < stop)
		pulse4 = stop;
	if (pulse5 < stop)
		pulse5 = stop;

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

