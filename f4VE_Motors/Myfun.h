#ifndef _MYFUN
#define _MYFUN
#include <stm32f4xx.h>
#include <stm32f407xx.h>
#include <stdlib.h>
#include <stdio.h>
typedef enum{LedOff = 0, LedOn = 1, LedTog = 2}eLed;
typedef enum{Right = 0, Left = 1, Both = 2}eMot;
typedef enum{Reset1 = 0,Set2 = 1}ePinRead;


void delay_ms(uint32_t ms);

void Led_Conf(void);
void Led_OnOff(uint8_t num, eLed state);

void Motor_Conf(void);
void Pwm_Conf(void);

void PinRead_Conf(void);
ePinRead Pin_Read(void);

void USART_Conf(void);
void ComSendChar(USART_TypeDef *USARTx, char c);
int ComReceive(USART_TypeDef* USARTx);
void SetAngle(void);
void EMG_signal (void);





typedef struct
{
	double 		setangle;
	double 		actangle;
	double 		e;
	double		eprev;
	double		ed;
	double		etotal;
	double		etotalmax;
	double		kp;
	double		ki;
	double		kd;
	double 		out;
	uint16_t 	pwm;
}sServoDC;




void ServoDC_StructConf(void);
void ServoDC_Pwm_Conf(void);
void ServoDC_Encoder_Conf(void);
void ServoDC_RegPID(int num);
void ControlDC (int num);

void ADC_Conf(void);
#define MOT_PWM1			TIM3->CCR1 //PA6
#define MOT_PWM2			TIM3->CCR2 //PA7
#define MOT_PWM3			TIM3->CCR3 //PB0
#define MOT_PWM4			TIM3->CCR4 //PB1
#define MOT_PWM5			TIM5->CCR1 //PA9

#define MOT0LON  GPIOD->ODR |= GPIO_ODR_ODR_5
#define MOT0LOFF GPIOD->ODR &= ~GPIO_ODR_ODR_5
#define MOT0RON  GPIOD->ODR |= GPIO_ODR_ODR_4
#define MOT0ROFF GPIOD->ODR &= ~GPIO_ODR_ODR_4

#define MOT1LON  GPIOD->ODR |= GPIO_ODR_ODR_7
#define MOT1LOFF GPIOD->ODR &= ~GPIO_ODR_ODR_7
#define MOT1RON  GPIOD->ODR |= GPIO_ODR_ODR_6
#define MOT1ROFF GPIOD->ODR &= ~GPIO_ODR_ODR_6

#define MOT2LON  GPIOD->ODR |= GPIO_ODR_ODR_3
#define MOT2LOFF GPIOD->ODR &= ~GPIO_ODR_ODR_3
#define MOT2RON  GPIOD->ODR |= GPIO_ODR_ODR_2
#define MOT2ROFF GPIOD->ODR &= ~GPIO_ODR_ODR_2

#define MOT3LON  GPIOD->ODR |= GPIO_ODR_ODR_1
#define MOT3LOFF GPIOD->ODR &= ~GPIO_ODR_ODR_1
#define MOT3RON  GPIOD->ODR |= GPIO_ODR_ODR_0
#define MOT3ROFF GPIOD->ODR &= ~GPIO_ODR_ODR_0

#define MOT4LON  GPIOA->ODR |= GPIO_ODR_ODR_12
#define MOT4LOFF GPIOA->ODR &= ~GPIO_ODR_ODR_12
#define MOT4RON  GPIOA->ODR |= GPIO_ODR_ODR_11
#define MOT4ROFF GPIOA->ODR &= ~GPIO_ODR_ODR_11



#endif
