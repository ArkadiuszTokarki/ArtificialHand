#include "Myfun.h"
volatile uint32_t tick = 0;
volatile uint8_t PinRead = 0;
volatile int imp[5];
volatile uint8_t control=0;
uint8_t stan = 0;
int state[5];
volatile uint16_t adc_val[2000];
volatile int ADC_State = 0;
extern sServoDC SDC0,SDC1,SDC2,SDC3,SDC4;
volatile int TotalTime=0;
int MuscleTension=0;
volatile int MuscleTension_state=0;
volatile 	uint32_t mean=0;
volatile float volt;
volatile int GripTime;


void delay_ms(uint32_t ms)
{
	tick = 0;
	while(tick < ms);
}

void SysTick_Handler(void)
{
	tick++;
	EMG_signal ();

}

void Motor_Conf(void)
{
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOD->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;
	GPIOD->MODER |= GPIO_MODER_MODER3_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER0_0;
	
	GPIOD->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;
	GPIOD->PUPDR |= GPIO_MODER_MODER3_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER0_0;
	
	GPIOA->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER11_0;
	GPIOA->PUPDR |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER11_0;
	
}

void Pwm_Conf(void)
{

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOA->MODER	|= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 |  GPIO_MODER_MODER0_1;
	GPIOA->PUPDR	|= GPIO_PUPDR_PUPDR6_0 | GPIO_MODER_MODER7_0 |  GPIO_MODER_MODER0_1;
	GPIOA->AFR[0]	|= 0x22000002;
	//GPIOA->AFR[1]	|= 0x00000020;
	

	GPIOB->MODER	|= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;
	GPIOB->PUPDR	|= GPIO_PUPDR_PUPDR0_0 | GPIO_MODER_MODER1_1;
	GPIOB->AFR[0]	|= 0x00000022;
	

	
	//timer 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; //PWM Mode 1 CH1 i CH2
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; //PWM Mode 1 CH3 i CH4
	TIM3->CCER	|= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;	
	TIM3->PSC	= 16-1;	//Prescaler
	TIM3->ARR	= 1000-1;	//Period
	TIM3->CR1	|= TIM_CR1_CEN;	//Enable TIM3
	
	// timer 5
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 ;	//PWM Mode 1 CH1
	TIM5->CCER	|= TIM_CCER_CC1E;
	TIM5->PSC	= 16-1;	//Prescaler
	TIM5->ARR	= 1000-1;	//Period
	TIM5->CR1	|= TIM_CR1_CEN;	//Enable TIM5
}

void PinRead_Conf(void){
	
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER 	&= ~GPIO_MODER_MODER4;
	GPIOE->PUPDR	|= GPIO_PUPDR_PUPD4_0;

	GPIOE->MODER 	&= ~GPIO_MODER_MODER2;
	GPIOE->PUPDR	|= GPIO_PUPDR_PUPD2_0;

	GPIOE->MODER 	&= ~GPIO_MODER_MODER3;
	GPIOE->PUPDR	|= GPIO_PUPDR_PUPD3_0;
	
	GPIOE->MODER 	&= ~GPIO_MODER_MODER5;
	GPIOE->PUPDR	|= GPIO_PUPDR_PUPD5_0;
	
	GPIOE->MODER 	&= ~GPIO_MODER_MODER6;
	GPIOE->PUPDR	|= GPIO_PUPDR_PUPD6_0;
}
ePinRead Pin_Read(void){
	
	//////////////////////////  1   ////////////////////
 		
	if((GPIOD->IDR & GPIO_IDR_ID4) ==RESET)
	{
  			if((GPIOE->IDR & GPIO_IDR_ID2) ==RESET && state[0] == 0)
			{	
				state[0] = 1;
				imp[0] ++;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID2)!=RESET && state[0] == 1)
			{
				state[0] = 0;	
			} 
	}
		if((GPIOD->IDR & GPIO_IDR_ID5) ==RESET)
	{
			if((GPIOE->IDR & GPIO_IDR_ID2) ==RESET && state[0] == 0)
			{	
				state[0]=1;
				imp[0] --;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID2)!=RESET && state[0] == 1)
			{
				state[0] = 0;	
			} 
	} 
	///////////////////////////   2   /////////////////////////////////////////
		if((GPIOD->IDR & GPIO_IDR_ID6) ==RESET)
	{
  			if((GPIOE->IDR & GPIO_IDR_ID3) ==RESET && state[1] == 0)
			{	
				state[1] = 1;
				imp[1] ++;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID3)!=RESET && state[1] == 1)
			{
				state[1] = 0;	
			} 
	}
		if((GPIOD->IDR & GPIO_IDR_ID7) ==RESET)
	{
			if((GPIOE->IDR & GPIO_IDR_ID3) ==RESET && state[1] == 0)
			{	
				state[1] =1;
				imp[1] --;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID3)!=RESET && state[1] == 1)
			{
				state[1] = 0;	
			} 
	} 
	//////////////////////////       3      //////////////////////////////////////////
		if((GPIOD->IDR & GPIO_IDR_ID2) ==RESET)
	{
  			if((GPIOE->IDR & GPIO_IDR_ID4) ==RESET && state[2] == 0)
			{	
				state[2] = 1;
				imp[2] ++;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID4)!=RESET && state[2] == 1)
			{
				state[2] = 0;	
			} 
	}
		if((GPIOD->IDR & GPIO_IDR_ID3) ==RESET)
	{
			if((GPIOE->IDR & GPIO_IDR_ID4) ==RESET && state[2] == 0)
			{	
				state[2]=1;
				imp[2] --;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID4)!=RESET && state[2] == 1)
			{
				state[2] = 0;	
			} 
	} 
	//////////////////////////       4       //////////////////////////////////////////
		if((GPIOD->IDR & GPIO_IDR_ID0) ==RESET)
	{
  			if((GPIOE->IDR & GPIO_IDR_ID5) ==RESET && state[3] == 0)
			{	
				state[3] = 1;
				imp[3] ++;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID5)!=RESET && state[3] == 1)
			{
				state[3] = 0;	
			} 
	}
		if((GPIOD->IDR & GPIO_IDR_ID1) ==RESET)
	{
			if((GPIOE->IDR & GPIO_IDR_ID5) ==RESET && state[3] == 0)
			{	
				state[3]=1;
				imp[3] --;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID5)!=RESET && state[3] == 1)
			{
				state[3] = 0;	
			} 
	} 
	///////////////////////////     5      /////////////////////////////////////////
		if((GPIOA->IDR & GPIO_IDR_ID11) ==RESET)
	{
  			if((GPIOE->IDR & GPIO_IDR_ID6) ==RESET && state[4] == 0)
			{	
				state[4] = 1;
				imp[4] ++;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID6)!=RESET && state[4] == 1)
			{
				state[4] = 0;	
			} 
	}
		if((GPIOA->IDR & GPIO_IDR_ID12) ==RESET)
	{
			if((GPIOE->IDR & GPIO_IDR_ID6) ==RESET && state[4] == 0)
			{	
				state[4]=1;
				imp[4] --;	
			}
			if((GPIOE->IDR & GPIO_IDR_ID6)!=RESET && state[4] == 1)
			{
				state[4] = 0;	
			} 
	} 
	////////////////////////////////////////////////////////////////////
}
void ServoDC_StructConf(void)
{
	// nastawy regulatora dla 1 silnika
	SDC0.kp = 15.0;
	SDC0.ki = 5.0;
	SDC0.kd = 0.04;
	SDC0.etotalmax = 100000;
	// nastawy regulatora dla 2 silnika
	SDC1.kp = 15.0;
	SDC1.ki = 5.0;
	SDC1.kd = 0.04;
	SDC1.etotalmax = 100000;
	// nastawy regulatora dla 3 silnika
	SDC2.kp = 15.0;
	SDC2.ki = 5.0;
	SDC2.kd = 0.04;
	SDC2.etotalmax = 100000;
	// nastawy regulatora dla 4 silnika
	SDC3.kp = 15.0;
	SDC3.ki = 5.0;
	SDC3.kd = 0.04;
	SDC3.etotalmax = 100000;
	// nastawy regulatora dla 5 silnika
	SDC4.kp = 15.0;
	SDC4.ki = 5.0;
	SDC4.kd = 0.04;
	SDC4.etotalmax = 100000;
}
static void ServoDC_GetEncoderValue(void)
{
	Pin_Read();

	SDC0.actangle = 360.0 * ((double)imp[0] / (180));
	SDC1.actangle = 360.0 * ((double)imp[1] / (180));
	SDC2.actangle = 360.0 * ((double)imp[2] / (180));
	SDC3.actangle = 360.0 * ((double)imp[3] / (180));
	SDC4.actangle = 360.0 * ((double)imp[4] / (180));
}
static void ServoDC_SetSpeed(double speed, int num)
{
	switch(num)
	{
		case 0:
				if(speed > 100.0)
					speed = 100.0;
				else if(speed < -100.0)
					speed = -100.0;
				
				if(speed > 0.0)
				{
					MOT0LON;
					MOT0ROFF;
					MOT_PWM1 = abs (speed * 9.99);
				}
				if(speed < 0.0)
				{
					MOT0RON;
					MOT0LOFF;
					MOT_PWM1 = abs (speed * 9.99);
				}
				else if(speed > -0.1 && speed < 0.1)
				{
					MOT_PWM1 = 0;
				}
		break;
				
				case 1:
				if(speed > 100.0)
					speed = 100.0;
				else if(speed < -100.0)
					speed = -100.0;
				
				if(speed > 0.0)
				{
					MOT1LON;
					MOT1ROFF;
					MOT_PWM2 = abs (speed * 9.99);
				}
				if(speed < 0.0)
				{
					MOT1RON;
					MOT1LOFF;
					MOT_PWM2 = abs (speed * 9.99);
				}
				else if(speed > -0.1 && speed < 0.1)
				{
					MOT_PWM2 = 0;
				}
			break;
				
				
				case 2:
				if(speed > 100.0)
					speed = 100.0;
				else if(speed < -100.0)
					speed = -100.0;
				
				if(speed > 0.0)
				{
					MOT2LON;
					MOT2ROFF; 
					
					MOT_PWM3 = abs (speed * 9.99);
				}
				if(speed < 0.0)
				{
					MOT2RON;
					MOT2LOFF;
					MOT_PWM3 = abs (speed * 9.99);
				}
				else if(speed > -0.1 && speed < 0.1)
				{
					MOT_PWM3 = 0;
				}
			break;
				
				
				case 3:
				if(speed > 100.0)
					speed = 100.0;
				else if(speed < -100.0)
					speed = -100.0;
				
				if(speed > 0.0)
				{
					MOT3LON;
					MOT3ROFF;
					MOT_PWM4 = abs (speed * 9.99);
				}
				if(speed < 0.0)
				{
					MOT3RON;
					MOT3LOFF;
					MOT_PWM4 = abs (speed * 9.99);
				}
				else if(speed > -0.1 && speed < 0.1)
				{
					MOT_PWM4 = 0;
				}
			break;
				
				case 4:
				if(speed > 100.0)
					speed = 100.0;
				else if(speed < -100.0)
					speed = -100.0;
				
				if(speed > 0.0)
				{
					MOT4LON;
					MOT4ROFF;
					MOT_PWM5 = abs (speed * 9.99);
				}
				if(speed < 0.0)
				{
					MOT4RON;
					MOT4LOFF;
					MOT_PWM5 = abs (speed * 9.99);
				}
				else if(speed > -0.1 && speed < 0.1)
				{
					MOT_PWM5 = 0;
				}
			break;
	}
}
void ServoDC_RegPID(int num)
{
	
	ServoDC_GetEncoderValue();
	
	switch (num)
	{
		case 0:
		SDC0.eprev = SDC0.e;
	SDC0.e = SDC0.setangle - SDC0.actangle;
	SDC0.ed =SDC0.e - SDC0.eprev;
	
	SDC0.etotal = SDC0.etotal + SDC0.e;
	if(SDC0.etotal > SDC0.etotalmax)
		SDC0.etotal = SDC0.etotalmax;
	if(SDC0.etotal < -SDC0.etotalmax)
		SDC0.etotal = -SDC0.etotalmax;
	SDC0.out = (SDC0.e * SDC0.kp) + (SDC0.etotal * 0.001 * SDC0.ki) + (SDC0.ed / 0.001 * SDC0.kd);
	
	 if(SDC0.setangle == SDC0.actangle) TIM3->CCR1=0;	//SDC0.out = 0;
	else ServoDC_SetSpeed(SDC0.out,0);
			break;
		
		case 1:
		SDC1.eprev = SDC1.e;
	SDC1.e = SDC1.setangle - SDC1.actangle;
	SDC1.ed =SDC1.e - SDC1.eprev;
	
	SDC1.etotal = SDC1.etotal + SDC1.e;
	if(SDC1.etotal > SDC1.etotalmax)
		SDC1.etotal = SDC1.etotalmax;
	if(SDC1.etotal < -SDC1.etotalmax)
		SDC1.etotal = -SDC1.etotalmax;
	SDC1.out = (SDC1.e * SDC1.kp) + (SDC1.etotal * 0.001 * SDC1.ki) + (SDC1.ed / 0.001 * SDC1.kd);
	
	 if(SDC1.setangle == SDC1.actangle) TIM3->CCR2=0; //SDC1.out = 0;

	else ServoDC_SetSpeed(SDC1.out,1);
			break;
		
		case 2:
	SDC2.eprev = SDC2.e;
	SDC2.e = SDC2.setangle - SDC2.actangle;
	SDC2.ed =SDC2.e - SDC2.eprev;
	
	SDC2.etotal = SDC2.etotal + SDC2.e;
	if(SDC2.etotal > SDC2.etotalmax)
		SDC2.etotal = SDC2.etotalmax;
	if(SDC2.etotal < -SDC2.etotalmax)
		SDC2.etotal = -SDC2.etotalmax;
	SDC2.out = (SDC2.e * SDC2.kp) + (SDC2.etotal * 0.001 * SDC2.ki) + (SDC2.ed / 0.001 * SDC2.kd);
	
	 if(SDC2.setangle == SDC2.actangle) TIM3->CCR3=0; //SDC2.out = 0;
	else ServoDC_SetSpeed(SDC2.out,2);
			break;
		
		case 3:
		SDC3.eprev = SDC3.e;
	SDC3.e = SDC3.setangle - SDC3.actangle;
	SDC3.ed =SDC3.e - SDC3.eprev;
	
	SDC3.etotal = SDC3.etotal + SDC3.e;
	if(SDC3.etotal > SDC3.etotalmax)
		SDC3.etotal = SDC3.etotalmax;
	if(SDC3.etotal < -SDC3.etotalmax)
		SDC3.etotal = -SDC3.etotalmax;
	SDC3.out = (SDC3.e * SDC3.kp) + (SDC3.etotal * 0.001 * SDC3.ki) + (SDC3.ed / 0.001 * SDC3.kd);
	
	 if(SDC3.setangle == SDC3.actangle) TIM3->CCR4=0; //SDC3.out = 0;
	else ServoDC_SetSpeed(SDC3.out,3);
			break;
		
		case 4:
			SDC4.eprev = SDC4.e;
	SDC4.e = SDC4.setangle - SDC4.actangle;
	SDC4.ed =SDC4.e - SDC4.eprev;
	
	SDC4.etotal = SDC4.etotal + SDC4.e;
	if(SDC4.etotal > SDC4.etotalmax)
		SDC4.etotal = SDC4.etotalmax;
	if(SDC4.etotal < -SDC4.etotalmax)
		SDC4.etotal = -SDC4.etotalmax;
	
	SDC4.out = (SDC4.e * SDC4.kp) + (SDC4.etotal * 0.001 * SDC4.ki) + (SDC4.ed / 0.001 * SDC4.kd);
	
	 if(SDC4.setangle == SDC4.actangle) TIM5->CCR1=0; //SDC4.out = 0;
	else ServoDC_SetSpeed(SDC4.out,4);
			break;
	}
	
}
void USART_Conf(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	GPIOC->MODER &= ~GPIO_MODER_MODER10 & ~GPIO_MODER_MODER11;
	GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;
	GPIOC->AFR[1] |= 0x00007700;
	
	USART3->BRR = 	16000000/9600; // BYLO 57600
  USART3->CR1 |= 	USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}
void ComSendChar(USART_TypeDef *USARTx, char c)
{
	while(!(USARTx->SR & USART_SR_TXE)){;}
		USARTx->DR = c;
}
void ComPuts(USART_TypeDef* USARTx, const char* str)
{
	while(*str)
		ComSendChar(USARTx, *str++);
}
int ComReceive(USART_TypeDef* USARTx)
{
	int znak;
	if((USARTx->SR & USART_SR_RXNE) != RESET)
	{
		znak = USARTx->DR;
		return znak;
	}
	return 0;
}

void SetAngle(void)
{
			switch ( ComReceive(USART3) )
	{
		case 1:
			control = 1;
			ADC_State = 1;
		
			break;
		case 2:
			control = 2;
			break;
		case 3:
			control = 3;
			break;
		case 4:
			control = 4;
			ADC_State = 0;
			break;

	}	

	
}
void ADC_Conf(void){
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;		
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	GPIOA->MODER |= GPIO_MODER_MODER2;

	
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)adc_val;
	DMA2_Stream0->NDTR = (uint32_t) 2000;
	DMA2_Stream0->CR |= DMA_SxCR_CIRC | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_EN;
	
	ADC1->SQR3 |= (2<<0);
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_ADON | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}
void ControlDC (int num)
{

	if  (num ==1)
	{

			SDC0.setangle = 700; //maly
			ServoDC_RegPID(0);
		
			SDC1.setangle = 750; //serdeczny
			ServoDC_RegPID(1);
		
			SDC2.setangle = 550; // kciuk
			ServoDC_RegPID(2);
		
			SDC3.setangle = 800; // srodkowy
			ServoDC_RegPID(3);
		
			SDC4.setangle = 750; //wskazujacy
			ServoDC_RegPID(4);
			
	}

	else if ( num == 2)
	{
			SDC0.setangle = 0; //maly
			ServoDC_RegPID(0);
		
			SDC1.setangle = 0; //  serdeczny
			ServoDC_RegPID(1);
		
			SDC2.setangle = 550; // kciuk
			ServoDC_RegPID(2);
		
			SDC3.setangle = 800; // srodkowy
			ServoDC_RegPID(3);
		
			SDC4.setangle = 750; //wskazujacy
			ServoDC_RegPID(4);
		
		
	}
	
	
	else if ( num == 3)
	{
			SDC0.setangle = 750; //maly
			ServoDC_RegPID(0);
		
			SDC1.setangle = 750; //  serdeczny
			ServoDC_RegPID(1);
		
			SDC2.setangle = 0; // kciuk
			ServoDC_RegPID(2);
		
			SDC3.setangle = 800; // srodkowy
			ServoDC_RegPID(3);
		
			SDC4.setangle = 750; //wskazujacy
			ServoDC_RegPID(4);
	}
	
	
	else if ( num == 4)
	{
			SDC0.setangle = 0;
			ServoDC_RegPID(0);
		
			SDC1.setangle = 0;
			ServoDC_RegPID(1);
		
			SDC2.setangle = 0;
			ServoDC_RegPID(2);
		
			SDC3.setangle = 0;
			ServoDC_RegPID(3);
		
			SDC4.setangle = 0;
			ServoDC_RegPID(4);
	}

}

void EMG_signal (void){	
// zmienna inkrementowana co 1 ms
TotalTime++;
// po upływie 3 sec zmienna zliczająca ilość napięć mięśnia jest zerowana
	if(TotalTime == 3000) MuscleTension=0;

	if(adc_val[0]>2000 ){	
	/* gdy na wejściu ADC pojawi się sygnał powyżej 1,5 , a wcześniej był niższy
	(MuscleTension_state==0) po zinkrementowaniu zmiennej MuscleTension,
	wartość MuscleTension_state zostje zmieniona, zapobiega to zwiększaniu 
	zmiennej MuscleTension przy stałym napięciu mięśnia*/
		if(MuscleTension_state==0) {
			MuscleTension++;
			MuscleTension_state=1; }
		// Pierwsze napięcie mięśnia zeruje zmienna zliczającą upływ czasu
		if(MuscleTension ==1)  TotalTime=0;
		// Po 5 krotnym napięciu mięśnia dłoń zostaje zamknięta
		if(MuscleTension ==5 && ADC_State == 0 ){
			ADC_State = 1;
			MuscleTension=0;
			control = 1; }
		// Po kolejnym 5 krotnym napięciu mięśnia dłoń zostaje otwarta
		else if(MuscleTension ==5 && ADC_State == 1 ){
			ADC_State = 0;
			MuscleTension=0;
			control = 4; }
	}
	
	if(adc_val[0]<1900 ) MuscleTension_state=0;
}

