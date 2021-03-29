#include "Myfun.h"
sServoDC SDC0,SDC1,SDC2,SDC3,SDC4;
	extern uint8_t PinRead;
	extern uint8_t control;
	extern int imp[];
	extern uint16_t adc_val[2000];
	extern int state[];
	extern uint32_t mean;
	extern float volt;
	extern int ADC_State;
	extern int GripTime[5];

int main(void)
{
	SysTick_Config(16000000/1000);
	Motor_Conf();
	PinRead_Conf();
	Pwm_Conf();
	ServoDC_StructConf();
	USART_Conf();
	ADC_Conf();

	while(1)
	{

		SetAngle();
		ControlDC(control);
		

}
	}

