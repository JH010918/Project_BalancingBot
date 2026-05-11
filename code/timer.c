#include "device_driver.h"

#define TIM2_TICK         	(20) 
#define TIM2_FREQ 	  		(1000000/TIM2_TICK)	// Hz
#define TIME2_PLS_OF_1ms  	(1000/TIM2_TICK)
#define TIM2_MAX	  		(0xffffu)

#define TIM4_TICK	  		(20) 
#define TIM4_FREQ 	  		(1000000/TIM4_TICK) // Hz
#define TIME4_PLS_OF_1ms  	(1000/TIM4_TICK)
#define TIM4_MAX	  		(0xffffu)

void TIM2_Stopwatch_Start(void){
	Macro_Set_Bit(RCC->APB1ENR, 0);

	TIM2->CR1 = (1<<4)|(1<<3);
	TIM2->PSC = (unsigned int)(TIMXCLK/50000.0 + 0.5)-1;
	TIM2->ARR = TIM2_MAX;

	Macro_Set_Bit(TIM2->EGR,0);
	Macro_Set_Bit(TIM2->CR1, 0);
}

unsigned int TIM2_Stopwatch_Stop(void){
	unsigned int time;

	Macro_Clear_Bit(TIM2->CR1, 0);
	time = (TIM2_MAX - TIM2->CNT) * TIM2_TICK;
	return time;
}

/* Delay Time Max = 65536 * 20use = 1.3sec */

#if 0

#else

/* Delay Time Extended */

void TIM2_Delay(int time)
{
	int i;
	unsigned int t = TIME2_PLS_OF_1ms * time;

	Macro_Set_Bit(RCC->APB1ENR, 0);

	TIM2->PSC = (unsigned int)(TIMXCLK/(double)TIM2_FREQ + 0.5)-1;
	TIM2->CR1 = (1<<4)|(1<<3);
	TIM2->ARR = 0xffff;
	Macro_Set_Bit(TIM2->EGR,0);

	for(i=0; i<(t/0xffffu); i++)
	{
		Macro_Set_Bit(TIM2->EGR,0);
		Macro_Clear_Bit(TIM2->SR, 0);
		Macro_Set_Bit(TIM2->CR1, 0);
		while(Macro_Check_Bit_Clear(TIM2->SR, 0));
	}

	TIM2->ARR = t % 0xffffu;
	Macro_Set_Bit(TIM2->EGR,0);
	Macro_Clear_Bit(TIM2->SR, 0);
	Macro_Set_Bit(TIM2->CR1, 0);
	while (Macro_Check_Bit_Clear(TIM2->SR, 0));

	Macro_Clear_Bit(TIM2->CR1, 0);
}

#endif

void TIM4_Repeat(int time)
{
	Macro_Set_Bit(RCC->APB1ENR, 2);

	TIM4->CR1 = (1<<4)|(0<<3);
	TIM4->PSC = (unsigned int)(TIMXCLK/(double)TIM4_FREQ + 0.5)-1;
	TIM4->ARR = TIME4_PLS_OF_1ms * time - 1;

	Macro_Set_Bit(TIM4->EGR,0);
	Macro_Clear_Bit(TIM4->SR, 0);
	Macro_Set_Bit(TIM4->CR1, 0);
}

int TIM4_Check_Timeout(void)
{
	if(Macro_Check_Bit_Set(TIM4->SR, 0))
	{
		Macro_Clear_Bit(TIM4->SR, 0);
		return 1;
	}
	else
	{
		return 0;
	}
}

void TIM4_Stop(void)
{
	Macro_Clear_Bit(TIM4->CR1, 0);
}

void TIM4_Change_Value(int time)
{
	TIM4->ARR = TIME4_PLS_OF_1ms * time;
}

void TIM4_Repeat_Interrupt_Enable(int en, int time)
{
	if(en)
	{
		Macro_Set_Bit(RCC->APB1ENR, 2);

		TIM4->CR1 = (1<<4)|(0<<3);
		TIM4->PSC = (unsigned int)(TIMXCLK/(double)TIM4_FREQ + 0.5)-1;
		TIM4->ARR = TIME4_PLS_OF_1ms * time;
		Macro_Set_Bit(TIM4->EGR,0);

		Macro_Clear_Bit(TIM4->SR, 0);
		NVIC_ClearPendingIRQ(30);

		Macro_Set_Bit(TIM4->DIER, 0);
		NVIC_EnableIRQ(30);

		Macro_Set_Bit(TIM4->CR1, 0);
	}

	else
	{
		NVIC_DisableIRQ(30);
		Macro_Clear_Bit(TIM4->CR1, 0);
		Macro_Clear_Bit(TIM4->DIER, 0);
	}
}

void TIM1_PWM_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, 1); 
    Macro_Set_Bit(RCC->APB2ENR, 0); 

    // Alternate Function(AF) 모드
    Macro_Write_Block(GPIOB->MODER, 0xF, 0xA, 26); 
    
    //  AF01(TIM1) 연결
    Macro_Write_Block(GPIOB->AFR[1], 0xFF, 0x11, 20); // 
    
    // 타이머 속도 설정
    TIM1->PSC = 4 - 1;    // 96MHz / 4 = 24MHz 
    TIM1->ARR = 1200 - 1; // 24MHz / 1200 = 20kHz
    
    // PWM 모드 1 설정 (CH1, CH2)
    TIM1->CCMR1 |= (6 << 4) | (1 << 3);
    TIM1->CCMR1 |= (6 << 12) | (1 << 11);
    
    // 역상 채널(CH1N, CH2N) 출력 활성화
    TIM1->CCER |= (1 << 2) | (1 << 6);
    TIM1->BDTR |= (1 << 15); 
    
    TIM1->CR1 |= (1 << 0);
}

void TIM4_PWM_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, 1);
    Macro_Set_Bit(RCC->APB1ENR, 2); 

    Macro_Write_Block(GPIOB->MODER, 0xF, 0xA, 12); 
    
    Macro_Write_Block(GPIOB->AFR[0], 0xFF, 0x22, 24);
    
    TIM4->PSC = 4 - 1; 
    TIM4->ARR = 1200 - 1;
    
    TIM4->CCMR1 |= (6 << 4) | (1 << 3); 
    TIM4->CCMR1 |= (6 << 12) | (1 << 11); 
    
    TIM4->CCER |= (1 << 0) | (1 << 4); 
    
    TIM4->CR1 |= (1 << 0);
}


// 엔코더 핀 및 타이머 초기화 함수
void Encoder_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, 0);  
    Macro_Set_Bit(RCC->APB1ENR, 3);  

    // Alternate Function(AF) 모드(10) 설정
    Macro_Write_Block(GPIOA->MODER, 0xF, 0xA, 0); 
    Macro_Write_Block(GPIOA->AFR[0], 0xFF, 0x22, 0);
    Macro_Write_Block(GPIOA->PUPDR, 0xF, 0x5, 0);

    // TIM5 엔코더 모드 3 설정 (x4 체배)
    TIM5->CCMR1 |= (1 << 0) | (1 << 8); 
    TIM5->CCER &= ~((1 << 1) | (1 << 5)); 
    TIM5->SMCR |= (3 << 0); 
    TIM5->ARR = 0xFFFFFFFF;
    TIM5->CR1 |= (1 << 0); 

    Macro_Set_Bit(RCC->APB1ENR, 1);  // TIM3 클럭 ON
    
    // Alternate Function(AF) 모드(10) 설정
    Macro_Write_Block(GPIOA->MODER, 0xF, 0xA, 12); 
    // AF02(TIM3) 연결 
    Macro_Write_Block(GPIOA->AFR[0], 0xFF, 0x22, 24);
    Macro_Write_Block(GPIOA->PUPDR, 0xF, 0x5, 12);

    // TIM3 엔코더 모드 3 설정
    TIM3->CCMR1 |= (1 << 0) | (1 << 8);
    TIM3->CCER &= ~((1 << 1) | (1 << 5)); 
    TIM3->SMCR |= (3 << 0); 
    TIM3->ARR = 0xFFFF; 
    TIM3->CR1 |= (1 << 0); 
}