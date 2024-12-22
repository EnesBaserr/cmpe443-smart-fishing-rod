#include <stdint.h>
#include <stdio.h>

typedef struct{ // REFERENCE: All structs are taken from PS10 slide.
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	uint32_t reserved1;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RCR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	uint32_t reserved2[2];
	volatile uint32_t BDTR;
}TIM15_General;

typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	uint32_t reserved;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO;
typedef struct{
volatile uint32_t ISR;
volatile uint32_t IER;
volatile uint32_t CR;
volatile uint32_t CFGR;
volatile uint32_t CFG2;
volatile uint32_t SMPR1;
volatile uint32_t SMPR2;
uint32_t reserved2;
volatile uint32_t TR1;
volatile uint32_t TR2;
volatile uint32_t TR3;
uint32_t reserved3;
volatile uint32_t SQR1;
volatile uint32_t SQR2;
volatile uint32_t SQR3;
volatile uint32_t SQR4;
volatile uint32_t DR;
uint32_t reserved4[2];
volatile uint32_t JSQR;
uint32_t reserved5[4];
volatile uint32_t OFR1;
volatile uint32_t OFR2;
volatile uint32_t OFR3;
volatile uint32_t OFR4;
uint32_t reserved6[4];
volatile uint32_t JDR1;
volatile uint32_t JDR2;
volatile uint32_t JDR3;
volatile uint32_t JDR4;
uint32_t reserved7[4];
volatile uint32_t AWD2CR;
volatile uint32_t AWD3CR;
uint32_t reserved8[2];
volatile uint32_t DIFSEL;
volatile uint32_t CALFACT;
} ADCType;
typedef struct{
volatile uint32_t CSR;
uint32_t reserved1;
volatile uint32_t CCR;
volatile uint32_t CDR;
} ADCCommon;

#define RCC_APB2ENR *((volatile uint32_t *) 0x40021060)
#define ISER2 *((volatile uint32_t *) 0xE000E108)
#define GPIOA ((GPIO *) 0x42020000)
#define TIM15 ((TIM15_General *)  0x40014000)
#define ADC1 ((ADCType *) 0x42028000)
#define ADC2 ((ADCType *) 0x42028100)
#define ADC ((ADCCommon *) 0x42028300)
#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define ISER1 *((volatile uint32_t *) 0xE000E104)
uint16_t lux_value;

void init_GPIO(void){ //Initilazing GPIO
	//Enabling clock for GPIOA
	RCC_AHB2ENR |= 1;
	RCC_AHB2ENR |= 1 << 1;
	GPIOA->MODER |= 0b11 << 2; //Setting PA1 as analog mode
	RCC_AHB2ENR |= 1 << 13; //Enabling the ADC clock
}
void init_LED(void)
{
	GPIOA->MODER &= ~(1 << 17); //PA8 is output
	GPIOA->MODER &= ~(1 << 1); //PA0 is output

}
void open_LED_GREEN(void)
{
	GPIOA->ODR |= (1 << 8); //PA8 is 1 to open RGB LED in green

}
void open_LED_BLUE(void){
	GPIOA->ODR &= ~(1 << 0); //PA0 is 0 to let the current
}
void close_LED_BLUE(void){
	GPIOA->ODR |= (1 << 0); //PA0 is 1 to cut the current
}
void close_LED_GREEN(void)
{
	GPIOA->ODR &= ~(1 << 8); //PA8 is 0 to closeRGB LED

}

void init_ADC(void) //REFERENCE: PS10 Slide
{
	ADC1->CR &= ~(1 << 29); //Powering ADC Module
	ADC1->CR |= (1 << 28); //Enabling ADC Voltage Regulator
	RCC_CCIPR1 |= 3 << 28; //main ADC clock is system clock
	ADC->CCR |= 3 << 16; //Selecting ADC Clock mode (1Mhz)
	ADC1->SMPR1 |= 0b101 << 18; //sampling time is 640.5 ADC12_IN6 clock cycles to decrease sampling error
	ADC1->SQR1 &= ~(0b1111 << 0); // Number of conversion is 1
	ADC1->SQR1 |= 6 << 6; // 1st conversion is ADC12_IN6
	ADC1->CR |= (1 << 31); //calibrate ADC
	while((ADC1->CR & (1 << 31)) != 0) {

	}
	//Wait until Calibration
	ADC1->CR |= 1; //turn on ADC module
	while((ADC1->ISR & 1) == 0) {

	}
	//Waiting until ADC is enabled
	ADC1->CR |= 1 << 2; //Starting Conversion
	ADC1->IER |= 1 << 2; //turn on EOC interrupts
	ISER1 |= 1 << 5;//Enabling global signals for ADC Interrupt
}
void ADC1_2_IRQHandler(void)	//REFERENCE: PS 10 Slide
{
	if((ADC1->ISR & 1<<2) != 0)
	{
		lux_value = ADC1->DR; //Reading light intensity value
		ADC1->CR |= 1<<2; //Starting conversion
	}
}
void __enable_irq(void) //REFERENCE: F24 09-Interrupts slide
{
	__asm volatile(	//Enabling all configured interrupts
			"mov r0, #0 \n\t"
			"msr primask, r0 \n\t"
	);
}


int main(void)
{
	init_GPIO(); //Initiliaze GPIOA clock
	init_ADC();	//Initiliaze ADC
	init_LED();	//Initiliaze RGB Led
	__enable_irq(); //Enable interrupts


	while(1)
		{
		if (lux_value < 100) //If the light intensity is below second threshold light the led in both green and blue
		{
			open_LED_BLUE();
		}
		else if(lux_value < 300){ //If the light intensity is below first threshold but not second, light only green
			open_LED_GREEN();
			close_LED_BLUE();
		}
	else{						// If light intensity is not below any threshold, don't light the led.
		close_LED_GREEN();
		close_LED_BLUE();
		}
		__asm volatile("wfi"); //Wait for interrupts
	}
	return(1);
}
