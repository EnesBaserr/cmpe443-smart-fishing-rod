#include <stdint.h>

// Structs (From PS)
typedef struct{
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
	volatile uint32_t STH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO;

// Macros
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)		// Bus for GPIO (From PS)
#define RCC_APB2ENR *((volatile uint32_t *) 0x40021060)		// Bus for TIM15 (From PS)
#define ISER2 *((volatile uint32_t *) 0xE000E108)			// Interrupt enable (From PS)
#define GPIOA ((GPIO *) 0x42020000)							// GPIO pin A (From PS)
#define TIM15 ((TIM15_General *) 0x40014000)				// TIM15 pin (From PS)

// Globals
int vibration_count = 0;		// Storing the vibration count for IC
int cycle_count = 0;			// Storing the cycle count for seeing the effect of OC
int two_cycle = 0;				// Counting the vibrations for two cycles
int tempo = 0;					// Storing the value of duty cycle for OC

// Function vibration
void vibration_handler(void);	// The function will be implemented in next steps.

// IRQ
void TIM15_IRQHandler(void){	// Interrupt handler for TIM15 (From PS)
	if(TIM15->SR & (1 << 1)){		// Input interrupt handling
		TIM15->SR &= ~(1 << 1);		// Reset the value to zero
		vibration_count++;			// Increase vibration counter
	}
	if(TIM15->SR & (1 << 0)){		// Timer overflow interrupt handling (From PS)
		TIM15->SR &= ~(1 << 0);		// Reset the value to zero (From PS)
		two_cycle++;				// Increment two cycle variable
		if(two_cycle >= 2){			// If two, handle vibration results
			two_cycle = 0;			// Reset it to zero for next vibration count
			vibration_handler();	// Handle vibration
		}
	}
}

// GPIO
void init_GPIO(void){
	RCC_AHB2ENR |= 1;			// Bus enable for GPIO A (From PS)

	GPIOA->MODER &= ~(0b1111 << (2 * 2));		// Reset PA2 and PA3 to zero (From PS)
	GPIOA->MODER |= (0b1010 << (2 * 2));		// Mode PA2 and PA3 to alternate function (From PS)

	GPIOA->AFRL &= ~(0b11111111 << (4 * 2));	// Clear alternate function bits for PA2, PA3 (From PS)
	GPIOA->AFRL |= (0b11101110 << (4 * 2));		// Set alternate function configuration for PA2, PA3 (From PS)
}

// Timer
void init_TIMER(void){
	RCC_APB2ENR |= (1 << 16);			// Enable TIM15 clock (From PS) (From PS)

	TIM15->PSC = 399;					// Set prescaler to divide clock frequency (From PS)
	TIM15->ARR = 99;					// Set auto-reload value for desired timer period (From PS)

	TIM15->CCMR1 &= ~(0b111 << 12);		// Reset output compare mode for channel 2 (From PS)
	TIM15->CCMR1 |= (0b110 << 12);		// Configure PWM mode 1 for channel 2 (From PS)
	TIM15->CCER |= (1 << 4);			// Enable output for channel 2 (From PS)
	TIM15->CCR2 = 2;					// Set initial duty cycle for channel 2 (From PS)

	TIM15->CCMR1 &= ~(0b11);			// Reset input capture settings for channel 1 (From PS)
	TIM15->CCMR1 |= (0b01);				// Configure input capture for channel 1 (From PS)
	TIM15->CCER |= (0b1011);			// Enable input and output settings for channels (From PS)

	TIM15->DIER |= (0b11);				// Enable interrupts for input capture and timer overflow (From PS)

	TIM15->BDTR |= (1 << 15);			// Enable break and dead-time configuration (From PS)

	TIM15->CR1 |= 1;					// Start the timer (From PS)
}

// Interrupt
void init_INTERRUPT(void){
	ISER2 |= (1 << 5);				// Enable TIM15 interrupt in NVIC (From PS)
	__asm volatile("CPSIE I");		// Enable global interrupts
}

// Vibration
void vibration_handler(void){
	static uint8_t tempo_stat = 0;	// Tempo status state variable

	if(tempo_stat == 0){				// If in the initial state
		if(vibration_count > 250){		// High-intensity vibration detected
			vibration_count = 0;		// Reset vibration count
			tempo_stat = 1;				// Change state.
			tempo = 79;					// Set high duty cycle. Maximum strength for motor and buzzer.
		}
		else if(vibration_count > 80){	// Medium-intensity vibration detected
			vibration_count = 0;		// Reset vibration count
			tempo_stat = 1;				// Change state
			tempo = 29;					// Set medium duty cycle. Activate buzzer in medium strength and activate motor medium strength
		}
		else if(vibration_count > 0){	// Low-intensity vibration detected
			vibration_count = 0;		// Reset vibration count
			tempo_stat = 1;				// Change state
			tempo = 1;					// Set low duty cycle. Activate buzzer, motor still does not move.
		}
		else{
			tempo = 0;					// No vibration detected, set duty cycle to 0. Deactivate buzzer, deactivate motor.
		}
		TIM15->CCR2 = tempo;			// Update duty cycle
	}
	else{							// If in the post-processing state
		cycle_count++;				// Increment cycle count
		if(cycle_count > 50){		// After 50 cycles:
			cycle_count = 0;		// Reset cycle count
			tempo_stat = 0;			// Return to initial state
		}
	}
}

// Main
int main(void){
	init_GPIO();			// Initialize GPIO
	init_TIMER();			// Initialize Timer
	init_INTERRUPT();		// Initialize Interrupts

	while(1){
		__asm volatile("wfi");		// Wait for interrupt (low-power mode) (From PS)
	}
	return 0;		// Program should never reach here
}
