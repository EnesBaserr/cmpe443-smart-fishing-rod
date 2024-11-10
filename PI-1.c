#include <stdint.h>
#define wait_millisecond 1000
uint32_t wait_counter = 0;
#define RCC_AHB2ENR *((volatile uint32_t *) (0x40021000 + 0x4C))
#define GPIOB_MODER *((volatile uint32_t *) 0x42020400)
#define GPIOB_ODR *((volatile uint32_t *) (0x42020400 + 0x14))
#define GPIOB_IDR *((volatile uint32_t *) (0x42020400 + 0x10))
int main(void)
{
	// Turn on the GPIO ports.
	RCC_AHB2ENR|= 0x07;

	 // To access the Red Light. Set GPIO mode output. PA9
	 GPIOB_MODER &= ~(0x03 <<(11 * 2));		// PB11 as input mode
	 GPIOB_MODER &= ~(0x03 <<(8 * 2));		// PB8 clear bits
	 GPIOB_MODER |= (0x01 << (8 * 2));		// PB8 as output mode

	 GPIOB_MODER &= ~(0x03 <<(9 * 2));		// PB9 as input mode
	 GPIOB_MODER &= ~(0x03 <<(10 * 2));		// PB10 clear bits
	 GPIOB_MODER |= (0x01 << (10 * 2));		// PB10 as output mode

	 GPIOB_MODER &= ~(0x03 <<(6 * 2));		// PB6 clear bits
	 GPIOB_MODER |= (0x01 << (6 * 2));		// PB6 as output mode

	 GPIOB_ODR |= (0x01 << 6);

	 while(1) {
		 int index;
		 // If read value is 0 from PB11, turn on the buzzer. If 1, the buzzer will not be turned on.
		 if ((GPIOB_IDR & (0x1 << 11)) == 0) {
			 GPIOB_ODR |= (0x01 << 8);              // PB8 is set to 1, logic high
			 if ((GPIOB_IDR & (0x1 << 9)) == 0){    // If read value is 0 from PB9, turn on the DC motor
				 GPIOB_ODR |= (0x01 << 10);
			 }

			 else{                                  // If read value is 1 from PB9, turn off the DC motor
				 GPIOB_ODR &= ~(0x01 << 10);
			 }
		 }

		 else{                                      // If read value is 1 from PB11, turn off the buzzer
			 GPIOB_ODR &= ~(0x01 << 8);
			 GPIOB_ODR &= ~(0x01 << 10);
		 }


//		 skip_lights:;

//		 GPIOB_ODR &= ~(0x01 << 8);
//		 GPIOB_ODR &= ~(0x01 << 10);
//
//		 for (index = 0; index < wait_millisecond * 83; index ++) {
//			 wait_counter = wait_counter + 1;
//		 }
	 }
 }
