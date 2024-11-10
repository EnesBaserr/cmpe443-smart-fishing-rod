#include <stdint.h>
#define RCC_AHB2ENR *((volatile uint32_t *) (0x40021000 + 0x4C))
#define GPIOB_MODER *((volatile uint32_t *) 0x42020400)
#define GPIOB_ODR *((volatile uint32_t *) (0x42020400 + 0x14))
#define GPIOB_IDR *((volatile uint32_t *) (0x42020400 + 0x10))
int main(void)
{
	// Turn on the GPIO ports.
	 RCC_AHB2ENR|= (1 << 1);

	 GPIOB_MODER &= ~(0x03 <<(11 * 2));		// PB11 as input mode
	 GPIOB_MODER &= ~(0x03 <<(8 * 2));		// PB8 clear bits
	 GPIOB_MODER |= (0x01 << (8 * 2));		// PB8 as output mode

	 GPIOB_MODER &= ~(0x03 <<(9 * 2));		// PB9 as input mode
	 GPIOB_MODER &= ~(0x03 <<(10 * 2));		// PB10 clear bits
	 GPIOB_MODER |= (0x01 << (10 * 2));		// PB10 as output mode

	 GPIOB_MODER &= ~(0x03 <<(6 * 2));		// PB6 clear bits
	 GPIOB_MODER |= (0x01 << (6 * 2));		// PB6 as output mode

	 GPIOB_ODR |= (1 << 6);

	 while(1) {
		 // If read value is 1 from PB11(Switch 1), turn on the buzzer.
		 if ((GPIOB_IDR & (1 << 11)) == 0) {
			 GPIOB_ODR |= (1 << 8); // buzzer on
			 if ((GPIOB_IDR & (1 << 9)) == 0){  // If read value is 1 from PB9(Switch 2), turn on the dc motor.
				 GPIOB_ODR |= (1 << 10); // Turn on the dc motor.
			 }

			 else{
				 GPIOB_ODR &= ~(1 << 10); // If read value is 0 from PB9, turn off the dc motor.

			 }
		 }
		 else{
			 // If read value is 0 from PB11, turn off buzzer and the motor.
			 GPIOB_ODR &= ~(1 << 8); // Turn of the buzzer
			 GPIOB_ODR &= ~(1 << 10); // Turn of the motor
		 }
	 }
 }
