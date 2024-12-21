#include <stdint.h>

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
	volatile uint32_t BRR;
	uint32_t reserved;
	volatile uint32_t SECCFGR;

}GPIOType;

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	volatile uint32_t GTPR;
	volatile uint32_t RTOR;
	volatile uint32_t RQR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t RDR;
	volatile uint32_t TDR;
	volatile uint32_t PRESC;

}USARTType;

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	uint32_t reserved[2];
	volatile uint32_t RQR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t RDR;
	volatile uint32_t TDR;
	volatile uint32_t PRESC;

}LPUARTType;

#define USART2 ((USARTType *)0x40004400)
#define LPUART1 ((LPUARTType *) 0x40008000)
#define GPIOB ((GPIOType *) 0x42020400)
#define GPIOA ((GPIOType *)0x42020000)
#define GPIOG ((GPIOType *)0x42021800)
#define RCC_BASE  0X40021000
#define RCC_AHB2ENR      *((volatile uint32_t *)(RCC_BASE + 0x4C))
#define RCC_APB1ENR1    *((volatile uint32_t *)(RCC_BASE + 0x58))
#define RCC_APB1ENR2 *((volatile uint32_t *) 0x4002105C)
#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)
#define PWR_CR1 *((volatile uint32_t *) 0x40007000)
 #define PWR_CR2 *((volatile uint32_t *) 0x40007004)
// NVIC Registers
#define NVIC_ISER1      *((volatile uint32_t *) 0xE000E104) // Interrupt Set Enable Register for USART2
#define NVIC_ISER2      *((volatile uint32_t *) 0xE000E108)
char LPUART1_read_value = 0;
void init_gpio(void) {
	  RCC_AHB2ENR |= (1 << 0); // GPIOA  clock enable
	  RCC_AHB2ENR |= 1 << 6; // GPIOG clock enable
	   // Configure PA2 and PA3 as alternate function for USART2
	   GPIOA->MODER &= ~(3 << (2 * 2)); // Clear mode for PA2
	   GPIOA->MODER |= (2 << (2 * 2));  // Set PA2 to alternate function
	   GPIOA->MODER &= ~(3 << (3 * 2)); // Clear mode for PA3
	   GPIOA->MODER |= (2 << (3 * 2));  // Set PA3 to alternate function
	   GPIOG->MODER &= ~(3 << (7 * 2)); // Set PG7 to alternate function
	   GPIOG->MODER |= (2 << (7 * 2));  // Set PG7 to alternate function
	   GPIOG->MODER &= ~(3 << (8 * 2)); // Set PG8 to alternate function
	   GPIOG->MODER |= (2 << (8 * 2));  // Set PG8 to alternate function
}
void init_uart(void) {
	RCC_CCIPR1 &= ~(1 << 3);
	RCC_CCIPR1 |= 1 << 2;
    // Enable GPIOA clock for USART2 pins (PA2 - TX, PA3 - RX)
    RCC_APB1ENR1 |= (1 << 17); // Enable USART2 clock
    // Set alternate function 7 (AF7) for PA2 and PA3
    // Set alternate function 7 (AF7) for PA2 and PA3
      GPIOA->AFRL |= (7 << (2 * 4)); // Set PA2 (TX) to AF7
      GPIOA->AFRL |= (7 << (3 * 4)); // Set PA3 (RX) to AF7
    // Configure USART2
    USART2->BRR = 0x1A1;            // Set baud rate to 9600 for 4 MHz clock
    USART2->CR1 |= (1 << 2) | (1 << 3); // Enable RX and TX
    USART2->CR1 |= (1 << 29);        // Enable FIFO
    USART2->CR1 |= (1 << 5);        // Enable RXNE interrupt
    USART2->CR1 |= (1 << 0);        // Enable USART

    // Enable USART2 interrupt in NVIC
    NVIC_ISER1 |= (1 << 30);
}
// Utilized PS 11 code for this init_lpuart function block
void init_lpuart(void)
{
	//Enable Clock for Power Interface.
	RCC_APB1ENR1 |= 1 << 28;
	// Change the regulator mode to Low-power mode.
	PWR_CR1 |= 1 << 14;
	// Turn VDDIO2 for PG[15:2] pins.
	PWR_CR2 |= 1 << 9;
	// Select SYSCLK (=4MHz) for the clock source of Low-power UART.
	RCC_CCIPR1 &= ~(1 << 11);
	RCC_CCIPR1 |= 1 << 10;
	// Set alternate function for PG7 and PG8.
	GPIOG->MODER &= ~(0b0101 << (7 * 2));
	GPIOG->MODER |= 0b1010 << (7 * 2);
	// Connect PG7 to LPUART Tx.
	GPIOG->AFRL |= 0b1000 << (7 * 4);
	// Connect PG8 to LPUART Tx.
	GPIOG->AFRH |= 0b1000;
	// Enable Clock for LPUART.
	RCC_APB1ENR2 |= 1;
	// Set LPUART baud rate (BRR) for 115200 baud rate. 256 * 4000000 / 115200 for
	LPUART1->BRR = 8888;
	// Enable FIFO.
	LPUART1->CR1 |= 1 << 29;
	// Enable Transmitter and Receiver.
	LPUART1->CR1 |= 0b11 << 2;
	// Enable interrupt for FIFO is not empty.
	LPUART1->CR1 |= 1 << 5;
	// Enable global signalling from ISER.
	NVIC_ISER2 |= 1 << 2;
	// Enable LPUART.
	LPUART1->CR1 |= 1;
}
void USART2_IRQHandler(void) { //read from bluetooth
    if (USART2->ISR & (1 << 5)) { // RXNE (Receive Not Empty)
        char received_char = USART2->RDR; // Read received character

       LPUART1->TDR = received_char; // write to Serial Line COM console.
        }
    }
void LPUART1_IRQHandler(void) {
// Read data from the computer.
if ((LPUART1->ISR & (1 << 5)) != 0)
{
// Read the arrived value.
LPUART1_read_value = LPUART1->RDR;
// Send data to bluetooth device.
USART2->TDR = LPUART1_read_value;
}
}

void __enable_irq(void) {
__asm volatile(
			"mov r0, #0 \n\t"
			"msr primask, r0 \n\t"
		);
}
int main(void) {
	 init_gpio();
	 init_lpuart();
	    init_uart();
	    __enable_irq();
while(1){
__asm volatile("wfi");

}
}
