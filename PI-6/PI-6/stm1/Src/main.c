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
#define LPUART ((LPUARTType *) 0x40008000)
#define GPIOB ((GPIOType *) 0x42020400)
#define GPIOA ((GPIOType *)0x42020000)
#define RCC_BASE  0X40021000
#define RCC_AHB2ENR      *((volatile uint32_t *)(RCC_BASE + 0x4C))
#define RCC_APB1ENR1    *((volatile uint32_t *)(RCC_BASE + 0x58))
#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)
// NVIC Registers
#define NVIC_ISER1      *((volatile uint32_t *) 0xE000E104) // Interrupt Set Enable Register for USART2



void init_gpio(void) {
	  RCC_AHB2ENR |= (3 << 0); // GPIOA & B clock enable
	  // Set PB9 & PA8 as output.
	  GPIOB->MODER &= ~(0b1111 << (8 * 2));
	  GPIOB->MODER |= 0b0101 << (8 * 2);
	   // Configure PA2 and PA3 as alternate function for USART2
	   GPIOA->MODER &= ~(3 << (2 * 2)); // Clear mode for PA2
	   GPIOA->MODER |= (2 << (2 * 2));  // Set PA2 to alternate function
	   GPIOA->MODER &= ~(3 << (3 * 2)); // Clear mode for PA3
	   GPIOA->MODER |= (2 << (3 * 2));  // Set PA3 to alternate function

}

void init_uart(void) {
	RCC_CCIPR1 &= ~(1 << 3);
	RCC_CCIPR1 |= 1 << 2;
    // Enable GPIOA clock for USART2 pins (PA2 - TX, PA3 - RX)
    RCC_APB1ENR1 |= (1 << 17); // Enable USART2 clock

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
void uart_send_char(char c) {
    while (!(USART2->ISR & (1 << 7))); // Wait until TXE is set
    USART2->TDR = c;                  // Send character
}

void uart_send_string(const char *str) {
    while (*str) {
        uart_send_char(*str++); // Send each character in the string
    }
}
void __enable_irq(void) {
__asm volatile(
			"mov r0, #0 \n\t"
			"msr primask, r0 \n\t"
		);
}

void USART2_IRQHandler(void) {
    if (USART2->ISR & (1 << 5)) { // RXNE (Receive Not Empty)
        char received_char = USART2->RDR; // Read received character

        if (received_char == 'L') {
        	 uart_send_string("LEFT\r\n");
               GPIOB->ODR |= (1 << 9);
               GPIOB->ODR  &= ~(1 << 8);
        } else if(received_char == 'R') {
        	uart_send_string("RIGHT\r\n");
        	 GPIOB->ODR |= (1 << 8);
        	 GPIOB->ODR  &= ~(1 << 9);

            }
        else if(received_char == 'S'){
        	uart_send_string("STOPPED\r\n");
        	GPIOB->ODR  &= ~(1 << 9);
        	GPIOB->ODR  &= ~(1 << 8);

        }
        else{
        	uart_send_string("INVALID\r\n");
        }
        }
    }

int main(void) {
    init_gpio();
    init_uart();
    __enable_irq();

    __asm volatile (
    		"mov r0 , #0 \n\t"
    		"msr primask ,r0"
    		);
    while (1) {
    	__asm volatile("wfi");

    }
}
