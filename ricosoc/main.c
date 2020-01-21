#include <stdint.h>
#include <stdbool.h>

#define reg_spictrl (*(volatile uint32_t*)0x02000100)
#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000000)
#define reg_uart_data (*(volatile uint32_t*)0x02000004)
#define reg_gpio (*(volatile uint32_t*)0x02000200)

uint32_t defaultVar = 0xDEADBEEFu;
uint32_t testVar0;
uint32_t testVar1 = 0u;

uint8_t testString[] = "123 Check\r";
uint8_t testArray[12];

void main(void) {

    uint32_t ptr = 0u;
	reg_uart_clkdiv = 104;
    reg_gpio = 1;

    while(testString[ptr]) {
        testArray[ptr] = testString[ptr];
        reg_uart_data = testString[ptr];
        ptr++;
    }

    reg_gpio = 0xFFFFFFFFu;

    while(1) {
        defaultVar++;
        reg_spictrl = defaultVar;
        reg_gpio = defaultVar;
    }
}