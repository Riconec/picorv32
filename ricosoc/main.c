#include <stdint.h>
#include <stdbool.h>
#include "ice40feather_def.h"

uint32_t defaultVar = 0xDEADBEEFu;
uint32_t testVar0;
uint32_t testVar1 = 0u;

uint8_t testString[] = "123 Check\r";
uint8_t testArray[12];

void main(void) {

    uint32_t ptr = 0u;
	reg_uart_clkdiv = 104;
    //reg_gpio = 1;
/*
    while(testString[ptr]) {
        testArray[ptr] = testString[ptr];
        reg_uart_data = testString[ptr];
        ptr++;
    }
*/
    reg_gpio0_odr = 0x0u;
    reg_gpio0_idr = 0;
    reg_gpio0_ddr = 0;
    reg_gpio0_mode = 0;

    reg_gpio0_ddr = 3;
    reg_gpio0_odr = 3;

    while(1) {
        defaultVar++;
        //reg_spictrl = defaultVar;
        //reg_gpio = defaultVar;
    }
}