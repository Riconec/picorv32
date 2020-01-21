#include <stdint.h>
#include <stdbool.h>

#define reg_spictrl (*(volatile uint32_t*)0x02000000)
uint32_t defaultVar = 0xDEADBEEFu;
uint32_t testVar0;
uint32_t testVar1 = 0u;

uint8_t testString[] = "123 Check\r";
uint8_t testArray[12];

void main(void) {

    uint32_t ptr = 0u;
    while(testString[ptr]) {
        testArray[ptr] = testString[ptr];
        ptr++;
    }

    while(1) {
        defaultVar++;
        reg_spictrl = defaultVar;
    }
}