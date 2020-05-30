#ifndef ICE40_FEATHER_DEF_H
#define ICE40_FEATHER_DEF_H

#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000000)
#define reg_uart_data (*(volatile uint32_t*)0x02000004)

#define reg_spictrl (*(volatile uint32_t*)0x02000100)

#define reg_gpio0_odr (*(volatile uint32_t*)0x02000200)
#define reg_gpio0_idr (*(volatile uint32_t*)0x02000204)
#define reg_gpio0_ddr (*(volatile uint32_t*)0x02000208)
#define reg_gpio0_mode (*(volatile uint32_t*)0x0200020C)


#endif