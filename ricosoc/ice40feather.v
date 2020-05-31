/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

`ifdef PICOSOC_V
`error "ice40feather.v must be read before picosoc.v!"
`endif

`include "defines.v"

module ice40feather (
	input clk,

	output led_user,

	output ledr_n,
	output ledg_n,
	output ledb_n,

	output flash_csb,
	output flash_clk,
	inout  flash_io0,
	inout  flash_io1,
	inout  flash_io2,
	inout  flash_io3,
	inout  gpio0_pin0,
	inout  gpio0_pin1,
	inout  gpio0_pin2,
	inout  gpio0_pin3,
	inout  gpio0_pin4,
	inout  gpio0_pin5,
	inout  gpio0_pin6,
	inout  gpio0_pin7,
	inout  gpio0_pin8,
	inout  gpio0_pin9,
	inout  gpio0_pin10,
	inout  gpio0_pin11,
	inout  gpio0_pin12,
	inout  gpio0_pin13,
	inout  gpio0_pin14,
	inout  gpio0_pin15

);
	parameter integer MEM_WORDS = 32768;

// reset circutry to generate delay
// for correct ram init and reset at all
	reg [5:0] reset_cnt = 0;
	wire resetn = &reset_cnt;

	always @(posedge clk) begin
		reset_cnt <= reset_cnt + !resetn;
	end

	wire [7:0] leds;

	assign led_user = leds[0];
	assign ledr_n = !leds[1];
	assign ledg_n = !leds[2];
	assign ledb_n = !leds[3];

	wire        iomem_valid;
	wire         iomem_ready;
	wire [3:0]  iomem_wstrb;
	wire [31:0] iomem_addr;
	wire [31:0] iomem_wdata;
	wire  [31:0] iomem_rdata;

	SB_IO #(
		.PIN_TYPE(6'b 1010_01),
		.PULLUP(1'b 0)
	) flash_io_buf [3:0] (
		.PACKAGE_PIN({flash_io3, flash_io2, flash_io1, flash_io0}),
		.OUTPUT_ENABLE({flash_io3_oe, flash_io2_oe, flash_io1_oe, flash_io0_oe}),
		.D_OUT_0({flash_io3_do, flash_io2_do, flash_io1_do, flash_io0_do}),
		.D_IN_0({flash_io3_di, flash_io2_di, flash_io1_di, flash_io0_di})
	);

/*
	reg [31:0] gpio;
	assign leds = gpio;

	always @(posedge clk) begin
		if (!resetn) begin
			gpio <= 0;
		end else begin
			//iomem_ready <= 0;
			//if (iomem_valid && !iomem_ready && iomem_addr[31:24] == 8'h 03) begin
			if (gpio_sel) begin
				//iomem_ready <= 1;
				//iomem_rdata <= gpio;
				if (iomem_wstrb[0]) gpio[ 7: 0] <= iomem_wdata[ 7: 0];
				if (iomem_wstrb[1]) gpio[15: 8] <= iomem_wdata[15: 8];
				if (iomem_wstrb[2]) gpio[23:16] <= iomem_wdata[23:16];
				if (iomem_wstrb[3]) gpio[31:24] <= iomem_wdata[31:24];
			end
		end
	end
*/
	ricosoc #(
		.BARREL_SHIFTER(0),
		.ENABLE_MULDIV(0),
		.MEM_WORDS(MEM_WORDS)
	) soc (
		.clk          (clk         ),
		.resetn       (resetn      ),

		.irq_5        (1'b0        ),
		.irq_6        (1'b0        ),
		.irq_7        (1'b0        ),

		.iomem_valid  (iomem_valid ),
		.iomem_ready  (iomem_ready ),
		.iomem_wstrb  (iomem_wstrb ),
		.iomem_addr   (iomem_addr  ),
		.iomem_wdata  (iomem_wdata ),
		.iomem_rdata  (iomem_rdata ),

		.flash_csb    (flash_csb   ),
		.flash_clk    (flash_clk   ),

		.flash_io0_oe (flash_io0_oe),
		.flash_io1_oe (flash_io1_oe),
		.flash_io2_oe (flash_io2_oe),
		.flash_io3_oe (flash_io3_oe),

		.flash_io0_do (flash_io0_do),
		.flash_io1_do (flash_io1_do),
		.flash_io2_do (flash_io2_do),
		.flash_io3_do (flash_io3_do),

		.flash_io0_di (flash_io0_di),
		.flash_io1_di (flash_io1_di),
		.flash_io2_di (flash_io2_di),
		.flash_io3_di (flash_io3_di)
	);

	wire gpio0_ready;
	wire [31:0] gpio0_rdata;

	wire [15:0] alt_cpu_in;
	wire [15:0] alt_cpu_out;

	wire [15:0] i_alt_fpga_in;
	wire [15:0] i_alt_fpga_out;


	ice_gpio #(
		.PARAM_GPIO_ODR_ADDR(`GPIO0_ODR_ADDR),
		.PARAM_GPIO_IDR_ADDR(`GPIO0_IDR_ADDR),
		.PARAM_GPIO_DDR_ADDR(`GPIO0_DDR_ADDR),
		.PARAM_GPIO_MODE_ADDR(`GPIO0_MODE_ADDR)
	) gpio0_inst (
		.i_clk(clk), 
		.i_rst_n(resetn),
    	.io_memaddr(iomem_addr),
		.io_wdata(iomem_wdata),
		.io_wstrb(iomem_wstrb),
		.io_valid(iomem_valid),
		.io_rdata(gpio0_rdata),
		.io_ready(gpio0_ready),
    	.i_alt_cpu_in(16'd0),
    	.i_alt_fpga_in(16'd0),
    	.i_alt_cpu_out(),
    	.i_alt_fpga_out(),
    	.gpio({gpio0_pin15,
			   gpio0_pin14,
			   gpio0_pin13,
			   gpio0_pin12,
			   gpio0_pin11,
			   gpio0_pin10,
			   gpio0_pin9,
			   gpio0_pin8,
			   gpio0_pin7,
			   gpio0_pin6,
			   gpio0_pin5,
			   gpio0_pin4,
			   gpio0_pin3,
			   gpio0_pin2,
			   gpio0_pin1,
			   gpio0_pin0})
	);

	wire        simpleuart_reg_div_sel = (iomem_addr == `UART0_CLKDIV_ADDR);
	wire [31:0] simpleuart_reg_div_do;

	wire        simpleuart_reg_dat_sel = (iomem_addr == `UART0_DATA_ADDR);
	wire [31:0] simpleuart_reg_dat_do;
	wire        simpleuart_reg_dat_wait;

	assign iomem_ready = simpleuart_reg_div_sel 
						 || (simpleuart_reg_dat_sel && !simpleuart_reg_dat_wait)
						 || gpio0_ready;

	assign iomem_rdata = simpleuart_reg_div_sel ? simpleuart_reg_div_do : 
						 simpleuart_reg_dat_sel ? simpleuart_reg_dat_do : 
						 gpio0_ready ? gpio0_rdata :
						 32'h0000_0000;


	wire ser_tx, ser_rx;

	simpleuart simpleuart (
		.clk         (clk         ),
		.resetn      (resetn      ),

		.ser_tx      (ser_tx      ),
		.ser_rx      (ser_rx      ),

		.reg_div_we  (simpleuart_reg_div_sel ? iomem_wstrb : 4'b 0000),
		.reg_div_di  (iomem_wdata),
		.reg_div_do  (simpleuart_reg_div_do),

		.reg_dat_we  (simpleuart_reg_dat_sel ? iomem_wstrb[0] : 1'b 0),
		.reg_dat_re  (simpleuart_reg_dat_sel && !iomem_wstrb),
		.reg_dat_di  (iomem_wdata),
		.reg_dat_do  (simpleuart_reg_dat_do),
		.reg_dat_wait(simpleuart_reg_dat_wait)
	);



endmodule
