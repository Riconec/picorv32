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

`ifndef PICORV32_REGS
`ifdef PICORV32_V
`error "picosoc.v must be read before picorv32.v!"
`endif

`define PICORV32_REGS picosoc_regs
`endif

`ifndef PICOSOC_MEM
`define PICOSOC_MEM picosoc_mem
`endif

// this macro can be used to check if the verilog files in your
// design are read in the correct order.
`define PICOSOC_V

/* This part later will be transfered to separate include file */

`define RICOSOC_RAM_START_ADDR          32'h0000_0000
`define RICOSOC_RAM_END_ADDR            32'h0000_FFFF

`define RICOSOC_EXECRAM_START_ADDR      32'h0001_0000
`define RICOSOC_EXECRAM_END_ADDR        32'h0001_FFFF
`define RICOSOC_INTRAM_END_ADDR         `RICOSOC_EXECRAM_END_ADDR

`define RICOSOC_EXTRAM_START_ADDR       32'h0002_0000
`define RICOSOC_EXTRAM_END_ADDR         32'h0081_FFFF

`define RICOSOC_ROM_START_ADDR          32'h0100_0000
`define RICOSOC_ROM_END_ADDR            32'h0100_FFFF
`define RICOSOC_EXTFLASH_START_ADDR     32'h0101_0000
`define RICOSOC_EXTFLASH_END_ADDR       32'h01FF_FFFF

`define RICOSOC_PERIPH_START_ADDR       32'h0200_0000
`define RICOSOC_PERIPH_END_ADDR         32'h02FF_FFFF
`define RICOSOC_PERIPH_PAGE_SIZE        32'h0200_0100

`define RICOSOC_UART0_PERIPH_NUM        0
`define RICOSOC_UART0_BASE_ADDR         `RICOSOC_PERIPH_START_ADDR + (`RICOSOC_PERIPH_PAGE_SIZE * `RICOSOC_UART0_PERIPH_NUM)
`define RICOSOC_UARTx_CLKDIV_ADDR       8'h00
`define RICOSOC_UARTx_DATA_ADDR         8'h04

`define RICOSOC_SPIEX_PERIPH_NUM        0
`define RICOSOC_SPIEX_BASE_ADDR         `RICOSOC_PERIPH_START_ADDR + (`RICOSOC_PERIPH_PAGE_SIZE * `RICOSOC_SPIEX_PERIPH_NUM)
`define RICOSOC_SPIEX_CFGREG_ADDR       8'h00

`define RICOSOC_GPIO0_PERIPH_NUM        2
`define RICOSOC_GPIO0_BASE_ADDR         `RICOSOC_PERIPH_START_ADDR + (`RICOSOC_GPIO0_PERIPH_NUM * `RICOSOC_UART0_PERIPH_NUM)
`define RICOSOC_GPIO0_PERIPH_NUM        3
`define RICOSOC_GPIO1_BASE_ADDR         `RICOSOC_PERIPH_START_ADDR + (`RICOSOC_GPIO0_PERIPH_NUM * `RICOSOC_UART0_PERIPH_NUM)
`define RICOSOC_GPIOx_PORT_ADDR         8'h00
`define RICOSOC_GPIOx_DDR_ADDR          8'h04
`define RICOSOC_GPIOx_MODE_ADDR         8'h08


`define UART0_CLKDIV_ADDR               (RICOSOC_UART0_BASE_ADDR + RICOSOC_UARTx_CLKDIV_ADDR)
`define UART0_DATA_ADDR                 (RICOSOC_UART0_BASE_ADDR + RICOSOC_UARTx_DATA_ADDR)

`define GPIO0_PORT_ADDR                 (RICOSOC_GPIO0_BASE_ADDR + RICOSOC_GPIOx_PORT_ADDR)
`define GPIO0_DDR_ADDR                  (RICOSOC_GPIO0_BASE_ADDR + RICOSOC_GPIOx_DDR_ADDR)
`define GPIO0_MODE_ADDR                 (RICOSOC_GPIO0_BASE_ADDR + RICOSOC_GPIOx_MODE_ADDR)

`define GPIO1_PORT_ADDR                 (RICOSOC_GPIO1_BASE_ADDR + RICOSOC_GPIOx_PORT_ADDR)
`define GPIO1_DDR_ADDR                  (RICOSOC_GPIO1_BASE_ADDR + RICOSOC_GPIOx_DDR_ADDR)
`define GPIO1_MODE_ADDR                 (RICOSOC_GPIO1_BASE_ADDR + RICOSOC_GPIOx_MODE_ADDR)

`define UART0_DATA_ADDR                 (RICOSOC_GPIO0_BASE_ADDR + RICOSOC_UARTx_CLKDIV_ADDR)



module ricosoc (
	input clk,
	input resetn,

	output        iomem_valid,
	input         iomem_ready,
	output [ 3:0] iomem_wstrb,
	output [31:0] iomem_addr,
	output [31:0] iomem_wdata,
	input  [31:0] iomem_rdata,

	input  irq_5,
	input  irq_6,
	input  irq_7,

	output flash_csb,
	output flash_clk,

	output flash_io0_oe,
	output flash_io1_oe,
	output flash_io2_oe,
	output flash_io3_oe,

	output flash_io0_do,
	output flash_io1_do,
	output flash_io2_do,
	output flash_io3_do,

	input  flash_io0_di,
	input  flash_io1_di,
	input  flash_io2_di,
	input  flash_io3_di

);
	parameter [0:0] BARREL_SHIFTER = 1;
	parameter [0:0] ENABLE_MULDIV = 1;
	parameter [0:0] ENABLE_COMPRESSED = 1;
	parameter [0:0] ENABLE_COUNTERS = 1;
	parameter [0:0] ENABLE_IRQ_QREGS = 0;

	parameter integer MEM_WORDS = 256;
	parameter [31:0] STACKADDR = (`RICOSOC_RAM_START_ADDR +(4 * MEM_WORDS));       // end of ram
	parameter [31:0] PROGADDR_RESET = `RICOSOC_ROM_START_ADDR; 
	parameter [31:0] PROGADDR_IRQ = 32'h 0000_0000;

	reg [31:0] irq;
	wire irq_stall = 0;
	wire irq_uart = 0;

	always @* begin
		irq = 0;
		irq[3] = irq_stall;
		irq[4] = irq_uart;
		irq[5] = irq_5;
		irq[6] = irq_6;
		irq[7] = irq_7;
	end

	wire mem_valid;
	wire mem_instr;
	wire mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	wire [31:0] mem_rdata;

	reg ram_ready;
	wire [31:0] ram_rdata;

	always @(posedge clk)
		ram_ready <= mem_valid && !mem_ready && (mem_addr >= `RICOSOC_RAM_START_ADDR) && (mem_addr <= `RICOSOC_RAM_END_ADDR);

    reg rom_ready;
	wire [31:0] rom_rdata;

    always @(posedge clk)
		rom_ready <= mem_valid && !mem_ready && (mem_addr >= `RICOSOC_ROM_START_ADDR) && (mem_addr <= `RICOSOC_ROM_END_ADDR);

    wire spimem_ready;
	wire [31:0] spimem_rdata;

    
    // External memory interface request valid only if address is outside ram and rom
    // simple check before other required
	assign iomem_valid = mem_valid && (mem_addr > `RICOSOC_EXTFLASH_START_ADDR);
	assign iomem_wstrb = mem_wstrb;
	assign iomem_addr = mem_addr;
	assign iomem_wdata = mem_wdata;

    wire spimemio_cfgreg_sel = mem_valid && (mem_addr == (`RICOSOC_SPIEX_BASE_ADDR + `RICOSOC_SPIEX_CFGREG_ADDR));
	wire [31:0] spimemio_cfgreg_do;

    // this won't work because it is in external address now :)
    /*
	wire        simpleuart_reg_div_sel = mem_valid && (mem_addr == 32'h 0200_0004);
	wire [31:0] simpleuart_reg_div_do;

	wire        simpleuart_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0008);
	wire [31:0] simpleuart_reg_dat_do;
	wire        simpleuart_reg_dat_wait;
    */

	assign mem_ready = ram_ready  || rom_ready || (iomem_valid && iomem_ready) || spimemio_cfgreg_sel;

	assign mem_rdata = ram_ready ? ram_rdata : rom_ready ? rom_rdata : spimemio_cfgreg_sel ? spimemio_cfgreg_do : (iomem_valid && iomem_ready) ? iomem_rdata : 32'h 0000_0000;
	//assign mem_rdata = (iomem_valid && iomem_ready) ? iomem_rdata : ram_ready ? ram_rdata : rom_ready ? rom_rdata : 32'h 0000_0000;
			//simpleuart_reg_div_sel ? simpleuart_reg_div_do :
			//simpleuart_reg_dat_sel ? simpleuart_reg_dat_do : 32'h 0000_0000;

	picorv32 #(
		.STACKADDR(STACKADDR),
		.PROGADDR_RESET(PROGADDR_RESET),
		.PROGADDR_IRQ(PROGADDR_IRQ),
		.BARREL_SHIFTER(BARREL_SHIFTER),
		.COMPRESSED_ISA(ENABLE_COMPRESSED),
		.ENABLE_COUNTERS(ENABLE_COUNTERS),
		.ENABLE_MUL(ENABLE_MULDIV),
		.ENABLE_DIV(ENABLE_MULDIV),
		.ENABLE_IRQ(1),
		.ENABLE_IRQ_QREGS(ENABLE_IRQ_QREGS)
	) cpu (
		.clk         (clk        ),
		.resetn      (resetn     ),
		.mem_valid   (mem_valid  ),
		.mem_instr   (mem_instr  ),
		.mem_ready   (mem_ready  ),
		.mem_addr    (mem_addr   ),
		.mem_wdata   (mem_wdata  ),
		.mem_wstrb   (mem_wstrb  ),
		.mem_rdata   (mem_rdata  ),
		.irq         (irq        )
	);

	spimemio spimemio (
		.clk    (clk),
		.resetn (resetn),
		.valid  (mem_valid && mem_addr >= 4*MEM_WORDS && mem_addr < 32'h 0200_0000),
		.ready  (spimem_ready),
		.addr   (mem_addr[23:0]),
		.rdata  (spimem_rdata),

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
		.flash_io3_di (flash_io3_di),

		.cfgreg_we(spimemio_cfgreg_sel ? mem_wstrb : 4'b 0000),
		.cfgreg_di(mem_wdata),
		.cfgreg_do(spimemio_cfgreg_do)
	);
/*
	simpleuart simpleuart (
		.clk         (clk         ),
		.resetn      (resetn      ),

		.ser_tx      (ser_tx      ),
		.ser_rx      (ser_rx      ),

		.reg_div_we  (simpleuart_reg_div_sel ? mem_wstrb : 4'b 0000),
		.reg_div_di  (mem_wdata),
		.reg_div_do  (simpleuart_reg_div_do),

		.reg_dat_we  (simpleuart_reg_dat_sel ? mem_wstrb[0] : 1'b 0),
		.reg_dat_re  (simpleuart_reg_dat_sel && !mem_wstrb),
		.reg_dat_di  (mem_wdata),
		.reg_dat_do  (simpleuart_reg_dat_do),
		.reg_dat_wait(simpleuart_reg_dat_wait)
	);
*/
	`PICOSOC_MEM #(
		.WORDS(MEM_WORDS)
	) memory (
		.clk(clk),
		.wen((mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS) ? mem_wstrb : 4'b0),
		.addr(mem_addr[23:2]),
		.wdata(mem_wdata),
		.rdata(ram_rdata)
	);

    rom #(
        .ROM_FILE_NAME("bootloader_fw.hex"),
        .RAM_ADDR_WIDTH(8)
    ) bootrom (
        .clk(clk), 
        .raddr(mem_addr),
        .rdata(rom_rdata)
    );

endmodule

// Implementation note:
// Replace the following two modules with wrappers for your SRAM cells.

module picosoc_regs (
	input clk, wen,
	input [5:0] waddr,
	input [5:0] raddr1,
	input [5:0] raddr2,
	input [31:0] wdata,
	output [31:0] rdata1,
	output [31:0] rdata2
);
	reg [31:0] regs [0:31];

	always @(posedge clk)
		if (wen) regs[waddr[4:0]] <= wdata;

	assign rdata1 = regs[raddr1[4:0]];
	assign rdata2 = regs[raddr2[4:0]];
endmodule

module picosoc_mem #(
	parameter integer WORDS = 256
) (
	input clk,
	input [3:0] wen,
	input [21:0] addr,
	input [31:0] wdata,
	output reg [31:0] rdata
);
	reg [31:0] mem [0:WORDS-1];

	always @(posedge clk) begin
		rdata <= mem[addr];
		if (wen[0]) mem[addr][ 7: 0] <= wdata[ 7: 0];
		if (wen[1]) mem[addr][15: 8] <= wdata[15: 8];
		if (wen[2]) mem[addr][23:16] <= wdata[23:16];
		if (wen[3]) mem[addr][31:24] <= wdata[31:24];
	end
endmodule

