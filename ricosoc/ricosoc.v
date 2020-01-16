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

`define RAM_ADDR_START  32'h0000_0000
`define RAM_ADDR_END    32'h00FF_FFFF+1'b1
`define ROM_ADDR_START  32'h0100_0000
`define ROM_ADDR_END    32'h01FF_FFFF+1'b1
`define RICOSOC_EXTFLASH_START_ADDR     32'h0200_0000

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
	input  irq_7

);
	parameter [0:0] BARREL_SHIFTER = 1;
	parameter [0:0] ENABLE_MULDIV = 1;
	parameter [0:0] ENABLE_COMPRESSED = 1;
	parameter [0:0] ENABLE_COUNTERS = 1;
	parameter [0:0] ENABLE_IRQ_QREGS = 0;

	parameter integer MEM_WORDS = 256;
	parameter [31:0] STACKADDR = (`RAM_ADDR_START +(4 * MEM_WORDS));       // end of ram
	parameter [31:0] PROGADDR_RESET = `ROM_ADDR_START; // 1 MB into flash
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

    reg rom_ready;
	wire [31:0] rom_rdata;

	always @(posedge clk)
		ram_ready <= mem_valid && !mem_ready && (mem_addr < 4 * MEM_WORDS); //RAM_ADDR_END set actual define

    always @(posedge clk)
		rom_ready <= mem_valid && !mem_ready && (mem_addr < `ROM_ADDR_END); //4*MEM_WORDS; set actual size
    
    // External memory interface request valid only if address is outside ram and rom
	assign iomem_valid = mem_valid && (mem_addr > `RICOSOC_EXTFLASH_START_ADDR);
	assign iomem_wstrb = mem_wstrb;
	assign iomem_addr = mem_addr;
	assign iomem_wdata = mem_wdata;

    // this won't work because it is in external address now :)
    /*
	wire        simpleuart_reg_div_sel = mem_valid && (mem_addr == 32'h 0200_0004);
	wire [31:0] simpleuart_reg_div_do;

	wire        simpleuart_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0008);
	wire [31:0] simpleuart_reg_dat_do;
	wire        simpleuart_reg_dat_wait;
    */

	assign mem_ready = ram_ready  || rom_ready || (iomem_valid && iomem_ready);
	//assign mem_ready = (iomem_valid && iomem_ready)  ||  ram_ready  || rom_ready;// ||
			//simpleuart_reg_div_sel || (simpleuart_reg_dat_sel && !simpleuart_reg_dat_wait);

	assign mem_rdata = ram_ready ? ram_rdata : rom_ready ? rom_rdata : (iomem_valid && iomem_ready) ? iomem_rdata : 32'h 0000_0000;
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
        .ROM_FILE_NAME("dummy.hex"),
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

