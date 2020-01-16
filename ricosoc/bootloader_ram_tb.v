`timescale 1 ns / 1 ps

module rom_tb;
    parameter ROM_FILE_NAME = "dummy.hex";
    parameter RAM_DATA_WIDTH = 32;
    parameter RAM_ADDR_WIDTH = 8;

    reg clk;
    reg [RAM_ADDR_WIDTH-1:0] raddr;
    wire [RAM_DATA_WIDTH-1:0]rdata;

    always #5 clk = (clk === 1'b0);

    initial begin
        clk = 0;
        raddr = 0;
		$dumpfile("testbench_rom.vcd");
		$dumpvars(0, rom_tb);

		repeat (512) begin
			@(negedge clk);
			raddr = raddr + 1;
		end
		$finish;
	end

    
    
    rom #(.RAM_DATA_WIDTH(RAM_DATA_WIDTH), .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH), .ROM_FILE_NAME(ROM_FILE_NAME)) rom_inst_tb(clk, raddr, rdata);

endmodule