`define INITIALIZE_RAM

module rom (clk, raddr, rdata);
    input clk;
    input [31:0] raddr;
    output [31:0] rdata;

    parameter ROM_FILE_NAME = "dummy.hex";
    parameter RAM_DATA_WIDTH = 32;
    parameter RAM_ADDR_WIDTH = 8;

    ram #(.RAM_DATA_WIDTH(32),
          .RAM_ADDR_WIDTH(8),
          .ROM_FILE_NAME(ROM_FILE_NAME))
    ram_inst1 (clk, 1'b0, 1'b1, RAM_ADDR_WIDTH'd0, raddr[RAM_ADDR_WIDTH-1:0], 32'd0, rdata);

endmodule

module ram (clk, wen, ren, waddr, raddr, wdata, rdata);
    parameter RAM_DATA_WIDTH = 32;
    parameter RAM_ADDR_WIDTH = 8;
    parameter RAM_NUM_WORDS = (1 << RAM_ADDR_WIDTH);
    parameter ROM_FILE_NAME = "dummy.hex";

    input clk, wen, ren;
    input [RAM_ADDR_WIDTH-1:0] waddr, raddr;
    input [31:0] wdata;
    output reg [31:0] rdata;
    
    reg [31:0] mem [0:255];


`ifdef INITIALIZE_RAM
    initial $readmemh(ROM_FILE_NAME, mem);
`endif

    always @(posedge clk) begin
            if (wen)
                    mem[waddr] <= wdata;
            if (ren)
                    rdata <= mem[raddr];
    end

endmodule

module rom_test (clk, rdata);
    input clk;
    output [31:0] rdata;

    rom rom_inst1(clk, 8'd0, rdata);

endmodule
