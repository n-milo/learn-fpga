`include "clockworks.v"
`include "Memory.v"
`include "Processor.v"

module SOC (
    input            CLK,
    input            RESET,
    output reg [4:0] LEDS,
    input            RXD,
    output           TXD
);

    wire clk;
    wire resetn;

    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire mem_rstrb;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wmask;

    wire [31:0] x10;

    Processor CPU(
        .clk(clk),
        .resetn(resetn),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb),
        .mem_wdata(mem_wdata),
        .mem_wmask(mem_wmask),
        .x10(x10)
    );

    wire [31:0] RAM_rdata;
    wire [29:0] mem_wordaddr = mem_addr[31:2];
    wire isIO = mem_addr[22];
    wire isRAM = !isIO;
    wire mem_wstrb = |mem_wmask;

    Memory RAM(
        .clk(clk),
        .mem_addr(mem_addr),
        .mem_rdata(RAM_rdata),
        .mem_rstrb(isRAM & mem_rstrb),
        .mem_wdata(mem_wdata),
        .mem_wmask({4{isRAM}} & mem_wmask)
    );

    localparam IO_LEDS_bit = 0;
    always @(posedge clk) begin
        if (isIO & mem_wstrb & mem_wordaddr[IO_LEDS_bit]) begin
            LEDS <= mem_wdata;
        end
    end

    assign mem_rdata = isRAM ? RAM_rdata : 0;

    Clockworks CW(
        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)
    );

    assign TXD = 1'b0;

endmodule
