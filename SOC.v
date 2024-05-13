`include "clockworks.v"
`include "Memory.v"
`include "Processor.v"
`include "emitter_uart.v"

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
    localparam IO_UART_DATA_bit = 1;
    localparam IO_UART_CTRL_bit = 2;

    always @(posedge clk) begin
        if (isIO & mem_wstrb & mem_wordaddr[IO_LEDS_bit]) begin
            LEDS <= mem_wdata;
        end
    end

    wire uart_valid = isIO & mem_wstrb & mem_wordaddr[IO_UART_DATA_bit];
    wire uart_ready;

    corescore_emitter_uart #(
        .clk_freq_hz(`BOARD_FREQ*1000000),
        .baud_rate(115200)
    ) UART(
        .i_clk(clk),
        .i_rst(!resetn),
        .i_data(mem_wdata[7:0]),
        .i_valid(uart_valid),
        .o_ready(uart_ready),
        .o_uart_tx(TXD)
    );

    wire [31:0] IO_rdata = mem_wordaddr[IO_UART_CTRL_bit] ? {22'b0, !uart_ready, 9'b0} : 32'b0;

    assign mem_rdata = isRAM ? RAM_rdata : IO_rdata;

`ifdef BENCH
    always @(posedge clk) begin
        if (uart_valid) begin
            $write("%c", mem_wdata[7:0]);
            $fflush(32'h8000_0001);
        end
    end
`endif

    Clockworks CW(
        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)
    );

    assign TXD = 1'b0;

endmodule
