`include "clockworks.v"

module SOC (
    input  CLK,
    input  RESET,
    output [4:0] LEDS,
    input  RXD,
    output TXD
);

    wire clk;
    wire resetn;

    reg [4:0] MEM [0:20];
    initial begin
        MEM[0]  = 5'b00001;
        MEM[1]  = 5'b00010;
        MEM[2]  = 5'b00100;
        MEM[3]  = 5'b01000;
        MEM[4]  = 5'b10000;
    end

    reg [4:0] PC = 0;
    reg [4:0] leds = 0;
    always @(posedge clk) begin
        leds <= MEM[PC];
        PC <= (!resetn || PC==4) ? 0 : (PC+1);
    end

    Clockworks #(
        .SLOW(18)
    )CW(
        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)
    );

    assign LEDS = leds;
    assign TXD = 1'b0;

endmodule
