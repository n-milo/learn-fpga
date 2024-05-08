module Clockworks
(
    input   CLK,
    input   RESET,
    output  clk,
    output  resetn
);

    parameter SLOW=0;

    reg [SLOW:0] slow_CLK = 0;
    always @(posedge CLK) begin
        slow_CLK <= slow_CLK + 1;
    end

    assign clk = slow_CLK[SLOW];

endmodule