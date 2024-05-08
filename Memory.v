module Memory (
    input clk,
    input [31:0] mem_addr,
    output reg [31:0] mem_rdata,
    input mem_rstrb
);

    localparam slow_bit = 19;
    reg [31:0] MEM [0:255];

    `include "riscv_assembly.v"
    integer L0_ = 4;
    integer wait_ = 20;
    integer L1_ = 28;

    initial begin
        ADD(x10,x0,x0);
    Label(L0_);
        ADDI(x10,x10,1);
        JAL(x1,LabelRef(wait_));
        JAL(zero,LabelRef(L0_));

        EBREAK();

    Label(wait_);
        ADDI(x11,x0,1);
        SLLI(x11,x11,slow_bit);
    Label(L1_);
        ADDI(x11,x11,-1);
        BNE(x11,x0,LabelRef(L1_));
        JALR(x0,x1,0);

        endASM();
    end

    always @(posedge clk) begin
        if (mem_rstrb) begin
            mem_rdata <= MEM[mem_addr[31:2]];
        end
    end
endmodule
