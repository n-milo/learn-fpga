module Memory (
    input             clk,
    input      [31:0] mem_addr,
    output reg [31:0] mem_rdata,
    input             mem_rstrb,
    input      [31:0] mem_wdata,
    input      [3:0]  mem_wmask
);

    localparam slow_bit = 13;
    reg [31:0] MEM [0:1535];

    `include "riscv_assembly.v"
    integer L1_   = 8;
    integer wait_ = 28;
    integer L2_   = 36;

    initial begin

        LI(gp,32'h400000);
        LI(a0,0);
    Label(L1_);
        SW(a0,gp,4);
        CALL(LabelRef(wait_));
        ADDI(a0,a0,1);
        J(LabelRef(L1_));

    Label(wait_);
        LI(t0,1);
        SLLI(t0,t0,slow_bit);
    Label(L2_);
        ADDI(t0,t0,-1);
        BNEZ(t0,LabelRef(L2_));
        RET();

        endASM();
    end

    wire [29:0] word_addr = mem_addr[31:2];
    always @(posedge clk) begin
        if (mem_rstrb) begin
            mem_rdata <= MEM[word_addr];
        end

        if (mem_wmask[0]) MEM[word_addr[ 7:0 ]] <= mem_wdata[ 7:0 ];
        if (mem_wmask[1]) MEM[word_addr[15:8 ]] <= mem_wdata[15:8 ];
        if (mem_wmask[2]) MEM[word_addr[23:16]] <= mem_wdata[23:16];
        if (mem_wmask[3]) MEM[word_addr[31:24]] <= mem_wdata[31:24];
    end
endmodule
