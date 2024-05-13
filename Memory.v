module Memory (
    input             clk,
    input      [31:0] mem_addr,
    output reg [31:0] mem_rdata,
    input             mem_rstrb,
    input      [31:0] mem_wdata,
    input      [3:0]  mem_wmask
);

    localparam IO_LEDS_bit = 0;
    localparam IO_UART_DATA_bit = 1;
    localparam IO_UART_CTRL_bit = 2;

    function [31:0] IO_BIT_TO_OFFSET;
        input [31:0] bit;
        begin
            IO_BIT_TO_OFFSET = 1 << (bit + 2);
        end
    endfunction

    localparam slow_bit = 13;
    reg [31:0] MEM [0:1535];

    `include "riscv_assembly.v"
    integer L1_   = 12;
    integer wait_ = 40;
    integer L2_   = 48;
    integer putc_ = 60;
    integer putc_loop_ = 68;

    initial begin

        LI(gp,32'h400000);
        LI(a0,65);
        LI(a1,0);
    Label(L1_);
        SW(a1,gp,IO_BIT_TO_OFFSET(IO_LEDS_bit));
        CALL(LabelRef(wait_));
        ADDI(a1,a1,1);
        CALL(LabelRef(putc_));
        J(LabelRef(L1_));

    Label(wait_);
        LI(t0,1);
        SLLI(t0,t0,slow_bit);
    Label(L2_);
        ADDI(t0,t0,-1);
        BNEZ(t0,LabelRef(L2_));
        RET();

    Label(putc_);
        SW(a0, gp, IO_BIT_TO_OFFSET(IO_UART_DATA_bit));
        LI(t0, 1<<9);
    Label(putc_loop_);
        LW(t1, gp, IO_BIT_TO_OFFSET(IO_UART_DATA_bit));
        AND(t1, t1, t0);
        BNEZ(t1, LabelRef(putc_loop_));
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
