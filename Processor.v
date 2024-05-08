module Processor (
    input clk,
    input resetn,
    output [31:0] mem_addr,
    input  [31:0] mem_rdata,
    output        mem_rstrb,
    output reg [31:0] x1
);

    reg [31:0] PC = 0;
    reg [31:0] instr;

    // https://github.com/riscv/riscv-isa-manual/releases/download/Ratified-IMAFDQC/riscv-spec-20191213.pdf
    // pg 130
    wire [6:0] opcode = instr[6:0];
    wire isLUI    = (opcode == 7'b0110111);
    wire isAUIPC  = (opcode == 7'b0010111);
    wire isJAL    = (opcode == 7'b1101111);
    wire isJALR   = (opcode == 7'b1100111);
    wire isBranch = (opcode == 7'b1100011);
    wire isLoad   = (opcode == 7'b0000011);
    wire isStore  = (opcode == 7'b0100011);
    wire isALUimm = (opcode == 7'b0010011);
    wire isALUreg = (opcode == 7'b0110011);
    wire isFENCE  = (opcode == 7'b0001111);
    wire isSYSTEM = (opcode == 7'b1110011);

    wire [4:0] rs2 = instr[24:20];
    wire [4:0] rs1 = instr[19:15];
    wire [4:0] rd  = instr[11:7];
    wire [2:0] funct3 = instr[14:12];
    wire [6:0] funct7 = instr[31:25];

    // immediate is [31:20] but sign extended to 32 bit
    // so repeat instr[31] 21 times and append to the bottom 11 bits
    wire [31:0] Iimm = {{21{instr[31]}}, instr[30:20]};

    // similar logic for other immediate values
    wire [31:0] Uimm = {instr[31:12], {12{1'b0}}};
    wire [31:0] Simm= {{21{instr[31]}}, instr[30:25], instr[11:7]};
    wire [31:0] Bimm= {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
    wire [31:0] Jimm= {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

    reg [31:0] RegisterBank [0:31];
    reg [31:0] rs1val;
    reg [31:0] rs2val;
    wire [31:0] writeBackData;
    wire writeBackEn;

`ifdef BENCH
    integer     i;
    initial begin
        for(i=0; i<32; ++i) begin
            RegisterBank[i] = 0;
        end
    end
`endif

    wire [31:0] aluIn1 = rs1val;
    wire [31:0] aluIn2 = isALUreg | isBranch ? rs2val : Iimm;
    reg [31:0] aluOut;
    wire [4:0] shiftAmt = isALUreg ? rs2val[4:0] : instr[24:20];

    // optimization: we can do the subtraction in 33 bits
    // then use minus[32] to test for comparisons
    wire [32:0] aluMinus = {1'b0, aluIn1} - {1'b0, aluIn2};
    wire EQ  = (aluMinus[31:0] == 0);
    wire LTU = aluMinus[32];
    wire LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32];

    wire [31:0] aluPlus = aluIn1 + aluIn2;

    function [31:0] flip32;
        input [31:0] x;
        flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
                  x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
                  x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
                  x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
    endfunction

    wire [31:0] shifter_in = (funct3 == 3'b001) ? flip32(aluIn1) : aluIn1;
    wire [31:0] shifter = $signed({instr[30] & aluIn1[31], shifter_in}) >>> aluIn2[4:0];
    wire [31:0] leftshift = flip32(shifter);

    always @(*) begin
        case (funct3)
            3'b000: aluOut = (funct7[5] & instr[5]) ? aluMinus[31:0] : aluPlus;
            3'b001: aluOut = leftshift;
            3'b010: aluOut = {31'b0, LT};
            3'b011: aluOut = {31'b0, LTU};
            3'b100: aluOut = (aluIn1 ^ aluIn2);
            3'b101: aluOut = shifter;
            3'b110: aluOut = (aluIn1 | aluIn2);
            3'b111: aluOut = (aluIn1 & aluIn2);
        endcase
    end

    reg takeBranch;
    always @(*) begin
        case (funct3)
            3'b000: takeBranch = EQ;
            3'b001: takeBranch = !EQ;
            3'b100: takeBranch = LT;
            3'b101: takeBranch = !LT;
            3'b110: takeBranch = LTU;
            3'b111: takeBranch = !LTU;
            default: takeBranch = 1'b0;
        endcase
    end

    localparam IF = 0;
    localparam WAIT_IF = 1;
    localparam ID = 2;
    localparam EX = 3;
    reg [1:0] state = IF;

    wire [31:0] PCplusImm = PC + (isJAL ? Jimm : isAUIPC ? Uimm : Bimm);
    wire [31:0] PCplus4 = PC + 4;

    assign writeBackData = (isJAL || isJALR) ? PCplus4 :
                           isLUI ? Uimm :
                           isAUIPC ? PCplusImm :
                           aluOut;

    wire isWriteBack = isALUreg || isALUimm || isJAL || isJALR || isLUI || isAUIPC;
    assign writeBackEn = (state == EX && isWriteBack);

    wire [31:0] nextPC = ((isBranch && takeBranch) || isJAL) ? PCplusImm :
                         isJALR ? {aluPlus[31:1], 1'b0} :
                         PCplus4;

    always @(posedge clk) begin
        if (!resetn) begin
            PC <= 0;
            state <= IF;
        end else begin
            if (writeBackEn && rd != 0) begin
                RegisterBank[rd] <= writeBackData;
                if (rd == 1) begin
                    x1 <= writeBackData;
                end
`ifdef BENCH
                // $display("x%0d <= %b", rd, writeBackData);
`endif
            end
            case (state)
                IF: begin
                    state <= WAIT_IF;
                end

                WAIT_IF: begin
                    instr <= mem_rdata;
                    state <= ID;
                end

                ID: begin
                    rs1val <= RegisterBank[rs1];
                    rs2val <= RegisterBank[rs2];
                    state <= EX;
                end

                EX: begin
                    PC <= nextPC;
                    state <= IF;
`ifdef BENCH
                    if (isSYSTEM) $finish();
`endif
                end
            endcase
        end
    end

    assign mem_addr = PC;
    assign mem_rstrb = (state == IF);

`ifdef BENCH
//    always @(posedge clk) begin
//        $display("PC=%0d",PC);
//        case (1'b1)
//            isALUreg: $display("ALUreg rd=%d rs1=%d rs2=%d funct3=%b",rd, rs1, rs2, funct3);
//            isALUimm: $display("ALUimm rd=%d rs1=%d imm=%0d funct3=%b",rd, rs1, Iimm, funct3);
//            isBranch: $display("BRANCH");
//            isJAL:    $display("JAL");
//            isJALR:   $display("JALR");
//            isAUIPC:  $display("AUIPC");
//            isLUI:    $display("LUI");
//            isLoad:   $display("LOAD");
//            isStore:  $display("STORE");
//            isSYSTEM: $display("SYSTEM");
//        endcase
//    end
`endif

endmodule
