module Processor (
    input clk,
    input resetn,
    output [31:0] mem_addr,
    input  [31:0] mem_rdata,
    output        mem_rstrb,
    output [31:0] mem_wdata,
    output [3:0]  mem_wmask,
    output reg [31:0] x10 = 0
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

    wire [31:0] loadstore_addr = rs1val + (isStore ? Simm : Iimm);
    wire [15:0] LOAD_halfword = loadstore_addr[1] ? mem_rdata[31:16] : mem_rdata[15:0];
    wire [7:0] LOAD_byte = loadstore_addr[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];
    wire mem_byteAccess = funct3[1:0] == 2'b00;
    wire mem_halfwordAccess = funct3[1:0] == 2'b01;

    wire LOAD_sign = !funct3[2] & (mem_byteAccess ? LOAD_byte[7] : LOAD_halfword[15]);

    wire [31:0] LOAD_data = mem_byteAccess ? {{24{LOAD_sign}}, LOAD_byte} :
                            mem_halfwordAccess ? {{16{LOAD_sign}}, LOAD_halfword} :
                            mem_rdata;

    wire [3:0] STORE_mask = mem_byteAccess ?
                                (loadstore_addr[1] ?
                                    (loadstore_addr[0] ? 4'b1000 : 4'b0100) :
                                    (loadstore_addr[0] ? 4'b0010 : 4'b0001)
                                ) :
                            mem_halfwordAccess ?
                                (loadstore_addr[1] ? 4'b1100 : 4'b0011) : 4'b1111;

    assign mem_wdata[ 7:0 ] = rs2val[7:0];
    assign mem_wdata[15:8 ] = loadstore_addr[0] ? rs2val[7:0]  : rs2val[15:8];
    assign mem_wdata[23:16] = loadstore_addr[0] ? rs2val[7:0]  : rs2val[23:16];
    assign mem_wdata[31:24] = loadstore_addr[0] ? rs2val[7:0]  :
                              loadstore_addr[1] ? rs2val[15:8] : rs2val[31:24];

    wire [31:0] aluIn1 = rs1val;
    wire [31:0] aluIn2 = isALUreg | isBranch ? rs2val : Iimm;
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

    reg [31:0] aluOut;
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
    localparam LOAD = 4;
    localparam WAIT_DATA = 5;
    localparam STORE = 6;
    reg [2:0] state = IF;

    wire [31:0] PCplusImm = PC + (isJAL ? Jimm : isAUIPC ? Uimm : Bimm);
    wire [31:0] PCplus4 = PC + 4;

    assign writeBackData = (isJAL || isJALR) ? PCplus4 :
                           isLUI ? Uimm :
                           isAUIPC ? PCplusImm :
                           aluOut;

    assign writeBackEn = (state == EX && !isBranch && !isStore && !isLoad) || (state == WAIT_DATA);

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
                if (rd == 10) begin
                    x10 <= writeBackData;
                end
`ifdef VERBOSE
                $display("x%0d <= %b", rd, writeBackData);
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
                    state <= isLoad ? LOAD : (isStore ? STORE : IF);
`ifdef BENCH
                    if (isSYSTEM) $finish();
`endif
                end

                LOAD: begin
                    state <= WAIT_DATA;
                end

                WAIT_DATA: begin
                    state <= IF;
                end

                STORE: begin
                    state <= IF;
                end
            endcase
        end
    end

    assign mem_addr = (state == WAIT_IF || state == IF) ? PC : loadstore_addr;
    assign mem_rstrb = (state == IF || state == LOAD);

    assign mem_wmask = {4{(state == STORE)}} & STORE_mask;

`ifdef VERBOSE
    always @(posedge clk) begin
        $display("PC=%0d",PC);
        case (1'b1)
            isALUreg: $display("ALUreg rd=%d rs1=%d rs2=%d funct3=%b",rd, rs1, rs2, funct3);
            isALUimm: $display("ALUimm rd=%d rs1=%d imm=%0d funct3=%b",rd, rs1, Iimm, funct3);
            isBranch: $display("BRANCH");
            isJAL:    $display("JAL");
            isJALR:   $display("JALR");
            isAUIPC:  $display("AUIPC");
            isLUI:    $display("LUI");
            isLoad:   $display("LOAD");
            isStore:  $display("STORE");
            isSYSTEM: $display("SYSTEM");
        endcase
    end
`endif

endmodule
