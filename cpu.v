`include "clockworks.v"

module SOC (
    input  CLK,
    input  RESET,
    output [31:0] LEDS,
    input  RXD,
    output TXD
);

    wire clk;
    wire resetn;

    reg [31:0] leds = 0;
    assign LEDS = leds;

    reg [31:0] MEM [0:255];
    reg [31:0] PC = 0;
    reg [31:0] instr;

    `include "riscv_assembly.v"
    integer L0_ = 8;
    initial begin
        ADD(x1,x0,x0);
        ADDI(x2,x0,5);
    Label(L0_);
        ADDI(x1,x1,1);
        BNE(x1,x2,LabelRef(L0_));
        EBREAK();
        endASM();
    end

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
    wire [31:0] aluIn2 = isALUreg ? rs2val : Iimm;
    reg [31:0] aluOut;
    wire [4:0] shiftAmt = isALUreg ? rs2val[4:0] : instr[24:20];

    always @(*) begin
        case (funct3)
            3'b000: aluOut = (funct7[5] & instr[5]) ? (aluIn1 - aluIn2) : (aluIn1 + aluIn2);
            3'b001: aluOut = aluIn1 << shiftAmt;
            3'b010: aluOut = ($signed(aluIn1) < $signed(aluIn2));
            3'b011: aluOut = (aluIn1 < aluIn2);
            3'b100: aluOut = (aluIn1 ^ aluIn2);
            3'b101: aluOut = funct7[5] ? ($signed(aluIn1) >>> shiftAmt) : (aluIn1 >> shiftAmt);
            3'b110: aluOut = (aluIn1 | aluIn2);
            3'b111: aluOut = (aluIn1 & aluIn2);
        endcase
    end

    reg takeBranch;
    always @(*) begin
        case (funct3)
            3'b000: takeBranch = (rs1val == rs2val);
            3'b001: takeBranch = (rs1val != rs2val);
            3'b100: takeBranch = ($signed(rs1val) < $signed(rs2val));
            3'b101: takeBranch = ($signed(rs1val) == $signed(rs2val));
            3'b110: takeBranch = (rs1val < rs2val);
            3'b111: takeBranch = (rs1val >= rs2val);
            default: takeBranch = 1'b0;
        endcase
    end

    localparam IF = 0;
    localparam ID = 1;
    localparam EX = 2;
    reg [1:0] state = IF;

    assign writeBackData = aluOut;
    wire isWriteBack = isALUreg || isALUimm || isJAL || isJALR;
    assign writeBackEn = (state == EX && isWriteBack);

    wire [31:0] nextPC = (isBranch && takeBranch) ? PC+Bimm :
                         isJAL ? PC+Jimm :
                         isJALR ? rs1val+Iimm :
                         PC+4;

    always @(posedge clk) begin
        if (!resetn) begin
            PC <= 0;
            state <= IF;
        end else begin
            if (writeBackEn && rd != 0) begin
                RegisterBank[rd] <= writeBackData;
                if (rd == 1) begin
                    leds <= writeBackData;
                end
`ifdef BENCH
                $display("x%0d <= %b", rd, writeBackData);
`endif
            end
            case (state)
                IF: begin
                    instr <= MEM[PC[31:2]];
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

`ifdef BENCH
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

    Clockworks #(
        .SLOW(18)
    )CW(
        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)
    );

    assign TXD = 1'b0;

endmodule
