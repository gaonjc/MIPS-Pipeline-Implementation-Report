`timescale 1ns / 1ps
// Top-level module: PIPELINE
module PIPELINE (
    input wire clk,
    input wire reset
);
    // Wires connecting the stages
    wire [31:0] IF_ID_NPC, IF_ID_IR;
    wire [31:0] ID_EX_NPC, ID_EX_A, ID_EX_B, ID_EX_IMM;
    wire [4:0] ID_EX_RT, ID_EX_RD;
    wire [1:0] ID_EX_WBCTRL;
    wire [2:0] ID_EX_MCTRL;
    wire [3:0] ID_EX_EXCTRL;
    
    wire [31:0] EX_MEM_NPC, EX_MEM_ALU_RESULT, EX_MEM_B;
    wire [4:0] EX_MEM_WRITEREG;
    wire [1:0] EX_MEM_WBCTRL;
    wire [2:0] EX_MEM_MCTRL;
    wire EX_MEM_ZERO;
    
    wire [31:0] MEM_WB_ALU_RESULT, MEM_WB_READ_DATA;
    wire [4:0] MEM_WB_WRITEREG;
    wire [1:0] MEM_WB_WBCTRL;
    
    wire [31:0] WB_WRITEDATA;
    wire WB_REGWRITE;

    // PCSrc signal from MEM stage to IF stage
    wire PCSrc;
    
    wire [5:0]  ID_EX_FUNCT;
    
    // Instantiate the pipeline stages
    I_FETCH if_stage (
        .clk(clk),
        .reset(reset),
        .PCSrc(PCSrc),
        .EX_MEM_NPC(EX_MEM_NPC),
        .IF_ID_NPC(IF_ID_NPC),
        .IF_ID_IR(IF_ID_IR)
    );
    
    I_DECODE id_stage (
        .clk(clk),
        .reset(reset),
        .IF_ID_IR(IF_ID_IR),
        .IF_ID_NPC(IF_ID_NPC),
        .MEM_WB_WRITEREG(MEM_WB_WRITEREG),
        .MEM_WB_WRITEDATA(WB_WRITEDATA),
        .MEM_WB_REGWRITE(WB_REGWRITE),
        .ID_EX_NPC(ID_EX_NPC),
        .ID_EX_A(ID_EX_A),
        .ID_EX_B(ID_EX_B),
        .ID_EX_IMM(ID_EX_IMM),
        .ID_EX_FUNCT(ID_EX_FUNCT),
        .ID_EX_RT(ID_EX_RT),
        .ID_EX_RD(ID_EX_RD),
        .ID_EX_WBCTRL(ID_EX_WBCTRL),
        .ID_EX_MCTRL(ID_EX_MCTRL),
        .ID_EX_EXCTRL(ID_EX_EXCTRL)
    );
    
    I_EXECUTE ex_stage (
        .clk(clk),
        .reset(reset),
        .ID_EX_FUNCT(ID_EX_FUNCT),
        .ID_EX_NPC(ID_EX_NPC),
        .ID_EX_A(ID_EX_A),
        .ID_EX_B(ID_EX_B),
        .ID_EX_IMM(ID_EX_IMM),
        .ID_EX_RT(ID_EX_RT),
        .ID_EX_RD(ID_EX_RD),
        .ID_EX_WBCTRL(ID_EX_WBCTRL),
        .ID_EX_MCTRL(ID_EX_MCTRL),
        .ID_EX_EXCTRL(ID_EX_EXCTRL),
        .EX_MEM_NPC(EX_MEM_NPC),
        .EX_MEM_ALU_RESULT(EX_MEM_ALU_RESULT),
        .EX_MEM_B(EX_MEM_B),
        .EX_MEM_WRITEREG(EX_MEM_WRITEREG),
        .EX_MEM_WBCTRL(EX_MEM_WBCTRL),
        .EX_MEM_MCTRL(EX_MEM_MCTRL),
        .EX_MEM_ZERO(EX_MEM_ZERO)
    );
    
    MEMORY mem_stage (
        .clk(clk),
        .reset(reset),
        .EX_MEM_NPC(EX_MEM_NPC),
        .EX_MEM_ALU_RESULT(EX_MEM_ALU_RESULT),
        .EX_MEM_B(EX_MEM_B),
        .EX_MEM_WRITEREG(EX_MEM_WRITEREG),
        .EX_MEM_WBCTRL(EX_MEM_WBCTRL),
        .EX_MEM_MCTRL(EX_MEM_MCTRL),
        .EX_MEM_ZERO(EX_MEM_ZERO),
        .MEM_WB_READ_DATA(MEM_WB_READ_DATA),
        .MEM_WB_ALU_RESULT(MEM_WB_ALU_RESULT),
        .MEM_WB_WRITEREG(MEM_WB_WRITEREG),
        .MEM_WB_WBCTRL(MEM_WB_WBCTRL),
        .PCSrc(PCSrc)
    );
    
    WB wb_stage (
        .MEM_WB_READ_DATA(MEM_WB_READ_DATA),
        .MEM_WB_ALU_RESULT(MEM_WB_ALU_RESULT),
        .MEM_WB_WBCTRL(MEM_WB_WBCTRL),
        .WB_WRITEDATA(WB_WRITEDATA),
        .WB_REGWRITE(WB_REGWRITE)
    );
    
endmodule

// IF Stage: Instruction Fetch
module I_FETCH (
    input wire clk,
    input wire reset,
    input wire PCSrc,
    input wire [31:0] EX_MEM_NPC,
    output reg [31:0] IF_ID_NPC,
    output reg [31:0] IF_ID_IR
);
    reg [31:0] PC;
    wire [31:0] PC_next, PC_plus_4, instruction;
    
    // Initialize PC
    initial begin
        PC = 32'b0;
        $display("IF Stage: PC initialized to 0");
    end
    
    // PC update logic with enhanced debug
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC <= 32'b0;
            $display("IF Stage: PC reset to 0");
        end
        else begin
            PC <= PC_next;
            $display("IF Stage: PC updated from %h to %h", PC, PC_next);
        end
    end
    
    // Incrementer by 4
    INCR incrementer (
        .pcin(PC),
        .pcout(PC_plus_4)
    );
    
    // Mux for selecting next PC value
    MUX pc_mux (
        .a(EX_MEM_NPC),  // Branch target address
        .b(PC_plus_4),   // PC + 4
        .sel(PCSrc),     // Select signal
        .y(PC_next)      // Output next PC
    );
    
    // Instruction Memory
    INST_MEM instruction_memory (
        .addr(PC),
        .data(instruction)
    );
    
    // IF/ID Pipeline Register with debug
    initial begin
        IF_ID_IR = 32'b0;
        IF_ID_NPC = 32'b0;
    end
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            IF_ID_IR <= 32'b0;
            IF_ID_NPC <= 32'b0;
            $display("IF Stage: IF/ID registers reset");
        end
        else begin
            IF_ID_IR <= instruction;
            IF_ID_NPC <= PC_plus_4;
            $display("IF Stage: IF/ID_IR updated to %b, IF_ID_NPC to %h", instruction, PC_plus_4);
        end
    end
endmodule

// ID Stage: Instruction Decode
module I_DECODE (
    input wire clk,
    input wire reset,
    input wire [31:0] IF_ID_IR,
    input wire [31:0] IF_ID_NPC,
    input wire [4:0] MEM_WB_WRITEREG,
    input wire [31:0] MEM_WB_WRITEDATA,
    input wire MEM_WB_REGWRITE,
    output reg [31:0] ID_EX_NPC,
    output reg [31:0] ID_EX_A,
    output reg [31:0] ID_EX_B,
    output reg [31:0] ID_EX_IMM,
    output reg [4:0] ID_EX_RT,
    output reg [4:0] ID_EX_RD,
    output reg [1:0] ID_EX_WBCTRL,
    output reg [2:0] ID_EX_MCTRL,
    output reg [3:0] ID_EX_EXCTRL,
    output reg [5:0]  ID_EX_FUNCT
);
    wire [8:0] control_signals;
    wire [31:0] read_data_1, read_data_2, sign_extended_imm;
    
    // Control Unit
    CONTROL control_unit (
        .opcode(IF_ID_IR[31:26]), 
        .controlbits(control_signals)
    );
    
    // Register File
    REG register_file (
        .clk(clk),
        .reset(reset),
        .rs(IF_ID_IR[25:21]),
        .rt(IF_ID_IR[20:16]),
        .rd(MEM_WB_WRITEREG),
        .writedata(MEM_WB_WRITEDATA),
        .regwrite(MEM_WB_REGWRITE),
        .A(read_data_1),
        .B(read_data_2)
    );
    
    // Sign Extend Unit
    S_EXTEND sign_extend (
        .imm16(IF_ID_IR[15:0]),
        .imm32(sign_extended_imm)
    );
    
    // ID/EX Pipeline Register
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ID_EX_NPC <= 32'b0;
            ID_EX_A <= 32'b0;
            ID_EX_B <= 32'b0;
            ID_EX_IMM <= 32'b0;
            ID_EX_RT <= 5'b0;
            ID_EX_RD <= 5'b0;
            ID_EX_WBCTRL <= 2'b0;
            ID_EX_MCTRL <= 3'b0;
            ID_EX_EXCTRL <= 4'b0;
            ID_EX_FUNCT <= 6'b0;
        end
        else begin
            ID_EX_NPC <= IF_ID_NPC;
            ID_EX_A <= read_data_1;
            ID_EX_B <= read_data_2;
            ID_EX_IMM <= sign_extended_imm;
            ID_EX_RT <= IF_ID_IR[20:16];
            ID_EX_RD <= IF_ID_IR[15:11];
            ID_EX_WBCTRL <= control_signals[8:7];     // RegWrite, MemtoReg
            ID_EX_MCTRL <= control_signals[6:4];      // Branch, MemRead, MemWrite
            ID_EX_EXCTRL <= control_signals[3:0];     // RegDst, ALUOp, ALUSrc
            ID_EX_FUNCT <= IF_ID_IR[5:0];      // ? grab the real funct bit
        end
    end
endmodule

// EX Stage: Execution
module I_EXECUTE (
    input wire clk,
    input wire reset,
    input wire [31:0] ID_EX_NPC,
    input wire [31:0] ID_EX_A,
    input wire [31:0] ID_EX_B,
    input wire [31:0] ID_EX_IMM,
    input wire [4:0] ID_EX_RT,
    input wire [4:0] ID_EX_RD,
    input wire [1:0] ID_EX_WBCTRL,
    input wire [2:0] ID_EX_MCTRL,
    input wire [3:0] ID_EX_EXCTRL,
    output reg [31:0] EX_MEM_NPC,
    output reg [31:0] EX_MEM_ALU_RESULT,
    output reg [31:0] EX_MEM_B,
    output reg [4:0] EX_MEM_WRITEREG,
    output reg [1:0] EX_MEM_WBCTRL,
    output reg [2:0] EX_MEM_MCTRL,
    output reg EX_MEM_ZERO,
    input wire [5:0]  ID_EX_FUNCT
);
    wire [31:0] alu_input_b, alu_result, branch_target;
    wire [4:0] write_reg;
    wire [2:0] alu_control;
    wire alu_zero;
    wire [31:0] imm_shifted = ID_EX_IMM << 2;
    // ALU Control Unit
    ALU_CONTROL alu_ctrl (
        .alu_op(ID_EX_EXCTRL[2:1]),
        .funct(ID_EX_FUNCT),
        .select(alu_control)
    );
    
    // 5-bit Mux for selecting Write Register
    MUX5 write_reg_mux (
        .a(ID_EX_RD),
        .b(ID_EX_RT),
        .sel(ID_EX_EXCTRL[3]),  // RegDst
        .y(write_reg)
    );
    
    // MUX for ALU second operand
    MUX alu_src_mux (
        .a(imm_shifted),
        .b(ID_EX_B),
        .sel(ID_EX_EXCTRL[0]),  // ALUSrc
        .y(alu_input_b)
    );
    
    // ALU
    ALU alu (
        .A(ID_EX_A),
        .B(alu_input_b),
        .control(alu_control),
        .result(alu_result),
        .zero(alu_zero)
    );
    
    // Adder for branch target address
    ADDER branch_adder (
        .add_in1(ID_EX_NPC),
        .add_in2(ID_EX_IMM),  // We'll assume shift left 2 is done in hardware
        .add_out(branch_target)
    );
    
    // Initialize EX/MEM pipeline register
    initial begin
        EX_MEM_NPC = 32'b0;
        EX_MEM_ALU_RESULT = 32'b0;
        EX_MEM_B = 32'b0;
        EX_MEM_WRITEREG = 5'b0;
        EX_MEM_WBCTRL = 2'b0;
        EX_MEM_MCTRL = 3'b0;
        EX_MEM_ZERO = 1'b0;
    end
    
    // EX/MEM Pipeline Register
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            EX_MEM_NPC <= 32'b0;
            EX_MEM_ALU_RESULT <= 32'b0;
            EX_MEM_B <= 32'b0;
            EX_MEM_WRITEREG <= 5'b0;
            EX_MEM_WBCTRL <= 2'b0;
            EX_MEM_MCTRL <= 3'b0;
            EX_MEM_ZERO <= 1'b0;
        end
        else begin
            EX_MEM_NPC <= branch_target;
            EX_MEM_ALU_RESULT <= alu_result;
            EX_MEM_B <= ID_EX_B;
            EX_MEM_WRITEREG <= write_reg;
            EX_MEM_WBCTRL <= ID_EX_WBCTRL;
            EX_MEM_MCTRL <= ID_EX_MCTRL;
            EX_MEM_ZERO <= alu_zero;
        end
    end
endmodule

// MEM Stage: Memory
module MEMORY (
    input wire clk,
    input wire reset,
    input wire [31:0] EX_MEM_NPC,
    input wire [31:0] EX_MEM_ALU_RESULT,
    input wire [31:0] EX_MEM_B,
    input wire [4:0] EX_MEM_WRITEREG,
    input wire [1:0] EX_MEM_WBCTRL,
    input wire [2:0] EX_MEM_MCTRL,
    input wire EX_MEM_ZERO,
    output reg [31:0] MEM_WB_READ_DATA,
    output reg [31:0] MEM_WB_ALU_RESULT,
    output reg [4:0] MEM_WB_WRITEREG,
    output reg [1:0] MEM_WB_WBCTRL,
    output wire PCSrc
);
    wire mem_read, mem_write, branch;
    wire [31:0] read_data;
    
    // Extract control signals
    assign branch = EX_MEM_MCTRL[2];
    assign mem_read = EX_MEM_MCTRL[1];
    assign mem_write = EX_MEM_MCTRL[0];
    
    // AND gate for branch decision
    AND branch_and (
        .zero(EX_MEM_ZERO),
        .m_ctlout(branch),
        .PCSrc(PCSrc)
    );
    
    // Data Memory
    D_MEM data_memory (
        .clk(clk),
        .MemRead(mem_read),
        .MemWrite(mem_write),
        .Address(EX_MEM_ALU_RESULT),
        .Write_data(EX_MEM_B),
        .Read_data(read_data)
    );
    
    // MEM/WB Pipeline Register
    initial begin
        MEM_WB_READ_DATA = 32'b0;
        MEM_WB_ALU_RESULT = 32'b0;
        MEM_WB_WRITEREG = 5'b0;
        MEM_WB_WBCTRL = 2'b0;
    end
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            MEM_WB_READ_DATA <= 32'b0;
            MEM_WB_ALU_RESULT <= 32'b0;
            MEM_WB_WRITEREG <= 5'b0;
            MEM_WB_WBCTRL <= 2'b0;
        end
        else begin
            MEM_WB_READ_DATA <= read_data;
            MEM_WB_ALU_RESULT <= EX_MEM_ALU_RESULT;
            MEM_WB_WRITEREG <= EX_MEM_WRITEREG;
            MEM_WB_WBCTRL <= EX_MEM_WBCTRL;
        end
    end
endmodule

// WB Stage: Write Back
module WB (
    input wire [31:0] MEM_WB_READ_DATA,
    input wire [31:0] MEM_WB_ALU_RESULT,
    input wire [1:0] MEM_WB_WBCTRL,
    output wire [31:0] WB_WRITEDATA,
    output wire WB_REGWRITE
);
    wire mem_to_reg;
    // Extract control signals
    assign WB_REGWRITE = MEM_WB_WBCTRL[1];
    assign mem_to_reg = MEM_WB_WBCTRL[0];
    
    // Debug the control signals
    always @(MEM_WB_WBCTRL) begin
        $display("WB Stage: RegWrite=%b, MemtoReg=%b", 
                 MEM_WB_WBCTRL[1], MEM_WB_WBCTRL[0]);
    end
    
    // MUX for selecting write data
    MUX wb_mux (
        .a(MEM_WB_READ_DATA),  // Data from memory when MemtoReg=1
        .b(MEM_WB_ALU_RESULT), // ALU result when MemtoReg=0
        .sel(mem_to_reg),      // Control signal (MemtoReg)
        .y(WB_WRITEDATA)       // Output to register file
    );
    
    // Debug the output
    always @(WB_WRITEDATA) begin
        if (WB_REGWRITE) 
            $display("WB Stage: Writing data %0d (source: %s)",
                     WB_WRITEDATA, mem_to_reg ? "Memory" : "ALU");
    end
endmodule
// Basic Components

// 32-bit Multiplexer
module MUX (
    input wire [31:0] a,
    input wire [31:0] b,
    input wire sel,
    output wire [31:0] y
);
    assign y = sel ? a : b;
endmodule

// 5-bit Multiplexer
module MUX5 (
    input wire [4:0] a,
    input wire [4:0] b,
    input wire sel,
    output wire [4:0] y
);
    assign y = sel ? a : b;
endmodule

// Incrementer by 4
module INCR (
    input wire [31:0] pcin,
    output wire [31:0] pcout
);
    assign pcout = pcin + 32'd4;
endmodule

// 32-bit Adder
module ADDER (
    input wire [31:0] add_in1,
    input wire [31:0] add_in2,
    output wire [31:0] add_out
);
    assign add_out = add_in1 + add_in2;
endmodule

// AND Gate
module AND (
    input wire zero,
    input wire m_ctlout,
    output wire PCSrc
);
    assign PCSrc = zero & m_ctlout;
endmodule

// Control Unit
module CONTROL (
    input wire [5:0] opcode,
    output reg [8:0] controlbits
);
    // Control signals: RegWrite(1), MemtoReg(1), Branch(1), MemRead(1), MemWrite(1), RegDst(1), ALUOp(2), ALUSrc(1)
    // Control bits: [8:0] = {RegWrite, MemtoReg, Branch, MemRead, MemWrite, RegDst, ALUOp[1:0], ALUSrc}
    
    parameter R_TYPE = 6'b000000;
    parameter LW = 6'b100011;
    parameter SW = 6'b101011;
    parameter BEQ = 6'b000100;
    parameter NOP = 6'b100000;
    
    always @(*) begin
        case (opcode)
            R_TYPE: begin
                controlbits = 9'b100001100;  // RegWrite=1, MemtoReg=0, Branch=0, MemRead=0, MemWrite=0, RegDst=1, ALUOp=10, ALUSrc=0
            end
            
            LW: begin
                controlbits = 9'b110100001;  // RegWrite=1, MemtoReg=1, Branch=0, MemRead=1, MemWrite=0, RegDst=0, ALUOp=00, ALUSrc=1
            end
            
            SW: begin
                controlbits = 9'b001010001;  // RegWrite=0, MemtoReg=0, Branch=0, MemRead=0, MemWrite=1, RegDst=0, ALUOp=00, ALUSrc=1
            end
            
            BEQ: begin
                controlbits = 9'b001000010;  // RegWrite=0, MemtoReg=0, Branch=1, MemRead=0, MemWrite=0, RegDst=0, ALUOp=01, ALUSrc=0
            end
            
            NOP: begin
                controlbits = 9'b000000000;  // All zeros for NOP
            end
            
            default: begin
                controlbits = 9'b000000000;
            end
        endcase
    end
endmodule




// Register File
module REG (
    input wire clk,
    input wire reset,
    input wire [4:0] rs,
    input wire [4:0] rt,
    input wire [4:0] rd,
    input wire [31:0] writedata,
    input wire regwrite,
    output wire [31:0] A,
    output wire [31:0] B
);
    reg [31:0] registers [0:31];
    integer i;
    
    // Initialize all registers to 0
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'b0;
        $display("Register file initialized with all zeros");
    end
    
    // Read data - read directly from register array
    assign A = (rs == 0) ? 32'b0 : registers[rs];
    assign B = (rt == 0) ? 32'b0 : registers[rt];
    
    // Write data with debug output
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 32'b0;
            $display("Register file reset to all zeros");
        end
        else if (regwrite && rd != 0) begin  // Ensure r0 remains 0
            registers[rd] <= writedata;
            $display("Writing value %0d to register r%0d", writedata, rd);
        end
    end
endmodule

// Sign Extend
module S_EXTEND (
    input wire [15:0] imm16,
    output wire [31:0] imm32
);
    assign imm32 = {{16{imm16[15]}}, imm16};
endmodule

// ALU Control
module ALU_CONTROL (
    input wire [1:0] alu_op,
    input wire [5:0] funct,
    output reg [2:0] select
);
    // ALU Control codes
    parameter ADD = 3'b010;
    parameter SUB = 3'b110;
    parameter AND = 3'b000;
    parameter OR = 3'b001;
    parameter SLT = 3'b111;
    
    // Function codes
    parameter FUNCT_ADD = 6'b100000;
    parameter FUNCT_SUB = 6'b100010;
    parameter FUNCT_AND = 6'b100100;
    parameter FUNCT_OR = 6'b100101;
    parameter FUNCT_SLT = 6'b101010;
    
    always @(*) begin
        case (alu_op)
            2'b00: select = ADD; // LW/SW - always ADD
            2'b01: select = SUB; // BEQ - always SUB for comparison
            2'b10: begin // R-type - based on funct field
                case (funct)
                    FUNCT_ADD: select = ADD;
                    FUNCT_SUB: select = SUB;
                    FUNCT_AND: select = AND;
                    FUNCT_OR: select = OR;
                    FUNCT_SLT: select = SLT;
                    default: select = 3'bxxx; // Undefined
                endcase
            end
            default: select = 3'bxxx; // Undefined ALUOp
        endcase
    end
endmodule

// ALU
module ALU (
    input wire [31:0] A,
    input wire [31:0] B,
    input wire [2:0] control,
    output reg [31:0] result,
    output wire zero
);
    // ALU operations
    parameter ADD = 3'b010;
    parameter SUB = 3'b110;
    parameter AND = 3'b000;
    parameter OR = 3'b001;
    parameter SLT = 3'b111;
    
    always @(*) begin
        case (control)
            ADD: result = A + B;
            SUB: result = A - B;
            AND: result = A & B;
            OR: result = A | B;
            SLT: result = (A < B) ? 32'd1 : 32'd0;
            default: result = 32'bx;
        endcase
    end
    
    assign zero = (result == 32'b0);
endmodule

// Instruction Memory
// Instruction Memory (file-loaded only)
module INST_MEM (
    input  wire [31:0] addr, 
    output reg  [31:0] data
);
    reg [31:0] MEM [0:127];  // 128 words  
    integer    i;

    initial begin
        // 1) Load everything from risc.txt (must be 24+ lines of 32-bit binaries)
        $readmemb("risc.txt", MEM);
        $display(">>> INST_MEM after $readmemb:");
        for (i = 0; i < 24; i = i + 1)
            $display("  MEM[%0d] = %b", i, MEM[i]);
    end

    // Read (word-aligned)
    always @(*) begin
        data = MEM[ addr >> 2 ];
        $display("INST_MEM read: addr=%h -> word[%0d] = %b", addr, addr>>2, data);
    end
endmodule


// Data Memory
module D_MEM (
    input wire clk,
    input wire MemRead,
    input wire MemWrite,
    input wire [31:0] Address,
    input wire [31:0] Write_data,
    output reg [31:0] Read_data
);
    reg [31:0] MEM [0:255];  // 256 words of 32-bit memory
    integer i;
    
    // Initialize memory with known values directly
    initial begin
        // Clear all memory
        for (i = 0; i < 256; i = i + 1)
            MEM[i] = 32'b0;
            
        // Set known data values (same as data.txt content)
        MEM[0] = 32'h00000000; // Data 0
        MEM[1] = 32'h00000001; // Data 1
        MEM[2] = 32'h00000002; // Data 2
        MEM[3] = 32'h00000003; // Data 3
        MEM[4] = 32'h00000004; // Data 4
        MEM[5] = 32'h00000005; // Data 5
        
        $display("Data Memory Initialized:");
        for (i = 0; i < 6; i = i + 1)
            $display("MEM[%0d] = %h (%0d)", i, MEM[i], MEM[i]);
        
        // Try to load from file as a backup
        $readmemb("data.txt", MEM);
        
        // Display memory contents after potential file load
        $display("Data Memory Contents (after potentially loading from file):");
        for (i = 0; i < 6; i = i + 1)
            $display("MEM[%0d] = %h (%0d)", i, MEM[i], MEM[i]);
    end
    
    // Read operation with explicit debug output
    always @(*) begin
        if (MemRead) begin
            // Calculate word address from byte address (right shift by 2)
            Read_data = MEM[Address >> 2];
            $display("D_MEM: READ Address=%h (word_addr=%0d) => Data=%h (%0d)", 
                     Address, Address >> 2, Read_data, Read_data);
        end
        else begin
            Read_data = 32'b0;
        end
    end
    
    // Write operation with debug output
    always @(posedge clk) begin
        if (MemWrite) begin
            MEM[Address >> 2] <= Write_data;
            $display("D_MEM: WRITE Address=%h (word_addr=%0d) <= Data=%h (%0d)", 
                     Address, Address >> 2, Write_data, Write_data);
        end
    end
endmodule
