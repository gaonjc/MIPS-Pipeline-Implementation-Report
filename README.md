# MIPS-Pipeline-Implementation-Report
Computer Architecture MIPS Pipeline
# MIPS Pipeline Implementation Report

This report provides a comprehensive overview of a 5-stage MIPS pipeline implementation in Verilog. Each stage (Fetch, Decode, Execute, Memory, and Writeback) is thoroughly analyzed with focus on its purpose, components, connections, and behavior. The report also examines how register values, particularly r1, change throughout program execution.

## Table of Contents
1. [Instruction Fetch Stage](#1-instruction-fetch-stage)
2. [Instruction Decode Stage](#2-instruction-decode-stage)
3. [Execution Stage](#3-execution-stage)
4. [Memory Stage](#4-memory-stage)
5. [Writeback Stage](#5-writeback-stage)
6. [Conclusion](#6-conclusion)

## 1. Instruction Fetch Stage

### Overview
The Instruction Fetch (IF) stage is responsible for retrieving the next instruction from memory based on the Program Counter (PC) value and incrementing the PC for the next cycle.

#### Purpose
- Retrieve the next instruction from instruction memory
- Calculate the next PC value
- Pass instruction and next PC to the ID stage

#### Components
- **Program Counter (PC)** register: Holds the address of the current instruction
- **Instruction Memory (INST_MEM)**: Stores program instructions
- **Incrementer (INCR)**: Adds 4 to PC to point to the next instruction
- **Multiplexer (MUX)**: Selects between PC+4 or branch target address
- **IF/ID Pipeline Register**: Stores the fetched instruction and PC+4 for the next stage

#### Connections
- PC is connected to the instruction memory to fetch instructions
- PC is connected to the incrementer to calculate PC+4
- The incrementer output and branch target address from MEM stage are inputs to the multiplexer
- Multiplexer output becomes the next PC value
- Instruction and PC+4 are stored in IF/ID pipeline register

#### Expected Inputs and Outputs
- **Inputs**: 
  - Current PC value
  - PCSrc control signal from MEM stage
  - Branch target address from MEM stage (EX_MEM_NPC)
- **Outputs**:
  - Fetched instruction (IF_ID_IR)
  - Incremented PC (IF_ID_NPC) for next instruction

### Code Components

#### INCR.v (PC Incrementer)
```verilog
module INCR (
    input wire [31:0] pcin,
    output wire [31:0] pcout
);
    assign pcout = pcin + 32'd4;
endmodule
```

#### MUX.v (32-bit Multiplexer)
```verilog
module MUX (
    input wire [31:0] a,
    input wire [31:0] b,
    input wire sel,
    output wire [31:0] y
);
    assign y = sel ? a : b;
endmodule
```

#### INST_MEM.v (Instruction Memory)
```verilog
module INST_MEM (
    input  wire [31:0] addr, 
    output reg  [31:0] data
);
    reg [31:0] MEM [0:127];  // 128 words  
    integer    i;

    initial begin
        // Load instructions from risc.txt
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
```

#### I_FETCH.v (Fetch Stage)
```verilog
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
```

### Timing Diagram

![IF Stage Timing Diagram](https://github.com/gaonjc/MIPS-Pipeline-Implementation-Report/blob/main/testbench_waveforms/IF_STAGE_WAVEFORM.gif)

The timing diagram shows the instruction fetch process over multiple clock cycles. At each positive clock edge, the PC is updated and a new instruction is fetched.

### Analysis of Instruction Memory Contents

Based on the code output, we can see the first few instructions loaded in memory:

```
MEM[0] = 10001100000000010000000000000001  // lw r1, 1(r0)
MEM[1] = 10001100000000100000000000000010  // lw r2, 2(r0)
MEM[2] = 10001100000000110000000000000011  // lw r3, 3(r0)
MEM[3] = 10000000000000000000000000000000  // nop
MEM[4] = 10000000000000000000000000000000  // nop
MEM[5] = 00000000001000100000100000100000  // add r1, r1, r2
```

These instructions demonstrate a simple program that:
1. Loads values from memory into registers r1, r2, and r3
2. Adds r1 and r2, storing the result in r1

## 2. Instruction Decode Stage

### Overview
The Instruction Decode (ID) stage extracts information from the instruction and prepares operands for the execution stage. It decodes instructions, reads registers, and generates control signals.

#### Purpose
- Decode the instruction to determine operation type
- Read operands from register file
- Generate control signals for subsequent stages
- Sign-extend immediate values for use in later stages

#### Components
- **Control Unit (CONTROL)**: Generates control signals based on opcode
- **Register File (REG)**: Stores and provides register values
- **Sign Extend Unit (S_EXTEND)**: Extends 16-bit immediate to 32-bit
- **ID/EX Pipeline Register**: Passes decoded information to the next stage

#### Connections
- Instruction from IF/ID register is decoded to extract opcode, register numbers, and immediate value
- Opcode feeds into the control unit to generate control signals
- Register numbers access the register file to retrieve operand values
- Immediate value is sign-extended for ALU operations
- All values and control signals are stored in ID/EX pipeline register

#### Expected Inputs and Outputs
- **Inputs**:
  - Instruction (IF_ID_IR)
  - Next PC value (IF_ID_NPC)
  - Write register address (MEM_WB_WRITEREG)
  - Write data (MEM_WB_WRITEDATA)
  - Register write enable (MEM_WB_REGWRITE)
- **Outputs**:
  - Read data 1 (ID_EX_A)
  - Read data 2 (ID_EX_B)
  - Sign-extended immediate (ID_EX_IMM)
  - Register numbers (ID_EX_RT, ID_EX_RD)
  - Control signals (ID_EX_WBCTRL, ID_EX_MCTRL, ID_EX_EXCTRL)
  - Function code (ID_EX_FUNCT)

### Code Components

#### CONTROL.v (Control Unit)
```verilog
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
```

#### REG.v (Register File)
```verilog
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
```

#### S_EXTEND.v (Sign Extend)
```verilog
module S_EXTEND (
    input wire [15:0] imm16,
    output wire [31:0] imm32
);
    assign imm32 = {{16{imm16[15]}}, imm16};
endmodule
```

#### I_DECODE.v (Decode Stage)
```verilog
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
            ID_EX_FUNCT <= IF_ID_IR[5:0];
        end
    end
endmodule
```

### Timing Diagram

![ID Stage Timing Diagram](https://github.com/gaonjc/MIPS-Pipeline-Implementation-Report/blob/main/testbench_waveforms/ID_STAGE_WAVEFORM.gif)

### Analysis of Register File Behavior

The register file is initialized with all zeros at the start. During execution, the testbench output shows how r1 changes throughout the program:

1. At Clock Cycle 5: r1 is written with value 1 from memory (first lw instruction)
2. At Clock Cycle 10: r1 is written with value 3 (result of add r1,r2)
3. At Clock Cycle 14: r1 is written with value 6 (another arithmetic operation)
4. At Clock Cycle 18: r1 is written with value 12 (another arithmetic operation)

These register updates align with the program logic where r1 is modified by various instructions.

## 3. Execution Stage

### Overview
The Execution (EX) stage performs arithmetic and logical operations using the ALU, calculates memory addresses for load/store operations, and computes branch target addresses.

#### Purpose
- Perform arithmetic and logical operations
- Calculate memory addresses for load/store instructions
- Compute branch target addresses
- Determine write register address for register-type instructions

#### Components
- **ALU Control Unit (ALU_CONTROL)**: Generates ALU operation signals
- **ALU**: Performs arithmetic and logical operations
- **Branch Target Adder (ADDER)**: Calculates branch target address
- **Multiplexers**: Select appropriate inputs and outputs
- **EX/MEM Pipeline Register**: Passes execution results to the next stage

#### Connections
- ALU Control receives operation type from control signals and function code
- ALU receives operands from register file or immediate value
- Branch target adder combines PC+4 and immediate value
- Multiplexers select appropriate data paths based on instruction type
- Results are stored in EX/MEM pipeline register

#### Expected Inputs and Outputs
- **Inputs**:
  - Register values (ID_EX_A, ID_EX_B)
  - Sign-extended immediate (ID_EX_IMM)
  - Register numbers (ID_EX_RT, ID_EX_RD)
  - Control signals (ID_EX_WBCTRL, ID_EX_MCTRL, ID_EX_EXCTRL)
  - Function code (ID_EX_FUNCT)
- **Outputs**:
  - ALU result (EX_MEM_ALU_RESULT)
  - Branch target address (EX_MEM_NPC)
  - Register value for memory operations (EX_MEM_B)
  - Write register address (EX_MEM_WRITEREG)
  - Zero flag for branch decisions (EX_MEM_ZERO)
  - Control signals for later stages (EX_MEM_WBCTRL, EX_MEM_MCTRL)

### Code Components

#### ALU_CONTROL.v (ALU Control Unit)
```verilog
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
```

#### ALU.v (Arithmetic Logic Unit)
```verilog
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
```

#### ADDER.v (32-bit Adder)
```verilog
module ADDER (
    input wire [31:0] add_in1,
    input wire [31:0] add_in2,
    output wire [31:0] add_out
);
    assign add_out = add_in1 + add_in2;
endmodule
```

#### MUX5.v (5-bit Multiplexer)
```verilog
module MUX5 (
    input wire [4:0] a,
    input wire [4:0] b,
    input wire sel,
    output wire [4:0] y
);
    assign y = sel ? a : b;
endmodule
```

#### I_EXECUTE.v (Execution Stage)
```verilog
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
```

### Timing Diagram

![Placeholder for EX Stage Timing Diagram]

### Analysis of ALU Operations for Register r1

The execution stage performs these key operations for register r1 modifications:

1. **Clock Cycle 3-4**: ALU calculates the memory address (0+4=4) for the first lw instruction, which loads data from memory location 1
2. **Clock Cycle 8-9**: ALU adds r1 (value: 1) and r2 (value: 2) to produce 3
3. **Clock Cycle 12-13**: ALU adds r1 (value: 3) and r3 (value: 3) to produce 6
4. **Clock Cycle 16-17**: ALU adds r1 (value: 6) and r1 (value: 6) to produce 12

These ALU results align with the expected arithmetic operations and the subsequent updates to r1 observed in the register file.

## 4. Memory Stage

### Overview
The Memory (MEM) stage performs memory operations for load and store instructions and determines whether to take a branch based on the ALU zero flag.

#### Purpose
- Access data memory for load/store operations
- Determine branch decision based on ALU results
- Pass ALU results and memory data to the writeback stage

#### Components
- **Data Memory (D_MEM)**: Stores and retrieves data
- **AND Gate**: Combines branch control and zero flag for branch decision
- **MEM/WB Pipeline Register**: Passes memory and ALU results to the next stage

#### Connections
- ALU result provides the memory address for load/store operations
- Register value from ID/EX stage provides data for store operations
- Branch control signal and zero flag determine branch decision
- Memory read data and ALU result are stored in MEM/WB pipeline register

#### Expected Inputs and Outputs
- **Inputs**:
  - ALU result (EX_MEM_ALU_RESULT)
  - Branch target address (EX_MEM_NPC)
  - Register value for store operations (EX_MEM_B)
  - Zero flag (EX_MEM_ZERO)
  - Control signals (EX_MEM_WBCTRL, EX_MEM_MCTRL)
- **Outputs**:
  - Memory read data (MEM_WB_READ_DATA)
  - ALU result (MEM_WB_ALU_RESULT)
  - Write register address (MEM_WB_WRITEREG)
  - Control signals for writeback stage (MEM_WB_WBCTRL)
  - Branch decision signal (PCSrc)

### Code Components

#### D_MEM.v (Data Memory)
```verilog
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
```

#### AND.v (AND Gate for Branch Control)
```verilog
module AND (
    input wire zero,
    input wire m_ctlout,
    output wire PCSrc
);
    assign PCSrc = zero & m_ctlout;
endmodule
```

#### MEMORY.v (Memory Stage)
```verilog
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
```

### Timing Diagram

![Placeholder for MEM Stage Timing Diagram]

### Analysis of Memory Operations for Register r1

In this simulation, the memory stage plays a crucial role in loading values into r1:

1. **Clock Cycle 4**: Memory stage reads data from memory address 4 (word address 1) and retrieves the value 1. This is from the first lw instruction that loads memory[1] into r1.
2. **Clock Cycle 9-10**: Memory stage passes along the ALU result (3) from adding r1 and r2.
3. **Clock Cycle 13-14**: Memory stage passes along the ALU result (6) from adding r1 and r3.
4. **Clock Cycle 17-18**: Memory stage passes along the ALU result (12) from adding r1 and r1.

The memory access at address 4 (word 1) returns the value 1, which is the first value loaded into r1. For the other operations, the memory stage simply passes the ALU results to the writeback stage.

## 5. Writeback Stage

### Overview
The Writeback (WB) stage is the final stage of the pipeline, responsible for writing results back to the register file.

#### Purpose
- Select appropriate data to write back to the register file
- Generate register write control signal
- Complete the instruction execution cycle

#### Components
- **Multiplexer**: Selects between ALU result and memory data
- **Write Back Control**: Generates register write control signal

#### Connections
- Multiplexer selects between ALU result and memory data based on MemtoReg control signal
- Register write control signal enables writing to the register file
- Selected data is written to the specified register in the register file

#### Expected Inputs and Outputs
- **Inputs**:
  - Memory read data (MEM_WB_READ_DATA)
  - ALU result (MEM_WB_ALU_RESULT)
  - Control signals (MEM_WB_WBCTRL)
- **Outputs**:
  - Write data for register file (WB_WRITEDATA)
  - Register write enable signal (WB_REGWRITE)

### Code Components

#### WB.v (Writeback Stage)
```verilog
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
```

### Timing Diagram

![Placeholder for WB Stage Timing Diagram]

### Analysis of Register r1 Updates in Writeback Stage

The writeback stage is where r1 gets its final values. Following the program execution through the simulation output:

1. **Clock Cycle 5**: WB stage receives value 1 from memory and writes it to r1 (MemtoReg=1, RegWrite=1)
2. **Clock Cycle 10**: WB stage receives value 3 from ALU and writes it to r1 (MemtoReg=0, RegWrite=1)
3. **Clock Cycle 14**: WB stage receives value 6 from ALU and writes it to r1 (MemtoReg=0, RegWrite=1)
4. **Clock Cycle 18**: WB stage receives value 12 from ALU and writes it to r1 (MemtoReg=0, RegWrite=1)

This sequence shows how r1 evolves from its initial value of 0, to 1 (loaded from memory), then to 3, 6, and finally 12 through successive arithmetic operations.

## 6. Conclusion

The implementation of the MIPS pipeline in this lab provides a functional but basic 5-stage pipeline design. However, there are several key areas where the performance and functionality could be improved:

### 1. Protection Against Hazards

The current implementation does not address potential hazards, which are critical issues in pipelined processors:

#### Data Hazards
Data hazards occur when an instruction depends on the result of a previous instruction that is still in the pipeline.

**Possible Improvements:**
- **Forwarding Unit**: Implement a forwarding (bypassing) unit that detects when an instruction needs a result that has been computed but not yet written back, and routes that result directly to where it's needed.
  - Add data paths from the ALU output and memory stage back to the ALU inputs
  - Add logic to detect data dependencies between instructions
  - Modify the EX stage to select forwarded values when appropriate

```verilog
// Example forwarding unit
module FORWARDING_UNIT (
    input wire [4:0] ID_EX_RS, ID_EX_RT,
    input wire [4:0] EX_MEM_RD, MEM_WB_RD,
    input wire EX_MEM_REGWRITE, MEM_WB_REGWRITE,
    output reg [1:0] FORWARD_A, FORWARD_B
);
    // Forward from MEM stage
    always @(*) begin
        // Forward A logic (for RS)
        if (EX_MEM_REGWRITE && EX_MEM_RD != 0 && EX_MEM_RD == ID_EX_RS)
            FORWARD_A = 2'b10;
        else if (MEM_WB_REGWRITE && MEM_WB_RD != 0 && MEM_WB_RD == ID_EX_RS)
            FORWARD_A = 2'b01;
        else
            FORWARD_A = 2'b00;
            
        // Forward B logic (for RT)
        if (EX_MEM_REGWRITE && EX_MEM_RD != 0 && EX_MEM_RD == ID_EX_RT)
            FORWARD_B = 2'b10;
        else if (MEM_WB_REGWRITE && MEM_WB_RD != 0 && MEM_WB_RD == ID_EX_RT)
            FORWARD_B = 2'b01;
        else
            FORWARD_B = 2'b00;
    end
endmodule
```

- **Stall Logic/Hazard Detection Unit**: When forwarding isn't possible (e.g., load-use hazards), implement stall logic to pause the pipeline until the data is available.
  - Detect load-use hazards specifically (when a load is followed by an instruction that uses the loaded value)
  - Insert bubbles/stalls into the pipeline

```verilog
// Example hazard detection unit
module HAZARD_DETECTION (
    input wire [4:0] ID_EX_RT,
    input wire [4:0] IF_ID_RS, IF_ID_RT,
    input wire ID_EX_MEMREAD,
    output reg STALL
);
    always @(*) begin
        if (ID_EX_MEMREAD && 
            (ID_EX_RT == IF_ID_RS || ID_EX_RT == IF_ID_RT))
            STALL = 1'b1;
        else
            STALL = 1'b0;
    end
endmodule
```

#### Control Hazards
Control hazards occur with branch instructions when the pipeline fetches instructions before knowing whether a branch is taken.

**Possible Improvements:**
- **Branch Prediction**: Implement static or dynamic branch prediction to reduce pipeline stalls.
  - Static prediction: Always predict not taken (simplest)
  - 1-bit predictor: Remember last branch decision
  - 2-bit predictor: More sophisticated state machine

```verilog
// Example 2-bit branch predictor
module BRANCH_PREDICTOR (
    input wire clk, reset,
    input wire [31:0] PC,
    input wire branch_result,  // Actual branch outcome
    input wire update,         // Signal to update predictor
    output wire prediction     // Predicted outcome
);
    // Simplified direct-mapped branch prediction buffer
    reg [1:0] prediction_bits [0:31];  // 2 bits per entry, 32 entries
    wire [4:0] index = PC[6:2];        // Use bits from PC as index
    
    // Prediction logic
    assign prediction = prediction_bits[index][1];  // MSB determines prediction
    
    // Update logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize to "weakly not taken"
            for (integer i = 0; i < 32; i = i + 1)
                prediction_bits[i] <= 2'b01;
        end
        else if (update) begin
            case (prediction_bits[index])
                2'b00: prediction_bits[index] <= branch_result ? 2'b01 : 2'b00;
                2'b01: prediction_bits[index] <= branch_result ? 2'b10 : 2'b00;
                2'b10: prediction_bits[index] <= branch_result ? 2'b11 : 2'b01;
                2'b11: prediction_bits[index] <= branch_result ? 2'b11 : 2'b10;
            endcase
        end
    end
endmodule
```

- **Early Branch Resolution**: Move branch decision logic earlier in the pipeline.
  - Compare registers in ID stage instead of waiting for EX stage
  - Add dedicated branch adder in ID stage

#### Structural Hazards
Structural hazards occur when multiple instructions try to use the same hardware resource simultaneously.

**Possible Improvements:**
- **Separate Instruction and Data Memory**: This would allow simultaneous access to both memories.
- **Multi-port Register File**: Allows multiple reads and writes in the same cycle.

### 2. Instruction Reordering

Instruction reordering can help mitigate pipeline stalls by arranging instructions in a way that minimizes hazards.

**Possible Improvements:**
- **Instruction Scheduling**: Implement a more sophisticated scheduler that can reorder instructions to minimize data dependencies and pipeline stalls.
  - Static scheduling at compile time
  - Dynamic scheduling with scoreboarding or Tomasulo's algorithm

```verilog
// Example of a simple reservation station for dynamic scheduling
module RESERVATION_STATION (
    input wire clk, reset,
    input wire [5:0] opcode,
    input wire [4:0] src1, src2, dest,
    input wire [31:0] src1_value, src2_value,
    input wire src1_valid, src2_valid,
    input wire issue,
    output reg [5:0] exec_opcode,
    output reg [31:0] exec_src1, exec_src2,
    output reg [4:0] exec_dest,
    output reg exec_valid
);
    // Reservation station entry
    reg busy;
    reg [5:0] rs_opcode;
    reg [4:0] rs_src1, rs_src2, rs_dest;
    reg [31:0] rs_src1_value, rs_src2_value;
    reg rs_src1_valid, rs_src2_valid;
    
    // Issue logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            busy <= 1'b0;
            exec_valid <= 1'b0;
        end
        else if (issue && !busy) begin
            busy <= 1'b1;
            rs_opcode <= opcode;
            rs_src1 <= src1;
            rs_src2 <= src2;
            rs_dest <= dest;
            rs_src1_value <= src1_value;
            rs_src2_value <= src2_value;
            rs_src1_valid <= src1_valid;
            rs_src2_valid <= src2_valid;
        end
        
        // Execute when all operands are ready
        if (busy && rs_src1_valid && rs_src2_valid) begin
            exec_opcode <= rs_opcode;
            exec_src1 <= rs_src1_value;
            exec_src2 <= rs_src2_value;
            exec_dest <= rs_dest;
            exec_valid <= 1'b1;
            busy <= 1'b0;
        end
        else begin
            exec_valid <= 1'b0;
        end
    end
endmodule
```

- **Out-of-Order Execution**: Allow instructions to execute as soon as their operands are available, rather than in program order.
  - Requires a robust mechanism to maintain program semantics
  - Add reorder buffer to ensure in-order completion

- **Register Renaming**: Eliminate false dependencies (WAR, WAW hazards) by renaming registers.
  - Map architectural registers to larger set of physical registers
  - Allow multiple versions of a register to exist simultaneously

### 3. Write-Through and Write-Back Improvements

The current memory system can be improved for better performance:

**Possible Improvements:**
- **Cache Memory**: Implement a cache memory system to reduce memory access latency.
  - L1 instruction cache
  - L1 data cache
  - Optional L2 unified cache

```verilog
// Example simple direct-mapped cache
module CACHE (
    input wire clk, reset,
    input wire [31:0] address,
    input wire [31:0] write_data,
    input wire read_req, write_req,
    output reg [31:0] read_data,
    output reg hit
);
    // Cache parameters
    parameter CACHE_SIZE = 1024;  // 1KB cache
    parameter LINE_SIZE = 4;      // 4 bytes per line (one word)
    parameter NUM_LINES = CACHE_SIZE / LINE_SIZE;
    
    // Cache storage
    reg [31:0] data [0:NUM_LINES-1];
    reg [23:0] tags [0:NUM_LINES-1];  // Tag bits
    reg valid [0:NUM_LINES-1];        // Valid bits
    
    // Cache indexing
    wire [7:0] index = address[9:2];  // 8-bit index for 256 lines
    wire [23:0] tag = address[31:10]; // 22-bit tag
    
    // Cache access
    always @(posedge clk) begin
        if (reset) begin
            for (integer i = 0; i < NUM_LINES; i = i + 1) begin
                valid[i] <= 1'b0;
            end
            hit <= 1'b0;
        end
        else if (read_req) begin
            if (valid[index] && tags[index] == tag) begin
                read_data <= data[index];
                hit <= 1'b1;
            end
            else begin
                hit <= 1'b0;
                // Memory access logic would go here
            end
        end
        else if (write_req) begin
            if (valid[index] && tags[index] == tag) begin
                data[index] <= write_data;
                hit <= 1'b1;
            end
            else begin
                hit <= 1'b0;
                // Memory access logic would go here
            end
        end
    end
endmodule
```

- **Write Buffer**: Add a write buffer to allow the processor to continue execution while memory writes are being performed.
  - Buffer multiple writes to hide memory latency
  - Coalesce consecutive writes to the same address

```verilog
// Example write buffer
module WRITE_BUFFER (
    input wire clk, reset,
    input wire [31:0] address,
    input wire [31:0] data,
    input wire write_req,
    output reg buffer_full,
    output reg [31:0] mem_address,
    output reg [31:0] mem_data,
    output reg mem_write
);
    // Buffer parameters
    parameter BUFFER_SIZE = 8;
    
    // Buffer storage
    reg [31:0] buffer_addr [0:BUFFER_SIZE-1];
    reg [31:0] buffer_data [0:BUFFER_SIZE-1];
    reg buffer_valid [0:BUFFER_SIZE-1];
    
    // Buffer pointers
    reg [2:0] write_ptr;
    reg [2:0] read_ptr;
    
    // Buffer management
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (integer i = 0; i < BUFFER_SIZE; i = i + 1)
                buffer_valid[i] <= 1'b0;
            write_ptr <= 3'b0;
            read_ptr <= 3'b0;
            buffer_full <= 1'b0;
            mem_write <= 1'b0;
        end
        else begin
            // Process write request
            if (write_req && !buffer_full) begin
                buffer_addr[write_ptr] <= address;
                buffer_data[write_ptr] <= data;
                buffer_valid[write_ptr] <= 1'b1;
                write_ptr <= write_ptr + 1;
                
                // Check if buffer is full
                if (write_ptr + 1 == read_ptr || (write_ptr == BUFFER_SIZE-1 && read_ptr == 0))
                    buffer_full <= 1'b1;
            end
            
            // Process memory write
            if (buffer_valid[read_ptr]) begin
                mem_address <= buffer_addr[read_ptr];
                mem_data <= buffer_data[read_ptr];
                mem_write <= 1'b1;
                buffer_valid[read_ptr] <= 1'b0;
                read_ptr <= read_ptr + 1;
                buffer_full <= 1'b0;
            end
            else begin
                mem_write <= 1'b0;
            end
        end
    end
endmodule
```

- **Memory Hierarchy**: Implement a complete memory hierarchy with multiple cache levels.
  - L1 cache (split I and D)
  - L2 unified cache
  - Main memory
  - Virtual memory system

### Pipeline Diagram with Improvements

Below is a conceptual diagram showing where the major improvements would be placed in the pipeline:

```
IF Stage:
  - Add branch predictor
  - Add instruction cache
  - Add prefetch buffer

ID Stage:
  - Add hazard detection unit
  - Add early branch resolution logic
  - Add register renaming logic

EX Stage:
  - Add forwarding unit inputs
  - Add reservation station for out-of-order execution
  - Add multiple execution units (ALUs, branch units, etc.)

MEM Stage:
  - Add data cache
  - Add write buffer
  - Add memory ordering logic

WB Stage:
  - Add reorder buffer for in-order completion
  - Add commit logic for speculative execution
  - Support for multiple simultaneous writeback operations
```

By implementing these improvements, the MIPS pipeline would achieve significantly better performance by reducing the impact of hazards, enabling more efficient memory access, and allowing for more optimized instruction execution. Each of these modifications would require careful design to maintain correctness while improving performance.
