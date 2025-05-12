
`timescale 1ns / 1ps
// Testbench for MIPS Datapath
module testbench();
    // Inputs
    reg clk;
    reg reset;
    
    // Clock count
    integer cycle_count;
    
    // Instantiate the PIPELINE module
    PIPELINE dut (
        .clk(clk),
        .reset(reset)
    );
    
    // Clock generation - 10ns period (5ns high, 5ns low)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 10ns period clock
    end
    
    // Test vectors
    initial begin
        // Create VCD file for waveform viewing if supported by simulator
        if ($test$plusargs("vcd")) begin
            $dumpfile("mips_datapath.vcd");
            $dumpvars(0, testbench);
        end
        
        // Initialize signals
        reset = 1;
        cycle_count = 0;
        
        // Release reset after 20ns
        #20 reset = 0;
        
        // Run for 24 cycles to ensure complete instruction pipeline flow
        #240;
        
        // End simulation
        $display("\nSimulation completed after %0d clock cycles", cycle_count);
        $finish;
    end
    
    // Monitor clock cycles
    always @(posedge clk) begin
        if (!reset) begin
            cycle_count = cycle_count + 1;
            $display("\n===================== Clock Cycle %0d =====================", cycle_count);
        end
    end
    
    // Monitor the instructions being executed
    always @(posedge clk) begin
        if (!reset) begin
            // Display PC and Instruction
            $display("IF Stage - PC: %h, Instruction: %b", dut.if_stage.PC, dut.if_stage.instruction);
            
            // Display pipeline register contents
            $display("IF/ID - NPC: %d, IR: %h", dut.if_stage.IF_ID_NPC, dut.if_stage.IF_ID_IR);
            $display("ID/EX - A(rs): %d, B(rt): %d, IMM: %h, RT: %d, RD: %d", 
                     dut.id_stage.ID_EX_A, dut.id_stage.ID_EX_B, dut.id_stage.ID_EX_IMM,
                     dut.id_stage.ID_EX_RT, dut.id_stage.ID_EX_RD);
            $display("ID/EX Control - WBCTRL: %b, MCTRL: %b, EXCTRL: %b", 
                     dut.id_stage.ID_EX_WBCTRL, dut.id_stage.ID_EX_MCTRL, dut.id_stage.ID_EX_EXCTRL);
            $display("EX/MEM - ALU_RESULT: %d, B: %d, WRITEREG: %d, ZERO: %b", 
                     dut.ex_stage.EX_MEM_ALU_RESULT, dut.ex_stage.EX_MEM_B, 
                     dut.ex_stage.EX_MEM_WRITEREG, dut.ex_stage.EX_MEM_ZERO);
            $display("EX/MEM Control - WBCTRL: %b, MCTRL: %b", 
                     dut.ex_stage.EX_MEM_WBCTRL, dut.ex_stage.EX_MEM_MCTRL);
            $display("MEM/WB - READ_DATA: %d, ALU_RESULT: %d, WRITEREG: %d", 
                     dut.mem_stage.MEM_WB_READ_DATA, dut.mem_stage.MEM_WB_ALU_RESULT, 
                     dut.mem_stage.MEM_WB_WRITEREG);
            $display("MEM/WB Control - WBCTRL: %b", dut.mem_stage.MEM_WB_WBCTRL);
            $display("WB - WRITEDATA: %d, REGWRITE: %b", 
                     dut.wb_stage.WB_WRITEDATA, dut.wb_stage.WB_REGWRITE);
            
            // Display first few registers (focus on r1, r2, r3)
            $display("\nRegister Contents:");
            $display("r0: %d", dut.id_stage.register_file.registers[0]);
            $display("r1: %d", dut.id_stage.register_file.registers[1]);
            $display("r2: %d", dut.id_stage.register_file.registers[2]);
            $display("r3: %d", dut.id_stage.register_file.registers[3]);
        end
    end
    
endmodule
