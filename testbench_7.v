// testbench_7.v
////////////////////////////////////////////////////////////////////////////////

// a testbench using the suggested program of lab 7
////////////////////////////////////////////////////////////////////////////////

// Dimitrios Paraschas (paraschas@gmail.com)
////////////////////////////////////////////////////////////////////////////////

// inf.uth.gr
// ce232 Computer Organization and Design
////////////////////////////////////////////////////////////////////////////////

// lab 8
// Implementation of a collection of MIPS instructions parsing CPU.
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

module cpu_tb;
    // unchangeable parameters
    // http://www.edaboard.com/thread194570.html
    localparam IMS = 32;
    localparam DMS = 64;
    reg clock, reset;  // clock and reset signals
    integer i;

    // CPU (clock, reset);
    // module with multiple parameters
    // http://www.asic-world.com/verilog/para_modules1.html
    CPU #(.INSTR_MEM_SIZE(IMS), .DATA_MEM_SIZE(DMS)) CPU_0 (clock, reset);

    always begin
        #5;
        clock = ~clock;
    end

    initial begin
        $dumpfile("dumpfile_7.vcd");
        $dumpvars(0, cpu_tb);

        for (i = 0; i < IMS; i = i + 1)
            $dumpvars(1, CPU_0.InstructionMemory_0.data[i]);

        for (i = 0; i < DMS; i = i + 1)
            $dumpvars(1, CPU_0.DataMemory_0.data[i]);

        // initialize the register file
        for (i = 0; i < 32; i = i + 1)
            CPU_0.Registers_0.data[i] = i;

        // initialize the memory data
        //$readmemh("program_7.mhex", CPU_0.InstructionMemory_0.data);
        $readmemb("program_7.mbin", CPU_0.InstructionMemory_0.data);

        // clock and reset signal generation
        clock = 0;
        reset = 0;
        #5;
        reset = 1;

        #55;
        $finish;
    end  // initial

endmodule
