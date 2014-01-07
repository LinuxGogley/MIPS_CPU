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
    localparam N_REGISTERS = 32;
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
        // specify a VCD dump file and variables
        // http://verilog.renerta.com/source/vrg00056.htm
        $dumpfile("dumpfile_7.vcd");
        $dumpvars(0, cpu_tb);
        for (i = 0; i < N_REGISTERS; i = i + 1)
            $dumpvars(1, CPU_0.Registers_0.data[i]);
        for (i = 0; i < IMS; i = i + 1)
            $dumpvars(1, CPU_0.InstructionMemory_0.data[i]);
        for (i = 0; i < DMS; i = i + 1)
            $dumpvars(1, CPU_0.DataMemory_0.data[i]);

        // clock and reset signals
        clock = 1;
        reset = 0;

        // load the program to the instruction memory
        //$readmemh("program_7.mhex", CPU_0.InstructionMemory_0.data);
        $readmemb("program_7.mbin", CPU_0.InstructionMemory_0.data);

        // TODO
        // zero waiting time is preferred
        //#0;
        #5;
        reset = 1;

        // initialize the registers
        for (i = 0; i < N_REGISTERS; i = i + 1)
            CPU_0.Registers_0.data[i] = i;

        #55;

        if ((CPU_0.Registers_0.data[8] == 24) &&
            (CPU_0.Registers_0.data[9] == 5) &&
            (CPU_0.Registers_0.data[17] == 16) &&
            (CPU_0.Registers_0.data[18] == 0)) begin
            $display("\n");
            $display("program 7 completed successfully");
            $display("\n");
        end  // if
        else begin
            $display("\n");
            $display("program 7 failed");
            $display("\n");
        end  // else

        $finish;
    end  // initial

endmodule
