// testbench.v
////////////////////////////////////////////////////////////////////////////////

// testbench for the CPU
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
    reg clock, reset;  // clock and reset signals
    integer i;

    // CPU (clock, reset);
    CPU CPU_0 (clock, reset);

    initial begin
        clock = 0;
        reset = 0;
        #10;
        reset = 1;
    end

    always begin
        #5;
        clock = ~clock;
    end

    initial begin
        $dumpfile("lab8_cpu.vcd");
        $dumpvars(0, cpu_tb);

        // TODO
        // monitor the outputs
        //$monitor("$t0 : %b", d, "  |  select = ", select, "  |  q = ", q);
        //$display("[%0d] serial: %c", $time, serial_out);

        // initialize the register file
        for (i = 0; i < 32; i = i + 1)
            CPU_0.Registers_0.data[i] = i;

        // initialize the memory data
        //$readmemh("program_7.mhex", CPU_0.InstructionMemory_0.data);
        //$readmemh("program_8.mhex", CPU_0.InstructionMemory_0.data);
        $readmemb("program_7.mbin", CPU_0.InstructionMemory_0.data);
        //$readmemb("program_8.mbin", CPU_0.InstructionMemory_0.data);

        // clock and reset signal generation
        clock = 0;
        reset = 0;
        #10;
        reset = 1;

        #50;
        $finish;
    end  // initial

endmodule
