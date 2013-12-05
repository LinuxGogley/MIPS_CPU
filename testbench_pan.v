// testbench.v
////////////////////////////////////////////////////////////////////////////////

// testbench for the CPU
////////////////////////////////////////////////////////////////////////////////

// Dimitrios Paraschas (paraschas@gmail.com)
// Panagiotis Kremmydas (kmd178@gmail.com)
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
        // clock and reset signal generation
        #5 clock = 0;
        #5 clock = 1'b1;
        reset = 1'b0;
        #5 clock = ~clock;
        #5 clock = ~clock;
        #10;
        clock = 1'b1;
        reset = 1'b1;

        // initialize the register file
        for (i = 0; i < 32; i = i+1)
            CPU_0.DataPath_0.Registers_0.data[i] = i;  // $r0 = 0 in MIPS

        // initialize the memory data
        $readmemh("program.hex", CPU_0.DataPath_0.Memory_0.data);

        $dumpfile("lab7_cpu_pan.vcd");
        $dumpvars(0, cpu_tb);

        #50;
        $finish;
    end  // initial

    always begin
        #5;  // wait 5 time units, that translates to a period of 10 time units
        clock = ~clock;
    end
endmodule
