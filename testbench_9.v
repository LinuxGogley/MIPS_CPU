// testbench_9.v
////////////////////////////////////////////////////////////////////////////////

// CPU testbench using the suggested program of lab 9
////////////////////////////////////////////////////////////////////////////////

// Dimitrios Paraschas (paraschas@gmail.com)
////////////////////////////////////////////////////////////////////////////////

// inf.uth.gr
// ce232 Computer Organization and Design
////////////////////////////////////////////////////////////////////////////////

// lab 9
// implementation of a subset of MIPS instruction five stages pipeline CPU
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

// modules
////////////////////////////////////////////////////////////////////////////////
module cpu_tb;
    // unchangeable parameters
    // http://www.edaboard.com/thread194570.html
    localparam N_REGISTERS = 32;
    localparam IMS = 64;
    localparam DMS = 32;
    reg clock, reset;  // clock and reset signals
    integer i;
    integer tests_passed;

    // CPU (clock, reset);
    // module with multiple parameters
    // http://www.asic-world.com/verilog/para_modules1.html
    CPU #(
        .INSTR_MEM_SIZE(IMS),
        .DATA_MEM_SIZE(DMS)
    ) CPU_0 (
        .clock(clock),
        .reset(reset)
    );

    always begin
        #5;
        clock = ~clock;
    end

    initial begin
        // specify a VCD dump file and variables
        // http://verilog.renerta.com/source/vrg00056.htm
        $dumpfile("dumpfile_9.vcd");
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

        // initialize the data memory
        for (i = 0; i < DMS; i = i + 1)
            CPU_0.DataMemory_0.data[i] = i;

        // load the program to the instruction memory
        //$readmemh("program_9.mhex", CPU_0.InstructionMemory_0.data);
        $readmemb("program_9.mbin", CPU_0.InstructionMemory_0.data);

        $display("\n");
        $display("Program 9 loaded and running.\n");

        // TODO
        // zero waiting time is preferred
        //#0;
        #5;
        reset = 1;

        // initialize the registers
        for (i = 0; i < N_REGISTERS; i = i + 1)
            CPU_0.Registers_0.data[i] = i;

        #65;
        #65;
        #5;
        #30;
        #10;
        #20;

        tests_passed = 0;

        // verify that the registers have the correct values
        $display("\n");
        if ((CPU_0.Registers_0.data[1] == 1) &&

            (CPU_0.Registers_0.data[2] == 28) &&

            (CPU_0.Registers_0.data[3] == 3) &&

            (CPU_0.Registers_0.data[4] == 30) &&

            (CPU_0.Registers_0.data[5] == 5) &&
            (CPU_0.Registers_0.data[6] == 6) &&
            (CPU_0.Registers_0.data[7] == 7) &&

            (CPU_0.Registers_0.data[8] == 24) &&
            (CPU_0.Registers_0.data[9] == 58) &&

            (CPU_0.Registers_0.data[10] == 10) &&
            (CPU_0.Registers_0.data[11] == 11) &&
            (CPU_0.Registers_0.data[12] == 12) &&

            (CPU_0.Registers_0.data[13] == 31) &&
            (CPU_0.Registers_0.data[14] == 31) &&

            (CPU_0.Registers_0.data[15] == 15) &&
            (CPU_0.Registers_0.data[16] == 16) &&
            (CPU_0.Registers_0.data[17] == 17) &&
            (CPU_0.Registers_0.data[18] == 18) &&

            (CPU_0.Registers_0.data[19] == 16) &&

            (CPU_0.Registers_0.data[20] == 20) &&
            (CPU_0.Registers_0.data[21] == 21) &&
            (CPU_0.Registers_0.data[22] == 22) &&
            (CPU_0.Registers_0.data[23] == 23) &&
            (CPU_0.Registers_0.data[24] == 24) &&
            (CPU_0.Registers_0.data[25] == 25) &&
            (CPU_0.Registers_0.data[26] == 26) &&
            (CPU_0.Registers_0.data[27] == 27) &&
            (CPU_0.Registers_0.data[28] == 28) &&

            (CPU_0.Registers_0.data[29] == 1) &&

            (CPU_0.Registers_0.data[30] == 30) &&
            (CPU_0.Registers_0.data[31] == 31)) begin
            tests_passed = tests_passed + 1;
            $display("Registers OK. All values are correct.");
        end  // if
        else begin
            $display("Registers mismatch. Wrong values.");
        end  // else
        $display("\n");

        // verify that the memory locations have the correct values
        if ((CPU_0.DataMemory_0.data[0] == 0) &&
            (CPU_0.DataMemory_0.data[1] == 1) &&
            (CPU_0.DataMemory_0.data[2] == 2) &&
            (CPU_0.DataMemory_0.data[3] == 3) &&
            (CPU_0.DataMemory_0.data[4] == 4) &&
            (CPU_0.DataMemory_0.data[5] == 5) &&
            (CPU_0.DataMemory_0.data[6] == 6) &&
            (CPU_0.DataMemory_0.data[7] == 7) &&

            (CPU_0.DataMemory_0.data[8] == 28) &&

            (CPU_0.DataMemory_0.data[9] == 9) &&
            (CPU_0.DataMemory_0.data[10] == 10) &&
            (CPU_0.DataMemory_0.data[11] == 11) &&

            (CPU_0.DataMemory_0.data[12] == 31) &&

            (CPU_0.DataMemory_0.data[13] == 13) &&
            (CPU_0.DataMemory_0.data[14] == 14) &&
            (CPU_0.DataMemory_0.data[15] == 15) &&
            (CPU_0.DataMemory_0.data[16] == 16) &&
            (CPU_0.DataMemory_0.data[17] == 17) &&
            (CPU_0.DataMemory_0.data[18] == 18) &&
            (CPU_0.DataMemory_0.data[19] == 19) &&
            (CPU_0.DataMemory_0.data[20] == 20) &&
            (CPU_0.DataMemory_0.data[21] == 21) &&
            (CPU_0.DataMemory_0.data[22] == 22) &&
            (CPU_0.DataMemory_0.data[23] == 23) &&
            (CPU_0.DataMemory_0.data[24] == 24) &&
            (CPU_0.DataMemory_0.data[25] == 25) &&
            (CPU_0.DataMemory_0.data[26] == 26) &&
            (CPU_0.DataMemory_0.data[27] == 27) &&
            (CPU_0.DataMemory_0.data[28] == 28) &&
            (CPU_0.DataMemory_0.data[29] == 29) &&
            (CPU_0.DataMemory_0.data[30] == 30) &&
            (CPU_0.DataMemory_0.data[31] == 31)) begin
            tests_passed = tests_passed + 1;
            $display("DataMemory OK. All values are correct.");
        end  // if
        else begin
            $display("DataMemory mismatch. Wrong values.");
        end  // else
        $display("\n");

        if (tests_passed == 2)
            $display("Program 9 successfully run.");
        else
            $display("Program 9 has failed.");
        $display("\n");

        $finish;
    end  // initial
endmodule
////////////////////////////////////////////////////////////////////////////////
