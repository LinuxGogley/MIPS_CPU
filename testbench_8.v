// testbench_8.v
////////////////////////////////////////////////////////////////////////////////

// CPU testbench using the suggested program of lab 8
////////////////////////////////////////////////////////////////////////////////

// Dimitrios Paraschas (paraschas@gmail.com)
////////////////////////////////////////////////////////////////////////////////

// inf.uth.gr
// ce232 Computer Organization and Design
////////////////////////////////////////////////////////////////////////////////

// lab 8
// implementation of a subset of MIPS instructions executing CPU
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

module cpu_tb;
    // unchangeable parameters
    // http://www.edaboard.com/thread194570.html
    localparam N_REGISTERS = 32;
    localparam IMS = 32;
    localparam DMS = 32;
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
        $dumpfile("dumpfile_8.vcd");
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
        //$readmemh("program_8.mhex", CPU_0.InstructionMemory_0.data);
        $readmemb("program_8.mbin", CPU_0.InstructionMemory_0.data);

        // TODO
        // zero waiting time is preferred
        //#0;
        #5;
        reset = 1;

        // initialize the registers
        for (i = 0; i < N_REGISTERS; i = i + 1)
            CPU_0.Registers_0.data[i] = i;

        #55;
        #30;

        if ((CPU_0.Registers_0.data[1] == 1) &&
            (CPU_0.Registers_0.data[2] == 2) &&
            (CPU_0.Registers_0.data[3] == 3) &&
            (CPU_0.Registers_0.data[4] == 4) &&
            (CPU_0.Registers_0.data[5] == 5) &&
            (CPU_0.Registers_0.data[6] == 6) &&
            (CPU_0.Registers_0.data[7] == 7) &&
            (CPU_0.Registers_0.data[8] == 8) &&

            (CPU_0.Registers_0.data[9] == 1) &&

            (CPU_0.Registers_0.data[10] == 10) &&
            (CPU_0.Registers_0.data[11] == 11) &&
            (CPU_0.Registers_0.data[12] == 12) &&
            (CPU_0.Registers_0.data[13] == 13) &&
            (CPU_0.Registers_0.data[14] == 14) &&
            (CPU_0.Registers_0.data[15] == 15) &&
            (CPU_0.Registers_0.data[16] == 16) &&
            (CPU_0.Registers_0.data[17] == 17) &&

            (CPU_0.Registers_0.data[18] == 12) &&

            (CPU_0.Registers_0.data[19] == 19) &&
            (CPU_0.Registers_0.data[20] == 20) &&
            (CPU_0.Registers_0.data[21] == 21) &&
            (CPU_0.Registers_0.data[22] == 22) &&
            (CPU_0.Registers_0.data[23] == 23) &&
            (CPU_0.Registers_0.data[24] == 24) &&
            (CPU_0.Registers_0.data[25] == 25) &&
            (CPU_0.Registers_0.data[26] == 26) &&
            (CPU_0.Registers_0.data[27] == 27) &&
            (CPU_0.Registers_0.data[28] == 28) &&
            (CPU_0.Registers_0.data[29] == 29) &&
            (CPU_0.Registers_0.data[30] == 30) &&
            (CPU_0.Registers_0.data[31] == 31) &&

            (CPU_0.DataMemory_0.data[16] == 12) &&

            // NOTE
            // the program counter would be 20 just after the program
            // execution in a sinlge cycle CPU.
            (CPU_0.ProgramCounter_0.pc == 32)) begin
            $display("\n");
            $display("program 8 completed successfully");
            $display("\n");
        end  // if
        else begin
            $display("\n");
            $display("program 8 failed");
            $display("\n");
        end  // else

        $finish;
    end  // initial

endmodule
