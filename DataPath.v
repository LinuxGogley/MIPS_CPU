// DataPath.v
////////////////////////////////////////////////////////////////////////////////

// connect the main modules to create the data path of the CPU
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

module DataPath (clock, reset, ALUControl, RegWrite, Opcode, Funct);
    input wire clock;
    input wire reset;
    input wire [3:0] ALUControl;
    input wire RegWrite;
    output wire [5:0] Opcode;
    output wire [5:0] Funct;

    wire [31:0] pc_new;
    wire [31:0] pc;
    wire ren;
    wire wen;
    wire [31:0] din;
    wire [31:0] dout;
    wire Zero;
    wire [31:0] out;
    wire [31:0] rdA;
    wire [31:0] rdB;

    // ProgramCounter (clock, reset, pc_new, pc);
    ProgramCounter ProgramCounter_0 (clock, reset, pc_new, pc);

    // PCPlus4 (clock, reset, pc, pc_new);
    PCPlus4 PCPlus4_0 (clock, reset, pc, pc_new);

    assign ren = 1;
    assign wen = 0;
    assign din = 32'b0;
    assign Opcode = dout[31:26];
    assign Funct = dout[5:0];

    // Memory (ren, wen, addr, din, dout);
    Memory Memory_0 (ren, wen, pc, din, dout);

    // RegFile (clock, reset, raA, raB, wa, wen, wd, rdA, rdB);
    RegFile RegFile_0 (clock, reset, dout[25:21], dout[20:16], dout[15:11],
            RegWrite, out, rdA, rdB);

    // ALU (out, zero, inA, inB, op);
    ALU ALU_0 (out, Zero, rdA, rdB, ALUControl);
endmodule
