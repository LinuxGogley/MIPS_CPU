// DataPath.v
////////////////////////////////////////////////////////////////////////////////

// connect the required modules to create the data path of the CPU
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

    // Memory (addr, ren, dout, wen, din);
    Memory Memory_0 (pc, ren, dout, wen, din);

    // Registers (clock, reset, raA, rdA, raB, rdB, wen, wa, wd);
    Registers Registers_0 (clock, reset, dout[25:21], rdA, dout[20:16], rdB,
            RegWrite, dout[15:11], out);

    // ALU (op, inA, inB, out, zero);
    ALU ALU_0 (ALUControl, rdA, rdB, out, Zero);
endmodule
