// CPU.v
////////////////////////////////////////////////////////////////////////////////

// connect the library modules to create the CPU
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

module CPU (clock, reset);
    parameter INSTR_MEM_SIZE = 1024;
    parameter DATA_MEM_SIZE = 4096;
    input wire clock;
    input wire reset;

    wire [31:0] pc_next;
    wire [31:0] pc;

    // ProgramCounter (clock, reset, pc_next, pc);
    ProgramCounter ProgramCounter_0 (clock, reset, pc_next, pc);

    wire [31:0] pc_four;

    // PCPlus4 (clock, reset, pc, pc_four);
    PCPlus4 PCPlus4_0 (clock, reset, pc, pc_four);

    wire [31:0] instr;

    // InstructionMemory (addr, dout);
    InstructionMemory #(INSTR_MEM_SIZE) InstructionMemory_0 (pc, instr);

    wire RegWrite;
    wire RegDst;
    wire MemRead;
    wire MemWrite;
    wire MemToReg;
    wire Branch;
    wire ALUSrc;
    wire [1:0] ALUOp;

    // Control (Opcode, RegWrite, RegDst, MemRead, MemWrite, MemToReg, Branch,
            // ALUSrc, ALUOp);
    Control Control_0 (instr[31:26], RegWrite, RegDst, MemRead, MemWrite,
            MemToReg, Branch, ALUSrc, ALUOp);

    wire [4:0] RegWriteAddress;

    // mux2to1 (inA, inB, select, out);
    mux2to1 #(5) MuxRegDst (instr[20:16], instr[15:11], RegDst, RegWriteAddress);

    wire [31:0] RegReadDataA;
    wire [31:0] RegReadDataB;
    wire [31:0] RegWriteData;

    // Registers (clock, reset, ReadAddressA, ReadDataA, ReadAddressB,
            // ReadDataB, WriteEnable, WriteAddress, WriteData);
    Registers Registers_0 (clock, reset, instr[25:21], RegReadDataA,
            instr[20:16], RegReadDataB, RegWrite, RegWriteAddress,
            RegWriteData);

    wire [31:0] extended;

    // SignExtender (immediate, extended);
    SignExtender SignExtender_0 (instr[15:0], extended);

    wire [3:0] ALUCtrl;

    // ALUControl (Funct, ALUOp, ALUCtrl);
    ALUControl ALUControl_0 (instr[5:0], ALUOp, ALUCtrl);

    wire [31:0] ALUArgB;

    // mux2to1 (inA, inB, select, out);
    mux2to1 #(32) MuxALUSrc (RegReadDataB, extended, ALUSrc, ALUArgB);

    wire [31:0] ALUResult;
    wire Zero;

    // ALU (op, inA, inB, out, Zero);
    ALU ALU_0 (ALUCtrl, RegReadDataA, ALUArgB, ALUResult, Zero);

    // mux2to1 (inA, inB, select, out);
    mux2to1 #(32) MuxPCNext (pc_four, (pc_four + (extended << 2)),
            (Branch & (instr[26] ? ~Zero : Zero)), pc_next);

    wire [31:0] MemReadData;

    // Memory (addr, ren, dout, wen, din);
    Memory #(DATA_MEM_SIZE) DataMemory_0 (ALUResult, MemRead, MemReadData,
            MemWrite, RegReadDataB);

    // mux2to1 (inA, inB, select, out);
    mux2to1 #(32) MuxMemtoReg (ALUResult, MemReadData, MemToReg, RegWriteData);

endmodule
