// CPU.v
////////////////////////////////////////////////////////////////////////////////

// subset of MIPS instructions executing CPU
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

    wire [31:0] instruction;

    // InstructionMemory #(parameter N = 1024) (Address, Instruction);
    InstructionMemory #(INSTR_MEM_SIZE) InstructionMemory_0 (pc, instruction);

    wire [5:0] opcode;
    assign opcode = instruction[31:26];
    wire [4:0] rs;
    assign rs = instruction[25:21];
    wire [4:0] rt;
    assign rt = instruction[20:16];
    wire [4:0] rd;
    assign rd = instruction[15:11];
    wire [4:0] shamt;
    assign shamt = instruction[10:6];
    wire [5:0] funct;
    assign funct = instruction[5:0];
    wire [15:0] immediate;
    assign immediate = instruction[15:0];
    wire [25:0] address;
    assign address = instruction[25:0];

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
    Control Control_0 (opcode, RegWrite, RegDst, MemRead, MemWrite,
            MemToReg, Branch, ALUSrc, ALUOp);

    wire [4:0] RegWriteAddress;

    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(5) MuxRegDst (rt, rd, RegDst, RegWriteAddress);

    wire [31:0] RegReadDataA;
    wire [31:0] RegReadDataB;
    wire [31:0] RegWriteData;

    // Registers (clock, reset, ReadAddressA, ReadDataA, ReadAddressB,
            // ReadDataB, WriteEnable, WriteAddress, WriteData);
    Registers Registers_0 (clock, reset, rs, RegReadDataA, rt, RegReadDataB,
            RegWrite, RegWriteAddress, RegWriteData);

    wire [31:0] extended;

    // SignExtender (immediate, extended);
    SignExtender SignExtender_0 (immediate, extended);

    wire [3:0] ALUCtrl;

    // ALUControl (Funct, ALUOp, ALUCtrl);
    ALUControl ALUControl_0 (funct, ALUOp, ALUCtrl);

    wire [31:0] ALUArgB;

    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(32) MuxALUSrc (RegReadDataB, extended, ALUSrc, ALUArgB);

    wire [31:0] ALUResult;
    wire Zero;

    // ALU #(parameter N = 32) (op, inA, inB, out, zero);
    ALU ALU_0 (ALUCtrl, RegReadDataA, ALUArgB, ALUResult, Zero);

    // TODO
    // this is hacky, replace it with a more clear implementation
    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(32) MuxPCNext (pc_four, (pc_four + (extended << 2)),
            (Branch && (instruction[26] ? ~Zero : Zero)), pc_next);

    wire [31:0] MemReadData;

    // Memory #(parameter N = 4096) (clock, Address, ReadEnable, ReadData,
            // WriteEnable, WriteData);
    Memory #(DATA_MEM_SIZE) DataMemory_0 (clock, ALUResult, MemRead,
            MemReadData, MemWrite, RegReadDataB);

    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(32) MuxMemtoReg (ALUResult, MemReadData, MemToReg, RegWriteData);

endmodule
