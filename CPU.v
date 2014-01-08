// CPU.v
////////////////////////////////////////////////////////////////////////////////

// subset of MIPS instructions executing five stages pipeline CPU
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
module CPU (clock, reset);
    parameter INSTR_MEM_SIZE = 1024;
    parameter DATA_MEM_SIZE = 4096;
    input wire clock;
    input wire reset;

    wire [31:0] pc_next;
    wire [31:0] pc;
    // ProgramCounter (clock, reset, pc_next, pc);
    ProgramCounter ProgramCounter_0 (clock, reset, pc_next, pc);

    wire [31:0] pc_plus_four;
    // PCPlus4 (pc, pc_plus_four);
    PCPlus4 PCPlus4_0 (pc, pc_plus_four);

    wire [31:0] instruction;
    // InstructionMemory #(parameter N = 1024) (Address, Instruction);
    InstructionMemory #(INSTR_MEM_SIZE) InstructionMemory_0 (pc, instruction);

    wire [31:0] IF_ID_pc_plus_four;
    wire [31:0] IF_ID_instruction;
    // IF_ID (clock, pc_plus_four, IF_ID_pc_plus_four, instruction,
            // IF_ID_instruction);
    IF_ID IF_ID_0 (clock, pc_plus_four, IF_ID_pc_plus_four, instruction,
            IF_ID_instruction);

    wire [5:0] opcode;
    assign opcode = IF_ID_instruction[31:26];
    wire [4:0] rs;
    assign rs = IF_ID_instruction[25:21];
    wire [4:0] rt;
    assign rt = IF_ID_instruction[20:16];
    wire [4:0] rd;
    assign rd = IF_ID_instruction[15:11];
    //wire [4:0] shamt;
    //assign shamt = IF_ID_instruction[10:6];
    //wire [5:0] funct;
    //assign funct = IF_ID_instruction[5:0];
    wire [15:0] immediate;
    assign immediate = IF_ID_instruction[15:0];
    //wire [25:0] address;
    //assign address = IF_ID_instruction[25:0];

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

    wire [31:0] RegReadDataA;
    wire [31:0] RegReadDataB;
    wire [31:0] RegWriteData;
    // Registers (clock, reset, ReadAddressA, ReadDataA, ReadAddressB,
            // ReadDataB, WriteEnable, WriteAddress, WriteData);
    Registers Registers_0 (clock, reset, rs, RegReadDataA, rt, RegReadDataB,
            ID_EX_RegWrite, RegWriteAddress, RegWriteData);

    wire [31:0] extended;
    // SignExtender (immediate, extended);
    SignExtender SignExtender_0 (immediate, extended);

    wire [31:0] ID_EX_pc_plus_four;
    wire [31:0] ID_EX_RegReadDataA;
    wire [31:0] ID_EX_RegReadDataB;
    wire [31:0] ID_EX_extended;
    wire ID_EX_RegWrite;
    wire ID_EX_RegDst;
    wire ID_EX_MemRead;
    wire ID_EX_MemWrite;
    wire ID_EX_MemToReg;
    wire ID_EX_Branch;
    wire ID_EX_ALUSrc;
    wire [1:0] ID_EX_ALUOp;
    wire [4:0] ID_EX_rt;
    wire [4:0] ID_EX_rd;
    // ID_EX (clock, IF_ID_pc_plus_four, ID_EX_pc_plus_four, RegReadDataA,
            // ID_EX_RegReadDataA, RegReadDataB, ID_EX_RegReadDataB, extended,
            // ID_EX_extended, RegWrite, ID_EX_RegWrite, RegDst, ID_EX_RegDst,
            // MemRead, ID_EX_MemRead, MemWrite, ID_EX_MemWrite, MemToReg,
            // ID_EX_MemToReg, Branch, ID_EX_Branch, ALUSrc, ID_EX_ALUSrc,
            // ALUOp, ID_EX_ALUOp, rt, ID_EX_rt, rd, ID_EX_rd);
    ID_EX ID_EX_0 (clock, IF_ID_pc_plus_four, ID_EX_pc_plus_four, RegReadDataA,
            ID_EX_RegReadDataA, RegReadDataB, ID_EX_RegReadDataB, extended,
            ID_EX_extended, RegWrite, ID_EX_RegWrite, RegDst, ID_EX_RegDst,
            MemRead, ID_EX_MemRead, MemWrite, ID_EX_MemWrite, MemToReg,
            ID_EX_MemToReg, Branch, ID_EX_Branch, ALUSrc, ID_EX_ALUSrc, ALUOp,
            ID_EX_ALUOp, rt, ID_EX_rt, rd, ID_EX_rd);

    wire [31:0] ALUArgB;
    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(32) MuxALUSrc (ID_EX_RegReadDataB, ID_EX_extended, ID_EX_ALUSrc,
            ALUArgB);

    wire [31:0] ALUResult;
    wire Zero;
    // ALU #(parameter N = 32) (op, inA, inB, out, zero);
    ALU ALU_0 (ALUCtrl, ID_EX_RegReadDataA, ALUArgB, ALUResult, Zero);

    wire [5:0] funct;
    assign funct = ID_EX_extended[5:0];

    wire [3:0] ALUCtrl;
    // ALUControl (Funct, ALUOp, ALUCtrl);
    ALUControl ALUControl_0 (funct, ID_EX_ALUOp, ALUCtrl);

    wire [4:0] RegWriteAddress;
    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(5) MuxRegDst (ID_EX_rt, ID_EX_rd, ID_EX_RegDst, RegWriteAddress);

    wire [31:0] branch_address;
    // BranchAdder (pc_plus_four, extended_times_four, branch_address);
    BranchAdder BranchAdder_0 (ID_EX_pc_plus_four, (ID_EX_extended << 2),
            branch_address);

    wire bneOne;
    assign bneOne = IF_ID_instruction[26];

    wire pc_chooser;
    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(1) MuxBeqBne (Zero, ~Zero, bneOne, pc_chooser);

    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(32) MuxPCNext (pc_plus_four, branch_address,
            (ID_EX_Branch && pc_chooser), pc_next);

    wire [31:0] MemReadData;
    // Memory #(parameter N = 4096) (clock, Address, ReadEnable, ReadData,
            // WriteEnable, WriteData);
    Memory #(DATA_MEM_SIZE) DataMemory_0 (clock, ALUResult, ID_EX_MemRead,
            MemReadData, ID_EX_MemWrite, ID_EX_RegReadDataB);

    // mux2to1 #(parameter N = 1) (inA, inB, select, out);
    mux2to1 #(32) MuxMemtoReg (ALUResult, MemReadData, ID_EX_MemToReg,
            RegWriteData);
endmodule
////////////////////////////////////////////////////////////////////////////////
