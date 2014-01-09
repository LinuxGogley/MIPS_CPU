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
module CPU #(
    parameter INSTR_MEM_SIZE = 1024,
    parameter DATA_MEM_SIZE = 4096
    ) (
    input wire clock,
    input wire reset
    );

    wire [31:0] pc_next;
    wire [31:0] pc;

    ProgramCounter ProgramCounter_0 (
        .clock(clock),
        .reset(reset),
        .pc_next(pc_next),
        .pc(pc)
    );

    wire [31:0] pc_plus_four;

    PCPlus4 PCPlus4_0 (
        .pc(pc),
        .pc_plus_four(pc_plus_four)
    );

    wire [31:0] instruction;

    InstructionMemory #(
        .SIZE(INSTR_MEM_SIZE)
    ) InstructionMemory_0 (
        .Address(pc),
        .Instruction(instruction)
    );

    wire [31:0] IF_ID_pc_plus_four;
    wire [31:0] IF_ID_instruction;

    IF_ID IF_ID_0 (
        .clock(clock),
        .pc_plus_four(pc_plus_four),
        .IF_ID_pc_plus_four(IF_ID_pc_plus_four),
        .instruction(instruction),
        .IF_ID_instruction(IF_ID_instruction)
    );

    wire [5:0] opcode;
    assign opcode = IF_ID_instruction[31:26];
    wire [4:0] rs;
    assign rs = IF_ID_instruction[25:21];
    wire [4:0] rt;
    assign rt = IF_ID_instruction[20:16];
    wire [4:0] rd;
    assign rd = IF_ID_instruction[15:11];
    // NOTE
    // currently not used
    //wire [4:0] shamt;
    //assign shamt = IF_ID_instruction[10:6];
    wire [5:0] funct;
    assign funct = IF_ID_instruction[5:0];
    wire [15:0] immediate;
    assign immediate = IF_ID_instruction[15:0];
    // NOTE
    // currently not used
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

    Control Control_0 (
        .Opcode(opcode),
        .RegWrite(RegWrite),
        .RegDst(RegDst),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .MemToReg(MemToReg),
        .Branch(Branch),
        .ALUSrc(ALUSrc),
        .ALUOp(ALUOp)
    );

    wire [31:0] RegReadDataA;
    wire [31:0] RegReadDataB;
    wire [31:0] RegWriteData;

    Registers Registers_0 (
        .clock(clock),
        .reset(reset),
        .ReadAddressA(rs),
        .ReadDataA(RegReadDataA),
        .ReadAddressB(rt),
        .ReadDataB(RegReadDataB),
        .WriteEnable(ID_EX_RegWrite),
        .WriteAddress(RegWriteAddress),
        .WriteData(RegWriteData)
    );

    wire [31:0] extended;

    SignExtender SignExtender_0 (
        .immediate(immediate),
        .extended(extended)
    );

    // TODO
    // debug wire ID_EX_instruction
    wire [31:0] ID_EX_instruction;

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

    ID_EX ID_EX_0 (
        .clock(clock),
        // TODO
        // debug port
        .IF_ID_instruction(IF_ID_instruction),
        // TODO
        // debug port
        .ID_EX_instruction(ID_EX_instruction),

        .IF_ID_pc_plus_four(IF_ID_pc_plus_four),
        .ID_EX_pc_plus_four(ID_EX_pc_plus_four),
        .RegReadDataA(RegReadDataA),
        .ID_EX_RegReadDataA(ID_EX_RegReadDataA),
        .RegReadDataB(RegReadDataB),
        .ID_EX_RegReadDataB(ID_EX_RegReadDataB),
        .extended(extended),
        .ID_EX_extended(ID_EX_extended),
        .RegWrite(RegWrite),
        .ID_EX_RegWrite(ID_EX_RegWrite),
        .RegDst(RegDst),
        .ID_EX_RegDst(ID_EX_RegDst),
        .MemRead(MemRead),
        .ID_EX_MemRead(ID_EX_MemRead),
        .MemWrite(MemWrite),
        .ID_EX_MemWrite(ID_EX_MemWrite),
        .MemToReg(MemToReg),
        .ID_EX_MemToReg(ID_EX_MemToReg),
        .Branch(Branch),
        .ID_EX_Branch(ID_EX_Branch),
        .ALUSrc(ALUSrc),
        .ID_EX_ALUSrc(ID_EX_ALUSrc),
        .ALUOp(ALUOp),
        .ID_EX_ALUOp(ID_EX_ALUOp),
        .rt(rt),
        .ID_EX_rt(ID_EX_rt),
        .rd(rd),
        .ID_EX_rd(ID_EX_rd)
    );

    wire [31:0] ALUArgB;

    mux2to1 #(
        .WIDTH(32)
    ) MuxALUSrc (
        .inA(ID_EX_RegReadDataB),
        .inB(ID_EX_extended),
        .select(ID_EX_ALUSrc),
        .out(ALUArgB)
    );

    wire [31:0] ALUResult;
    wire Zero;

    ALU #(
        .WIDTH(32)
    ) ALU_0 (
        .op(ALUCtrl),
        .inA(ID_EX_RegReadDataA),
        .inB(ALUArgB),
        .out(ALUResult),
        .zero(Zero)
    );

    wire [5:0] ID_EX_funct;
    assign ID_EX_funct = ID_EX_extended[5:0];

    wire [3:0] ALUCtrl;

    ALUControl ALUControl_0 (
        .Funct(ID_EX_funct),
        .ALUOp(ID_EX_ALUOp),
        .ALUCtrl(ALUCtrl)
    );

    wire [4:0] RegWriteAddress;

    mux2to1 #(
        .WIDTH(5)
    ) MuxRegDst (
        .inA(ID_EX_rt),
        .inB(ID_EX_rd),
        .select(ID_EX_RegDst),
        .out(RegWriteAddress)
    );

    // EX/MEM pipeline registers (3rd pipeline registers)

    wire [31:0] branch_address;

    BranchAdder BranchAdder_0 (
        .pc_plus_four(ID_EX_pc_plus_four),
        .extended_times_four(ID_EX_extended << 2),
        .branch_address(branch_address)
    );

    wire bneOne;
    assign bneOne = IF_ID_instruction[26];

    wire pc_chooser;

    mux2to1 #(
        .WIDTH(1)
    ) MuxBeqBne (
        .inA(Zero),
        .inB(~Zero),
        .select(bneOne),
        .out(pc_chooser)
    );

    mux2to1 #(
        .WIDTH(32)
    ) MuxPCNext (
        .inA(pc_plus_four),
        .inB(branch_address),
        .select(ID_EX_Branch && pc_chooser),
        .out(pc_next)
    );

    wire [31:0] MemReadData;

    Memory #(
        .SIZE(DATA_MEM_SIZE)
    ) DataMemory_0 (
        .clock(clock),
        .Address(ALUResult),
        .ReadEnable(ID_EX_MemRead),
        .ReadData(MemReadData),
        .WriteEnable(ID_EX_MemWrite),
        .WriteData(ID_EX_RegReadDataB)
    );

    mux2to1 #(
        .WIDTH(32)
    ) MuxMemtoReg (
        .inA(ALUResult),
        .inB(MemReadData),
        .select(ID_EX_MemToReg),
        .out(RegWriteData)
    );
endmodule
////////////////////////////////////////////////////////////////////////////////
