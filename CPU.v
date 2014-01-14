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

    wire [31:0] ID_pc_plus_four;
    wire [31:0] ID_instruction;

    // IF/ID pipeline registers (1st pipeline registers)
    IF_ID IF_ID_0 (
        .clock(clock),
        .pc_plus_four(pc_plus_four),
        .ID_pc_plus_four(ID_pc_plus_four),
        .instruction(instruction),
        .ID_instruction(ID_instruction)
    );

    wire [5:0] opcode;
    assign opcode = ID_instruction[31:26];
    wire [4:0] rs;
    assign rs = ID_instruction[25:21];
    wire [4:0] rt;
    assign rt = ID_instruction[20:16];
    wire [4:0] rd;
    assign rd = ID_instruction[15:11];
    // NOTE
    // currently not used
    //wire [4:0] shamt;
    //assign shamt = ID_instruction[10:6];
    wire [5:0] funct;
    assign funct = ID_instruction[5:0];
    wire [15:0] immediate;
    assign immediate = ID_instruction[15:0];
    // NOTE
    // currently not used
    //wire [25:0] address;
    //assign address = ID_instruction[25:0];

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
    // debug wire EX_instruction
    wire [31:0] EX_instruction;

    wire [31:0] EX_pc_plus_four;
    wire [31:0] EX_RegReadDataA;
    wire [31:0] EX_RegReadDataB;
    wire [31:0] EX_extended;

    wire EX_RegWrite;
    wire EX_RegDst;
    wire EX_MemRead;
    wire EX_MemWrite;
    wire EX_MemToReg;
    wire EX_Branch;
    wire EX_ALUSrc;
    wire [1:0] EX_ALUOp;

    wire [4:0] EX_rt;
    wire [4:0] EX_rd;

    // ID/EX pipeline registers (2nd pipeline registers)
    ID_EX ID_EX_0 (
        .clock(clock),
        // TODO
        // debug port
        .ID_instruction(ID_instruction),
        // TODO
        // debug port
        .EX_instruction(EX_instruction),

        .ID_pc_plus_four(ID_pc_plus_four),
        .EX_pc_plus_four(EX_pc_plus_four),
        .RegReadDataA(RegReadDataA),
        .EX_RegReadDataA(EX_RegReadDataA),
        .RegReadDataB(RegReadDataB),
        .EX_RegReadDataB(EX_RegReadDataB),
        .extended(extended),
        .EX_extended(EX_extended),

        .RegWrite(RegWrite),
        .EX_RegWrite(EX_RegWrite),
        .RegDst(RegDst),
        .EX_RegDst(EX_RegDst),
        .MemRead(MemRead),
        .EX_MemRead(EX_MemRead),
        .MemWrite(MemWrite),
        .EX_MemWrite(EX_MemWrite),
        .MemToReg(MemToReg),
        .EX_MemToReg(EX_MemToReg),
        .Branch(Branch),
        .EX_Branch(EX_Branch),
        .ALUSrc(ALUSrc),
        .EX_ALUSrc(EX_ALUSrc),
        .ALUOp(ALUOp),
        .EX_ALUOp(EX_ALUOp),

        .rt(rt),
        .EX_rt(EX_rt),
        .rd(rd),
        .EX_rd(EX_rd)
    );

    wire [31:0] ALUArgB;

    mux2to1 #(
        .WIDTH(32)
    ) MuxALUSrc (
        .inA(EX_RegReadDataB),
        .inB(EX_extended),
        .select(EX_ALUSrc),
        .out(ALUArgB)
    );

    wire [31:0] ALUResult;
    wire Zero;

    ALU #(
        .WIDTH(32)
    ) ALU_0 (
        .op(ALUCtrl),
        .inA(EX_RegReadDataA),
        .inB(ALUArgB),
        .out(ALUResult),
        .zero(Zero)
    );

    wire [5:0] EX_funct;
    assign EX_funct = EX_extended[5:0];

    wire [3:0] ALUCtrl;

    ALUControl ALUControl_0 (
        .Funct(EX_funct),
        .ALUOp(EX_ALUOp),
        .ALUCtrl(ALUCtrl)
    );

    wire [4:0] RegWriteAddress;

    mux2to1 #(
        .WIDTH(5)
    ) MuxRegDst (
        .inA(EX_rt),
        .inB(EX_rd),
        .select(EX_RegDst),
        .out(RegWriteAddress)
    );

    wire [31:0] branch_address;

    BranchAdder BranchAdder_0 (
        .pc_plus_four(EX_pc_plus_four),
        .extended_times_four(EX_extended << 2),
        .branch_address(branch_address)
    );

    wire bneOne;
    assign bneOne = ID_instruction[26];

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
