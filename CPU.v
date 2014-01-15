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
        .Stall(Stall),
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

    // IF/ID pipeline registers (1st)
    IF_ID IF_ID_0 (
        .clock(clock),
        .WriteEnable(~Stall),
        .Flush(0),
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

    //wire PC_WriteEnable;
    //wire IF_ID_WriteEnable;
    //wire ControlStall;
    wire Stall;

    HazardDetection HazardDetection_0 (
        .EX_MemRead(EX_MemRead),
        .rs(rs),
        .rt(rt),
        .EX_rt(EX_rt),
        //.PC_WriteEnable(PC_WriteEnable),
        //.IF_ID_WriteEnable(IF_ID_WriteEnable),
        //.ControlStall(ControlStall)
        .Stall(Stall)
    );

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

    wire [8:0] BufferedControl;

    mux2to1 #(
        .WIDTH(9)
    ) MuxControlStall (
        // TODO 1
        // should we do something more complicated but readable here?
        .inA( {RegWrite, RegDst, MemRead, MemWrite, MemToReg, Branch, ALUSrc,
            ALUOp} ),
        .inB(9'b0),
        .select(Stall),
        .out(BufferedControl)
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
        .WriteEnable(WB_RegWrite),
        .WriteAddress(WB_RegWriteAddress),
        .WriteData(RegWriteData)
    );

    wire [31:0] extended;

    SignExtender SignExtender_0 (
        .immediate(immediate),
        .extended(extended)
    );

    // TODO 3
    // debug wire
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

    wire [4:0] EX_rs;
    wire [4:0] EX_rt;
    wire [4:0] EX_rd;

    // ID/EX pipeline registers (2nd)
    ID_EX ID_EX_0 (
        .clock(clock),

        // TODO 3
        // debug ports
        .ID_instruction(ID_instruction),
        .EX_instruction(EX_instruction),

        .ID_pc_plus_four(ID_pc_plus_four),
        .EX_pc_plus_four(EX_pc_plus_four),
        .RegReadDataA(RegReadDataA),
        .EX_RegReadDataA(EX_RegReadDataA),
        .RegReadDataB(RegReadDataB),
        .EX_RegReadDataB(EX_RegReadDataB),
        .extended(extended),
        .EX_extended(EX_extended),

        // TODO 1
        // we probably should, this isn't readable at all.
        .RegWrite(BufferedControl[8]),
        .EX_RegWrite(EX_RegWrite),
        .RegDst(BufferedControl[7]),
        .EX_RegDst(EX_RegDst),
        .MemRead(BufferedControl[6]),
        .EX_MemRead(EX_MemRead),
        .MemWrite(BufferedControl[5]),
        .EX_MemWrite(EX_MemWrite),
        .MemToReg(BufferedControl[4]),
        .EX_MemToReg(EX_MemToReg),
        .Branch(BufferedControl[3]),
        .EX_Branch(EX_Branch),
        .ALUSrc(BufferedControl[2]),
        .EX_ALUSrc(EX_ALUSrc),
        .ALUOp(BufferedControl[1:0]),
        .EX_ALUOp(EX_ALUOp),

        .rs(rs),
        .EX_rs(EX_rs),
        .rt(rt),
        .EX_rt(EX_rt),
        .rd(rd),
        .EX_rd(EX_rd)
    );

    wire [1:0] ForwardA;
    wire [31:0] ALUArgA;

    mux4to1 #(
        .WIDTH(32)
    ) MuxForwardA (
        .inA(EX_RegReadDataA),
        .inB(RegWriteData),
        .inC(MEM_ALUResult),
        .inD(32'b0),
        .select(ForwardA),
        .out(ALUArgA)
    );

    wire [1:0] ForwardB;
    wire [31:0] ForwardBOut;

    mux4to1 #(
        .WIDTH(32)
    ) MuxForwardB (
        .inA(EX_RegReadDataB),
        .inB(RegWriteData),
        .inC(MEM_ALUResult),
        .inD(32'b0),
        .select(ForwardB),
        .out(ForwardBOut)
    );

    wire [31:0] ALUArgB;

    mux2to1 #(
        .WIDTH(32)
    ) MuxALUSrc (
        .inA(ForwardBOut),
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
        .inA(ALUArgA),
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

    // TODO 3
    // debug wire
    wire [31:0] MEM_instruction;

    wire [31:0] MEM_branch_address;
    wire MEM_Zero;
    wire [31:0] MEM_ALUResult;
    wire [31:0] MEM_RegReadDataB;
    wire [4:0] MEM_RegWriteAddress;
    wire MEM_RegWrite;
    wire MEM_MemRead;
    wire MEM_MemWrite;
    wire MEM_MemToReg;
    wire MEM_Branch;
    wire MEM_bneOne;

    Forwarding Forwarding_0 (
        .EX_rs(EX_rs),
        .EX_rt(EX_rt),
        .MEM_rd(MEM_rd),
        .WB_rd(WB_rd),
        .MEM_RegWrite(MEM_RegWrite),
        .WB_RegWrite(WB_RegWrite),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB)
    );

    // EX/MEM pipeline registers (3rd)
    EX_MEM EX_MEM_0 (
        .clock(clock),

        // TODO 3
        // debug ports
        .EX_instruction(EX_instruction),
        .MEM_instruction(MEM_instruction),

        .branch_address(branch_address),
        .MEM_branch_address(MEM_branch_address),
        .Zero(Zero),
        .MEM_Zero(MEM_Zero),
        .ALUResult(ALUResult),
        .MEM_ALUResult(MEM_ALUResult),
        .EX_RegReadDataB(EX_RegReadDataB),
        .MEM_RegReadDataB(MEM_RegReadDataB),
        .RegWriteAddress(RegWriteAddress),
        .MEM_RegWriteAddress(MEM_RegWriteAddress),

        .EX_RegWrite(EX_RegWrite),
        .MEM_RegWrite(MEM_RegWrite),
        .EX_MemRead(EX_MemRead),
        .MEM_MemRead(MEM_MemRead),
        .EX_MemWrite(EX_MemWrite),
        .MEM_MemWrite(MEM_MemWrite),
        .EX_MemToReg(EX_MemToReg),
        .MEM_MemToReg(MEM_MemToReg),
        .EX_Branch(EX_Branch),
        .MEM_Branch(MEM_Branch),
        .bneOne(bneOne),
        .MEM_bneOne(MEM_bneOne)
    );

    wire pc_chooser;

    // TODO
    // select can't be don't care (x). we may need to write a specific module
    // to always output 0 or 1.
    //mux2to1 #(
    //    .WIDTH(1)
    //) MuxBeqBne (
    //    .inA(MEM_Zero),
    //    .inB(~MEM_Zero),
    //    .select(MEM_bneOne),
    //    .out(pc_chooser)
    //);
    assign pc_chooser = 0;

    mux2to1 #(
        .WIDTH(32)
    ) MuxPCNext (
        .inA(pc_plus_four),
        .inB(MEM_branch_address),
        .select(MEM_Branch && pc_chooser),
        .out(pc_next)
    );

    wire [31:0] MemReadData;

    Memory #(
        .SIZE(DATA_MEM_SIZE)
    ) DataMemory_0 (
        .clock(clock),
        .Address(MEM_ALUResult),
        .ReadEnable(MEM_MemRead),
        .ReadData(MemReadData),
        .WriteEnable(MEM_MemWrite),
        .WriteData(MEM_RegReadDataB)
    );

    // TODO
    // probably superfluous
    wire [4:0] MEM_rd;
    assign MEM_rd = MEM_RegWriteAddress;

    // TODO 3
    // debug wire
    wire [31:0] WB_instruction;

    wire [31:0] WB_MemReadData;
    wire [31:0] WB_ALUResult;
    wire [4:0] WB_RegWriteAddress;

    wire WB_RegWrite;
    wire WB_MemToReg;

    // MEM/WB pipeline registers (4th)
    MEM_WB MEM_WB_0 (
        .clock(clock),

        // TODO 3
        // debug ports
        .MEM_instruction(MEM_instruction),
        .WB_instruction(WB_instruction),

        .MemReadData(MemReadData),
        .WB_MemReadData(WB_MemReadData),
        .MEM_ALUResult(MEM_ALUResult),
        .WB_ALUResult(WB_ALUResult),
        .MEM_RegWriteAddress(MEM_RegWriteAddress),
        .WB_RegWriteAddress(WB_RegWriteAddress),

        .MEM_RegWrite(MEM_RegWrite),
        .WB_RegWrite(WB_RegWrite),
        .MEM_MemToReg(MEM_MemToReg),
        .WB_MemToReg(WB_MemToReg)
    );

    mux2to1 #(
        .WIDTH(32)
    ) MuxMemtoReg (
        .inA(WB_ALUResult),
        .inB(WB_MemReadData),
        .select(WB_MemToReg),
        .out(RegWriteData)
    );

    // TODO
    // probably superfluous
    wire [4:0] WB_rd;
    assign WB_rd = WB_RegWriteAddress;
endmodule
////////////////////////////////////////////////////////////////////////////////
