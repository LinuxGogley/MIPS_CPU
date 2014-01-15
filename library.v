// library.v
////////////////////////////////////////////////////////////////////////////////

// modules used to implement a MIPS five stages pipeline CPU
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
module ALU #(
    parameter WIDTH = 32
    ) (
    input wire [3:0] op,
    input wire [WIDTH - 1:0] inA,
    input wire [WIDTH - 1:0] inB,
    output reg [WIDTH - 1:0] out,
    output wire zero
    );
    // Arithmetic and Logic Unit
    //
    // opcodes
    // -------
    //  0 : 4'b0000 : bitwise AND      : out = inA & inB
    //  1 : 4'b0001 : bitwise OR       : out = inA | inB
    //  2 : 4'b0010 : addition         : out = inA + inB
    //  6 : 4'b0110 : subtraction      : out = inA - inB
    //  7 : 4'b0111 : set on less than : out = (inA < inB) ? 1 : 0
    // 12 : 4'b1100 : bitwise NOR      : out = ~(inA | inB)

    always @(op, inA, inB) begin
        case(op)
             0 : out = inA & inB;
             1 : out = inA | inB;
             2 : out = inA + inB;
             6 : out = inA - inB;
             7 : out = (inA < inB) ? 1 : 0;
            12 : out = ~(inA | inB);
            default: out = 32'bx;
        endcase
    end  // always

    assign zero = (out == 0);
endmodule

module ALUControl (
    input wire [5:0] Funct,
    input wire [1:0] ALUOp,
    output reg [3:0] ALUCtrl
    );
    // ALU control unit

    always @ (Funct, ALUOp) begin
        case(ALUOp)
            // lw, sw
            // add : addition
            0 : ALUCtrl = 4'b0010;

            // beq, bne
            // sub : subtraction
            1 : ALUCtrl = 4'b0110;

            // R-format instructions
            2 : case(Funct)
                    // 32 : 6'b10_00_00 : add : addition
                    32 : ALUCtrl = 4'b0010;

                    // 34 : 6'b10_00_10 : sub : subtraction
                    34 : ALUCtrl = 4'b0110;

                    // 36 : 6'b10_01_00 : and : logical and
                    36 : ALUCtrl = 4'b0000;

                    // 37 : 6'b10_01_01 : or : logical or
                    37 : ALUCtrl = 4'b0001;

                    // 42 : 6'b10_10_10 : slt : set on less than
                    42 : ALUCtrl = 4'b0111;

                    default: ALUCtrl = 4'bx;
                endcase

            default: ALUCtrl = 4'bx;
        endcase
    end  // always
endmodule

module BranchAdder (
    input wire [31:0] pc_plus_four,
    input wire [31:0] extended_times_four,
    output wire [31:0] branch_address
    );
    // branch address adder

    assign branch_address = pc_plus_four + extended_times_four;
endmodule

module Control (
    input wire [5:0] Opcode,
    output reg RegWrite,
    output reg RegDst,
    output reg MemRead,
    output reg MemWrite,
    output reg MemToReg,
    output reg Branch,
    output reg ALUSrc,
    output reg [1:0] ALUOp
    );
    // opcode decoder
    //
    // opcodes
    // -------
    //  0 : 6'b00_00_00 : R-format instruction
    //  4 : 6'b00_01_00 : beq : branch on equal
    //  5 : 6'b00_01_01 : bne : branch on not equal
    // 35 : 6'b10_00_11 : lw : load word
    // 43 : 6'b10_10_11 : sw : store word

    always @ (Opcode) begin
        // NOTE
        // the opcodes could have been represented using the constants
        case(Opcode)
            //  0 : 6'b00_00_00 : R-format instruction
             0 : begin
                    RegDst = 1'b1;
                    Branch = 1'b0;
                    MemRead = 1'b0;
                    MemToReg = 1'b0;
                    ALUOp = 2'b10;
                    MemWrite = 1'b0;
                    ALUSrc = 1'b0;
                    RegWrite = 1'b1;
            end

            //  4 : 6'b00_01_00 : beq : branch on equal
             4 : begin
                    RegDst = 1'b0;
                    Branch = 1'b1;
                    MemRead = 1'b0;
                    MemToReg = 1'b0;
                    ALUOp = 2'b01;
                    MemWrite = 1'b0;
                    ALUSrc = 1'b0;
                    RegWrite = 1'b0;
            end

            //  5 : 6'b00_01_01 : bne : branch on not equal
             5 : begin
                    RegDst = 1'b0;
                    Branch = 1'b1;
                    MemRead = 1'b0;
                    MemToReg = 1'b0;
                    ALUOp = 2'b11;
                    MemWrite = 1'b0;
                    ALUSrc = 1'b0;
                    RegWrite = 1'b0;
            end

            // 35 : 6'b10_00_11 : lw : load word
            35 : begin
                    RegDst = 1'b0;
                    Branch = 1'b0;
                    MemRead = 1'b1;
                    MemToReg = 1'b1;
                    ALUOp = 2'b00;
                    MemWrite = 1'b0;
                    ALUSrc = 1'b1;
                    RegWrite = 1'b1;
            end

            // 43 : 6'b10_10_11 : sw : store word
            43 : begin
                    RegDst = 1'b0;
                    Branch = 1'b0;
                    MemRead = 1'b0;
                    MemToReg = 1'b0;
                    ALUOp = 2'b00;
                    MemWrite = 1'b1;
                    ALUSrc = 1'b1;
                    RegWrite = 1'b0;
            end

            // default
            default : begin
                    RegDst = 1'b0;
                    Branch = 1'b0;
                    MemRead = 1'b0;
                    MemToReg = 1'b0;
                    ALUOp = 2'b11;
                    MemWrite = 1'b0;
                    ALUSrc = 1'b0;
                    RegWrite = 1'b0;
            end
        endcase
    end  // always
endmodule

module InstructionMemory #(
    parameter SIZE = 1024
    ) (
    input wire [31:0] Address,
    output reg [31:0] Instruction
    );
    // instruction memory
    //
    // active 1024 words, from 12 address LSBs
    //
    // read-only

    reg [31:0] data[SIZE - 1:0];

    always @(Address) begin
        if (Address[31:12] != 0) begin
            $display("\nInstructionMemory WARNING (time %0d):", $time);
            $display("unused address MSBs not zero\n");
        end  // if
        Instruction = data[Address[11:0]];
    end  // always
endmodule

module mux2to1 #(
    parameter WIDTH = 1
    ) (
    input wire [WIDTH - 1:0] inA,
    input wire [WIDTH - 1:0] inB,
    input wire select,
    output wire [WIDTH - 1:0] out
    );
    // 2 to 1 multiplexer
    //
    // inA, inB : inputs
    // select : select
    // out : output
    //
    // WIDTH : input/output port width

    assign out = ~select ? inA : inB;
endmodule

module mux4to1 #(
    parameter WIDTH = 1
    ) (
    input wire [WIDTH - 1:0] inA,
    input wire [WIDTH - 1:0] inB,
    input wire [WIDTH - 1:0] inC,
    input wire [WIDTH - 1:0] inD,
    input wire [1:0] select,
    output reg [WIDTH - 1:0] out
    );
    // 4 to 1 multiplexer

    always @(inA, inB, inC, inD, select) begin
        case(select)
             0 : out = inA;
             1 : out = inB;
             2 : out = inC;
             3 : out = inD;
        endcase
    end  // always
endmodule

module Memory #(
    parameter SIZE = 4096
    ) (
    input wire clock,
    input wire [31:0] Address,
    input wire ReadEnable,
    output wire [31:0] ReadData,
    input wire WriteEnable,
    input wire [31:0] WriteData
    );
    // memory
    //
    // active 1024 words, from 12 address LSBs

    reg [31:0] data[SIZE - 1:0];

    always @(ReadEnable, WriteEnable)
        if (ReadEnable && WriteEnable) begin
            $display ("\nDataMemory ERROR (time %0d):", $time);
            $display ("ReadEnable and WriteEnable both active\n");
        end  // if

    always @(posedge ReadEnable, posedge WriteEnable)
        if (Address[31:12] != 0) begin
            $display("\nDataMemory WARNING (time %0d):", $time);
            $display("unused address MSBs not zero\n");
        end  // if

    // TODO
    // test this always block and replace the following assign statement with it
    //always @(Address, ReadEnable, WriteEnable)
    //    if ((ReadEnable == 1'b1) && (WriteEnable == 1'b0))
    //        ReadData = data[Address[11:0]];
    //    else
    //        ReadData = 32'bx;

    assign ReadData = ((WriteEnable == 1'b0) && (ReadEnable == 1'b1)) ? data[Address[11:0]] : 32'bx;

    always @(negedge clock)
        if ((ReadEnable == 1'b0) && (WriteEnable == 1'b1)) begin
            data[Address[11:0]] <= WriteData;

            $display("DataMemory:");
            $display("\twrote data %2d to address %2d at time %3d\n",
                WriteData, Address[11:0], $time);
        end  // if
endmodule

module ProgramCounter (
    input wire clock,
    input wire reset,
    input wire [31:0] pc_next,
    output reg [31:0] pc
    );
    // program counter

    always @(posedge clock, negedge reset) begin
        if (reset == 0)
            pc = 0;
        else
            pc = pc_next;
    end  // always
endmodule

module PCPlus4 (
    input wire [31:0] pc,
    output wire [31:0] pc_plus_four
    );
    // program counter incrementer

    assign pc_plus_four = pc + 4;
endmodule

module Registers (
    input wire clock,
    input wire reset,
    input wire [4:0] ReadAddressA,
    output reg [31:0] ReadDataA,
    input wire [4:0] ReadAddressB,
    output reg [31:0] ReadDataB,
    input wire WriteEnable,
    input wire [4:0] WriteAddress,
    input wire [31:0] WriteData
    );
    // registers

    reg [31:0] data[0:31];
    integer k;

    // NOTE
    // the assign statement in this case feels that describes the hardware
    // closer to its real operation. the always statement seems artificial.
    // the always implementation was kept because it makes the waveforms
    // more readable.
    //output wire [31:0] ReadDataA;
    //output wire [31:0] ReadDataB;
    //assign ReadDataA = data[ReadAddressA];
    //assign ReadDataB = data[ReadAddressB];

    always @(posedge clock, ReadAddressA, ReadAddressB) begin
        ReadDataA = data[ReadAddressA];
        ReadDataB = data[ReadAddressB];
    end  // always

    always @(negedge reset)
        for (k = 0; k < 32; k = k + 1)
            data[k] = 0;

    always @(negedge clock)
        if ((reset != 0) && (WriteEnable == 1)) begin
            data[WriteAddress] <= WriteData;
            $display("Registers:");
            $display("\twrote data %2d to register %2d at time %3d\n",
                WriteData, WriteAddress, $time);
        end  // if
endmodule

module SignExtender (
    input wire [15:0] immediate,
    output reg [31:0] extended
    );
    // sign extender

    always @(immediate) begin
        extended[31:0] = {{16{immediate[15]}}, immediate[15:0]};
        // NOTE
        // this also seems to work. I'm not sure which one is to be preferred.
        //extended = $signed(immediate);
    end  // always
endmodule

module IF_ID (
    input wire clock,
    input wire [31:0] pc_plus_four,
    output reg [31:0] ID_pc_plus_four,
    input wire [31:0] instruction,
    output reg [31:0] ID_instruction
    );
    // IF/ID pipeline registers (1st)

    always @(negedge clock) begin
        ID_pc_plus_four <= pc_plus_four;
        ID_instruction <= instruction;
    end  // always
endmodule

module ID_EX (
    input wire clock,

    // TODO
    // debug ports
    input wire [31:0] ID_instruction,
    output reg [31:0] EX_instruction,

    input wire [31:0] ID_pc_plus_four,
    output reg [31:0] EX_pc_plus_four,
    input wire [31:0] RegReadDataA,
    output reg [31:0] EX_RegReadDataA,
    input wire [31:0] RegReadDataB,
    output reg [31:0] EX_RegReadDataB,
    input wire [31:0] extended,
    output reg [31:0] EX_extended,

    input wire RegWrite,
    output reg EX_RegWrite,
    input wire RegDst,
    output reg EX_RegDst,
    input wire MemRead,
    output reg EX_MemRead,
    input wire MemWrite,
    output reg EX_MemWrite,
    input wire MemToReg,
    output reg EX_MemToReg,
    input wire Branch,
    output reg EX_Branch,
    input wire ALUSrc,
    output reg EX_ALUSrc,
    input wire [1:0] ALUOp,
    output reg [1:0] EX_ALUOp,

    input wire [4:0] rs,
    output reg [4:0] EX_rs,
    input wire [4:0] rt,
    output reg [4:0] EX_rt,
    input wire [4:0] rd,
    output reg [4:0] EX_rd
    );
    // ID/EX pipeline registers (2nd)

    always @(negedge clock) begin
        // TODO
        // debug ports
        EX_instruction <= ID_instruction;

        EX_pc_plus_four <= ID_pc_plus_four;
        EX_RegReadDataA <= RegReadDataA;
        EX_RegReadDataB <= RegReadDataB;
        EX_extended <= extended;

        EX_RegWrite <= RegWrite;
        EX_RegDst <= RegDst;
        EX_MemRead <= MemRead;
        EX_MemWrite <= MemWrite;
        EX_MemToReg <= MemToReg;
        EX_Branch <= Branch;
        EX_ALUSrc <= ALUSrc;
        EX_ALUOp <= ALUOp;

        EX_rs <= rs;
        EX_rt <= rt;
        EX_rd <= rd;
    end  // always
endmodule

module EX_MEM (
    input wire clock,

    // TODO
    // debug ports
    input wire [31:0] EX_instruction,
    output reg [31:0] MEM_instruction,

    input wire [31:0] branch_address,
    output reg [31:0] MEM_branch_address,
    input wire Zero,
    output reg MEM_Zero,
    input wire [31:0] ALUResult,
    output reg [31:0] MEM_ALUResult,
    input wire [31:0] EX_RegReadDataB,
    output reg [31:0] MEM_RegReadDataB,
    input wire [4:0] RegWriteAddress,
    output reg [4:0] MEM_RegWriteAddress,

    input wire EX_RegWrite,
    output reg MEM_RegWrite,
    input wire EX_MemRead,
    output reg MEM_MemRead,
    input wire EX_MemWrite,
    output reg MEM_MemWrite,
    input wire EX_MemToReg,
    output reg MEM_MemToReg,
    input wire EX_Branch,
    output reg MEM_Branch,
    input wire bneOne,
    output reg MEM_bneOne
    );
    // EX/MEM pipeline registers (3rd)

    always @(negedge clock) begin
        // TODO
        // debug ports
        MEM_instruction <= EX_instruction;

        MEM_branch_address <= branch_address;
        MEM_Zero <= Zero;
        MEM_ALUResult <= ALUResult;
        MEM_RegReadDataB <= EX_RegReadDataB;
        MEM_RegWriteAddress <= RegWriteAddress;

        MEM_RegWrite <= EX_RegWrite;
        MEM_MemRead <= EX_MemRead;
        MEM_MemWrite <= EX_MemWrite;
        MEM_MemToReg <= EX_MemToReg;
        MEM_Branch <= EX_Branch;
        MEM_bneOne <= bneOne;
    end  // always
endmodule

module MEM_WB (
    input wire clock,

    // TODO
    // debug ports
    input wire [31:0] MEM_instruction,
    output reg [31:0] WB_instruction,

    input wire [31:0] MemReadData,
    output reg [31:0] WB_MemReadData,
    input wire [31:0] MEM_ALUResult,
    output reg [31:0] WB_ALUResult,
    input wire [4:0] MEM_RegWriteAddress,
    output reg [4:0] WB_RegWriteAddress,

    input wire MEM_RegWrite,
    output reg WB_RegWrite,
    input wire MEM_MemToReg,
    output reg WB_MemToReg
    );
    // MEM/WB pipeline registers (4th)

    always @(negedge clock) begin
        // TODO
        // debug ports
        WB_instruction <= MEM_instruction;

        WB_MemReadData <= MemReadData;
        WB_ALUResult <= MEM_ALUResult;
        WB_RegWriteAddress <= MEM_RegWriteAddress;

        WB_RegWrite <= MEM_RegWrite;
        WB_MemToReg <= MEM_MemToReg;
    end  // always
endmodule

module Forwarding (
    input wire [4:0] EX_rs,
    input wire [4:0] EX_rt,
    input wire [4:0] MEM_rd,
    input wire [4:0] WB_rd,
    input wire MEM_RegWrite,
    input wire WB_RegWrite,
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB
    );
    // forwarding unit

    always @*
        // if (MEM/WB.RegWrite == 1 and
        //     MEM/WB.RegisterRd != 0 and
        //     MEM/WB.RegisterRd == ID/EX.RegisterRs and
        //     ((EX/MEM.RegisterRd != ID/EX.RegisterRs) or (EX.MEM.RegWrite == 0)))
        //     then ForwardA = 1
        if ((WB_RegWrite == 1) &&
            (WB_rd != 0) &&
            (WB_rd == EX_rs) &&
            ((MEM_rd != EX_rs) || (MEM_RegWrite == 0))) begin
            ForwardA = 2'b01;
        end

        // if (EX/MEM.RegWrite == 1 and
        //     EX/MEM.RegisterRd != 0 and
        //     EX/MEM.RegisterRd == ID/EX.RegisterRs)
        //     then ForwardA = 2
        else if ((MEM_RegWrite == 1) &&
            (MEM_rd != 0) &&
            (MEM_rd == EX_rs))
            ForwardA = 2'b10;
        else
            ForwardA = 2'b00;

    always @*
        // if (MEM/WB.RegWrite == 1 and
        //     MEM/WB.RegisterRd != 0 and
        //     MEM/WB.RegisterRd == ID/EX.RegisterRt and
        //     ((EX/MEM.RegisterRd != ID/EX.RegisterRt) or (EX.MEM.RegWrite == 0)))
        //     then ForwardB = 1
        if ((WB_RegWrite == 1) &&
            (WB_rd != 0) &&
            (WB_rd == EX_rt) &&
            ((MEM_rd != EX_rt) || (MEM_RegWrite == 0)))
            ForwardB = 2'b01;

        // if (EX/MEM.RegWrite == 1 and
        //     EX/MEM.RegisterRd != 0 and
        //     EX/MEM.RegisterRd == ID/EX.RegisterRt)
        //     then ForwardB = 2
        else if ((MEM_RegWrite == 1) &&
            (MEM_rd != 0) &&
            (MEM_rd == EX_rt))
            ForwardB = 2'b10;
        else
            ForwardB = 2'b00;

endmodule
////////////////////////////////////////////////////////////////////////////////
