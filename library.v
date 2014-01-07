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
module ALU #(parameter N = 32) (op, inA, inB, out, zero);
    input wire [3:0] op;
    input wire [N - 1:0] inA;
    input wire [N - 1:0] inB;
    output reg [N - 1:0] out;
    output wire zero;
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

module ALUControl (Funct, ALUOp, ALUCtrl);
    input wire [5:0] Funct;
    input wire [1:0] ALUOp;
    output reg [3:0] ALUCtrl;
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

module BranchAdder (pc_plus_four, extended_times_four, branch_address);
    input wire [31:0] pc_plus_four;
    input wire [31:0] extended_times_four;
    output wire [31:0] branch_address;
    // branch address adder

    assign branch_address = pc_plus_four + extended_times_four;
endmodule

module Control (Opcode, RegWrite, RegDst, MemRead, MemWrite, MemToReg, Branch,
        ALUSrc, ALUOp);
    input wire [5:0] Opcode;
    output reg RegWrite;
    output reg RegDst;
    output reg MemRead;
    output reg MemWrite;
    output reg MemToReg;
    output reg Branch;
    output reg ALUSrc;
    output reg [1:0] ALUOp;
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

module IF_ID (clock, pc_plus_four, IF_ID_pc_plus_four, instruction,
        IF_ID_instruction);
    input wire clock;
    input wire [31:0] pc_plus_four;
    output reg [31:0] IF_ID_pc_plus_four;
    input wire [31:0] instruction;
    output reg [31:0] IF_ID_instruction;
    // IF/ID pipeline registers (1st pipeline registers)
    //
    // 64 bits wide

    always @(negedge clock) begin
        IF_ID_pc_plus_four = pc_plus_four;
        IF_ID_instruction = instruction;
    end  // always
endmodule

module InstructionMemory #(parameter N = 1024) (Address, Instruction);
    input wire [31:0] Address;
    output reg [31:0] Instruction;
    // instruction memory
    //
    // active 1024 words, from 12 address LSBs
    //
    // read-only

    reg [31:0] data[N - 1:0];

    always @(Address) begin
        if (Address[31:12] != 0)
            $display("\ninstruction memory WARNING (time %0d): address unused MSBs not zero\n", $time);
        Instruction = data[Address[11:0]];
    end  // always
endmodule

module mux2to1 #(parameter N = 1) (inA, inB, select, out);
    input wire [N - 1:0] inA;
    input wire [N - 1:0] inB;
    input wire select;
    output wire [N - 1:0] out;
    // 2 to 1 multiplexer
    //
    // inA, inB : inputs
    // select : select
    // out : output
    //
    // N : input/output port width

    assign out = ~select ? inA : inB;
endmodule

module Memory #(parameter N = 4096) (clock, Address, ReadEnable, ReadData,
        WriteEnable, WriteData);
    input wire clock;
    input wire [31:0] Address;
    input wire ReadEnable;
    output wire [31:0] ReadData;
    input wire WriteEnable;
    input wire [31:0] WriteData;
    // memory
    //
    // active 1024 words, from 12 address LSBs

    reg [31:0] data[N - 1:0];

    always @(ReadEnable, WriteEnable)
        if (ReadEnable && WriteEnable)
            $display ("\nmemory ERROR (time %0d): ReadEnable and WriteEnable both active!\n", $time);

    always @(posedge ReadEnable, posedge WriteEnable)
        if (Address[31:12] != 0)
            $display("\nmemory WARNING (time %0d): unused address MSBs not zero\n", $time);

    // TODO
    // test this always block and replace the following assign statement with it
    //always @(Address, ReadEnable, WriteEnable)
    //    if ((ReadEnable == 1'b1) && (WriteEnable == 1'b0))
    //        ReadData = data[Address[11:0]];
    //    else
    //        ReadData = 32'bx;

    assign ReadData = ((WriteEnable == 1'b0) && (ReadEnable == 1'b1)) ? data[Address[11:0]] : 32'bx;

    always @(negedge clock)
        if ((ReadEnable == 1'b0) && (WriteEnable == 1'b1))
            data[Address[11:0]] = WriteData;
endmodule

module ProgramCounter (clock, reset, pc_next, pc);
    input wire clock;
    input wire reset;
    input wire [31:0] pc_next;
    output reg [31:0] pc;
    // program counter

    always @(posedge clock, negedge reset) begin
        if (reset == 0)
            pc = 0;
        else
            pc = pc_next;
    end  // always
endmodule

module PCPlus4 (pc, pc_plus_four);
    input wire [31:0] pc;
    output wire [31:0] pc_plus_four;
    // program counter incrementer

    assign pc_plus_four = pc + 4;
endmodule

module Registers (clock, reset, ReadAddressA, ReadDataA, ReadAddressB,
        ReadDataB, WriteEnable, WriteAddress, WriteData);
    input wire clock;
    input wire reset;
    input wire [4:0] ReadAddressA;
    output reg [31:0] ReadDataA;
    input wire [4:0] ReadAddressB;
    output reg [31:0] ReadDataB;
    input wire WriteEnable;
    input wire [4:0] WriteAddress;
    input wire [31:0] WriteData;
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
        if ((reset != 0) && (WriteEnable == 1))
            data[WriteAddress] = WriteData;
endmodule

module SignExtender (immediate, extended);
    input wire [15:0] immediate;
    output reg [31:0] extended;
    // sign extender

    always @(immediate) begin
        extended[31:0] = {{16{immediate[15]}}, immediate[15:0]};
        // NOTE
        // this also seems to work. I'm not sure which one is to be preferred.
        //extended = $signed(immediate);
    end  // always
endmodule
////////////////////////////////////////////////////////////////////////////////
