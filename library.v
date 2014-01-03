// mips_module_library.v
////////////////////////////////////////////////////////////////////////////////

// a collection of modules that are used to implement a MIPS processor
////////////////////////////////////////////////////////////////////////////////

// Dimitrios Paraschas (paraschas@gmail.com)
////////////////////////////////////////////////////////////////////////////////

// inf.uth.gr
// ce232 Computer Organization and Design
////////////////////////////////////////////////////////////////////////////////

// lab 8
// Implementation of a collection of MIPS instructions parsing CPU.
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

// modules
////////////////////////////////////////////////////////////////////////////////
module ALU (op, inA, inB, out, zero);
    parameter N = 32;
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
            default: out = 0;
            // TODO
            // is this better?
            //default: out = 'bx;
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

                    // TODO
                    // what should I put here until I implement all Funct codes?
                    default: ALUCtrl = 4'b1111;
                endcase

            // TODO
            // what should I put here until I implement all instructions?
            default : ALUCtrl = 4'b1111;
        endcase
    end  // always
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
                    ALUSrc = 1'b1;
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
                    ALUSrc = 1'b1;
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

            // 43 : 6'b10_10_11 : sw : store word
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

module InstructionMemory (addr, dout);
    parameter N = 1024;
    input wire [31:0] addr;
    output reg [31:0] dout;
    // instruction memory
    //
    // active 1024 words, from 12 address LSBs
    //
    // read-only
    //
    // read
    // ----
    // address addr, data dout

    reg [31:0] data[N - 1:0];

    always @(addr) begin
        if (addr[31:12] != 0)
            $display("\ninstruction memory WARNING (time %0d): address unused MSBs not zero\n", $time);
        dout = data[addr[11:0]];
    end  // always
endmodule

// TODO
// test
//module mux2to1 #(parameter N = 1) (inA, inB, select, out);
module mux2to1 (inA, inB, select, out);
    parameter N = 1;
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

module Memory (addr, ren, dout, wen, din);
    parameter N = 4096;
    input wire [31:0] addr;
    input wire ren;
    output wire [31:0] dout;
    input wire wen;
    input wire [31:0] din;
    // memory
    //
    // active 1024 words, from 12 address LSBs
    //
    // read
    // ----
    // enable ren, address addr, data dout
    //
    // write
    // -----
    // enable wen, address addr, data din

    reg [31:0] data[N - 1:0];

    always @(ren, wen)
        if (ren && wen)
            $display ("\nmemory ERROR (time %0d): ren and wen both active!\n", $time);

    always @(posedge ren, posedge wen)
        if (addr[31:12] != 0)
            $display("\nmemory WARNING (time %0d): unused address MSBs not zero\n", $time);

    assign dout = ((wen == 1'b0) && (ren == 1'b1)) ? data[addr[11:0]] : 32'bx;

    always @(din, wen, ren, addr)
        if ((wen == 1'b1) && (ren == 1'b0))
            data[addr[11:0]] = din;
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

module PCPlus4 (clock, reset, pc, pc_four);
    input wire clock;
    input wire reset;
    input wire [31:0] pc;
    output reg [31:0] pc_four;
    // program counter incrementer

    always @(negedge clock, negedge reset) begin
        if (reset == 0)
            pc_four = 0;
        else
            pc_four = pc + 4;
    end  // always
endmodule

module Registers (clock, reset, raA, rdA, raB, rdB, wen, wa, wd);
    input wire clock;
    input wire reset;
    input wire [4:0] raA;
    output reg [31:0] rdA;
    input wire [4:0] raB;
    output reg [31:0] rdB;
    input wire wen;
    input wire [4:0] wa;
    input wire [31:0] wd;
    // registers
    //
    // read ports
    // ----------
    // address raA, data rdA
    // address raB, data rdB
    //
    // write ports
    // -----------
    // enable wen, address wa, data wd

    reg [31:0] data[0:31];
    integer k;

    always @(raA)
        rdA = data[raA];

    always @(raB)
        rdB = data[raB];

    always @(negedge reset)
        for (k = 0; k < 32; k = k + 1)
            data[k] = 0;

    always @(negedge clock)
        if ((reset != 0) && (wen == 1))
            data[wa] = wd;
endmodule

module SignExtender (immediate, extended);
    input wire [15:0] immediate;
    output reg [31:0] extended;
    // sign extender

    always @(immediate) begin
        // TODO
        // test both implementations
        extended[31:0] = {{16{immediate[15]}}, immediate[15:0]};
        //extended = $signed(immediate);
    end  // always
endmodule
////////////////////////////////////////////////////////////////////////////////
