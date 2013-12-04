// mips_module_library.v
////////////////////////////////////////////////////////////////////////////////

// a collection of modules that are used to implement a MIPS processor
////////////////////////////////////////////////////////////////////////////////

// Dimitrios Paraschas (paraschas@gmail.com)
// Panagiotis Kremmydas (kmd178@gmail.com)
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
module ProgramCounter (clock, reset, pc_new, pc);
    input wire clock;
    input wire reset;
    input wire [31:0] pc_new;
    output reg [31:0] pc;

    always @(posedge clock, negedge reset) begin
        if (reset == 0)
            pc = 0;
        else
            pc = pc_new;
    end  // always
endmodule

module PCPlus4 (clock, reset, pc, pc_new);
    input wire clock;
    input wire reset;
    input wire [31:0] pc;
    output reg [31:0] pc_new;

    always @(negedge clock, negedge reset) begin
        if (reset == 0)
            pc_new = 0;
        else
            pc_new = pc + 4;
    end  // always
endmodule

// TODO
// will be used initially as DataMemory
module Memory (ren, wen, addr, din, dout);
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

    input wire ren, wen;
    input wire [31:0] addr, din;
    output wire [31:0] dout;

    reg [31:0] data[4095:0];

    always @(ren, wen)
        if (ren && wen)
            $display ("\nmemory ERROR (time %0d): ren and wen both active!\n", $time);

    always @(posedge ren, posedge wen)
        if (addr[31:12] != 0)
            $display("\nmemory WARNING (time %0d): unused address MSBs are not zero\n", $time);

    assign dout = ((wen == 1'b0) && (ren == 1'b1)) ? data[addr[11:0]] : 32'bx;

    always @(din, wen, ren, addr)
        if ((wen == 1'b1) && (ren == 1'b0))
            data[addr[11:0]] = din;
endmodule

module Registers (clock, reset, raA, raB, wa, wen, wd, rdA, rdB);
    // registers
    //
    // read ports
    // ----------
    // address raA, data rdA
    // address raB, data rdB
    //
    // write ports
    // -----------
    // address wa, data wd, enable wen

    input wire clock, reset;
    input wire [4:0] raA, raB, wa;
    input wire wen;
    input wire [31:0] wd;
    output reg [31:0] rdA, rdB;
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
        // TODO
        // check
        //if ((reset != 0) && (wen == 1))
        if (reset != 0)
            if (wen == 1)
                data[wa] = wd;
endmodule

module ALU (out, zero, inA, inB, op);
    // MIPS Arithmetic and Logic Unit
    //
    // opcodes
    // -------
    //  0 : 4'b0000 : bitwise AND      : out = inA & inB
    //  1 : 4'b0001 : bitwise OR       : out = inA | inB
    //  2 : 4'b0010 : addition         : out = inA + inB
    //  6 : 4'b0110 : subtraction      : out = inA - inB
    //  7 : 4'b0111 : set on less than : out = (inA < inB) ? 1 : 0
    // 12 : 4'b1100 : bitwise NOR      : out = ~(inA | inB)

    parameter N = 32;

    output reg [N - 1:0] out;
    output wire zero;

    input wire [N - 1:0] inA, inB;
    input wire [3:0] op;

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

module MainDecoder (Opcode, ALUOp, RegWrite);
    // opcodes
    // -------
    //  0 : 6'b00_00_00 : R-format instruction
    //  4 : 6'b00_01_00 : beq : branch on equal
    // 35 : 6'b10_00_11 : lw : load word
    // 43 : 6'b10_10_11 : sw : store word

    input wire [5:0] Opcode;
    output reg [1:0] ALUOp;
    output wire RegWrite;

    always @ (Opcode) begin
        // the opcodes could have been represented using the constants
        case(Opcode)
            //  0 : 6'b00_00_00 : R-format instruction
             0 : ALUOp = 2'b10;

            //  4 : 6'b00_01_00 : beq : branch on equal
             4 : ALUOp = 2'b01;

            // 35 : 6'b10_00_11 : lw : load word
            35 : ALUOp = 2'b00;

            // 43 : 6'b10_10_11 : sw : store word
            43 : ALUOp = 2'b00;

            // TODO
            // what should I put here until I implement all opcodes?
            default: ALUOp = 2'b11;
        endcase
    end  // always

    assign RegWrite = 1;
endmodule

module ALUDecoder (Funct, ALUOp, ALUControl);
    input wire [5:0] Funct;
    input wire [1:0] ALUOp;
    output reg [3:0] ALUControl;

    always @ (Funct, ALUOp) begin
        if (ALUOp == 2) begin
            case(Funct)
                // 32 : 6'b10_00_00 : add : addition
                32 : ALUControl = 4'b0010;

                // 34 : 6'b10_00_10 : sub : subtraction
                34 : ALUControl = 4'b0110;

                // 36 : 6'b10_01_00 : and : logical and
                36 : ALUControl = 4'b0000;

                // 37 : 6'b10_01_01 : or : logical or
                37 : ALUControl = 4'b0001;

                // 42 : 6'b10_10_10 : slt : set on less than
                42 : ALUControl = 4'b0111;

                // TODO
                // what should I put here until I implement all Funct codes?
                default: ALUControl = 4'b1111;
            endcase
        end  // if
        else begin
            // TODO
            // what should I put here until I implement instructions of formats
            // other than R?
            ALUControl = 4'b1111;
        end
    end  // always
endmodule
////////////////////////////////////////////////////////////////////////////////
