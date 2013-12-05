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
    // program counter

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
    // program counter incrementer

    always @(negedge clock, negedge reset) begin
        if (reset == 0)
            pc_new = 0;
        else
            pc_new = pc + 4;
    end  // always
endmodule

// TODO
// will be used initially as DataMemory
module Memory (addr, ren, dout, wen, din);
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

module InstructionMemory (addr, dout);
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


    reg [31:0] data[4095:0];

    always @(addr) begin
        if (addr[31:12] != 0)
            $display("\ninstruction memory WARNING (time %0d): address unused MSBs are not zero\n", $time);
        dout = data[addr[11:0]];
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
        // TODO
        // check
        //if ((reset != 0) && (wen == 1))
        if (reset != 0)
            if (wen == 1)
                data[wa] = wd;
endmodule

module SignExtender (immediate, extended);
    input wire [15:0] immediate;
    output reg [31:0] extended;
    // sign extender

    always @(immediate) begin
        // http://stackoverflow.com/questions/4176556/how-to-sign-extend-a-number-in-verilog
        // TODO
        // test both implementations
        extended[31:0] = {{16{immediate[15]}}, immediate[15:0]};
        //extended = $signed(immediate);
    end  // always
endmodule

module ALU (op, inA, inB, out, zero);
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

    parameter N = 32;

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
    input wire [5:0] Opcode;
    output reg [1:0] ALUOp;
    output wire RegWrite;
    // obsoleted by Control module
    //
    // opcodes
    // -------
    //  0 : 6'b00_00_00 : R-format instruction
    //  4 : 6'b00_01_00 : beq : branch on equal
    // 35 : 6'b10_00_11 : lw : load word
    // 43 : 6'b10_10_11 : sw : store word

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

module Control (Opcode, RegDst, Branch, MemRead, MemtoReg, ALUOp, MemWrite,
        ALUSrc, RegWrite);
    input wire [5:0] Opcode;
    output wire RegDst;
    output wire Branch;
    output wire MemRead;
    output wire MemtoReg;
    output reg [1:0] ALUOp;
    output wire MemWrite;
    output wire ALUSrc;
    output wire RegWrite;
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
        // the opcodes could have been represented using the constants
        case(Opcode)
            //  0 : 6'b00_00_00 : R-format instruction
             0 : ALUOp = 2'b10;

            //  4 : 6'b00_01_00 : beq : branch on equal
             4 : ALUOp = 2'b01;

            //  5 : 6'b00_01_01 : bne : branch on not equal
             5 : ALUOp = 2'b01;

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

module ALUControl (Funct, ALUOp, ALUControl);
    input wire [5:0] Funct;
    input wire [1:0] ALUOp;
    output reg [3:0] ALUControl;
    // ALU control unit

    always @ (Funct, ALUOp) begin
        // lw, sw
        if (ALUOp == 0) begin
            // add : addition
            ALUControl = 4'b0010;
        end

        // beq, bne
        if (ALUOp == 1) begin
            // sub : subtraction
            ALUControl = 4'b0110;
        end

        // R-format instructions
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
