`include "constants.h"

`timescale 1ns/1ps

module ALU (out, zero, inA, inB, op);
    // MIPS Arithmetic and Logic Unit
    //
    // opcodes
    // -------
    //  0 : bitwise AND      : out = inA & inB
    //  1 : bitwise OR       : out = inA | inB
    //  2 : addition         : out = inA + inB
    //  6 : subtraction      : out = inA - inB
    //  7 : set on less than : out = ((inA < inB) ? 1 : 0)
    // 12 : bitwise NOR      : out = ~(inA | inB)

    parameter N = 8;
    output reg [N - 1:0] out;
    output reg zero;
    input [N - 1:0] inA, inB;
    input [3:0] op;

    always @(op) begin
        case(op)
            0000: out = inA & inB;
            0001: out = inA | inB;
            0010: out = inA + inB;
            0110: out = inA - inB;
            0111: out = ((inA < inB) ? 1 : 0);
            1100: out = ~(inA | inB);
            default: out = 'bx;
        endcase

        // TODO
        // check this:
        //zero = (out == 0);
        zero = ((out == 0) ? 1 : 0);
    end  // always
endmodule

module Memory (ren, wen, addr, din, dout);
    // Memory (active 1024 words, from 10 address lsbs).
    // Read : enable ren, address addr, data dout
    // Write: enable wen, address addr, data din.
    input ren, wen;
    input [31:0] addr, din;
    output [31:0] dout;

    reg [31:0] data[4095:0];
    wire [31:0] dout;

    always @(ren, wen)
        if (ren & wen)
            $display ("\nMemory ERROR (time %0d): ren and wen both active!\n", $time);

    always @(posedge ren, posedge wen) begin
        if (addr[31:10] != 0)
            $display("Memory WARNING (time %0d): address msbs are not zero\n", $time);
    end

    assign dout = ((wen == 1'b0) && (ren == 1'b1)) ? data[addr[9:0]] : 32'bx;

    always @(din, wen, ren, addr) begin
        if ((wen == 1'b1) && (ren == 1'b0))
            data[addr[9:0]] = din;
    end
endmodule

module RegFile (clock, reset, raA, raB, wa, wen, wd, rdA, rdB);
    // register file
    //
    // read ports
    // ----------
    // address raA, data rdA
    // address raB, data rdB
    //
    // write ports
    // -----------
    // address wa, data wd, enable wen

    input clock, reset;
    input [4:0] raA, raB, wa;
    input wen;
    input [31:0] wd;
    output reg [31:0] rdA, rdB;
    reg [31:0] registers[0:31];
    integer k;

    always @(raA)
        rdA = registers[raA];

    always @(raB)
        rdB = registers[raB];

    always @(negedge reset)
        for (k = 0; k < 32; k = k + 1)
            registers[k] = 0;

    always @(negedge clock)
        if (reset != 0)
            if (wen == 1)
                registers[wa] = wd;
endmodule
