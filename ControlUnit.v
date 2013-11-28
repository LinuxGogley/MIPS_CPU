// ControlUnit.v
////////////////////////////////////////////////////////////////////////////////

// connect the MainDecoder and the ALUDecoder to create the ControlUnit
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

module ControlUnit (Opcode, Funct, ALUControl, RegWrite);
    input wire [5:0] Opcode;
    input wire [5:0] Funct;
    output wire [3:0] ALUControl;
    output wire RegWrite;

    wire [1:0] ALUOp;

    // MainDecoder (Opcode, ALUOp, RegWrite);
    MainDecoder MainDecoder_0 (Opcode, ALUOp, RegWrite);

    // ALUDecoder (Funct, ALUOp, ALUControl);
    ALUDecoder ALUDecoder_0 (Funct, ALUOp, ALUControl);
endmodule
