// CPU.v
////////////////////////////////////////////////////////////////////////////////

// connect the DataPath and the ControlUnit to create the CPU
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

module CPU (clock, reset);
    input wire clock;
    input wire reset;

    wire [5:0] Opcode;
    wire [5:0] Funct;
    wire [3:0] ALUControl;
    wire RegWrite;

    // ControlUnit (Opcode, Funct, ALUControl, RegWrite);
    ControlUnit ControlUnit_0 (Opcode, Funct, ALUControl, RegWrite);

    // DataPath (ALUControl, RegWrite, Opcode, Funct);
    DataPath DataPath_0 (ALUControl, RegWrite, Opcode, Funct);
endmodule
