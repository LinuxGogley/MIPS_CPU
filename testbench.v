// testbench.v
////////////////////////////////////////////////////////////////////////////////

// testbench for the CPU
////////////////////////////////////////////////////////////////////////////////

// Dimitrios Paraschas (paraschas@gmail.com)
////////////////////////////////////////////////////////////////////////////////

// inf.uth.gr
// ce232 Computer Organization and Design
////////////////////////////////////////////////////////////////////////////////

// lab 7
// Implementation of a collection of a MIPS R-format instructions parsing CPU.
////////////////////////////////////////////////////////////////////////////////

`include "constants.h"

`timescale 1ns/1ps

module cpu_tb;
    reg clock, reset;  // clock and reset signals
    integer i;

    // CPU with name cpu0 instantiation
    // CPU (clock, reset);
    CPU cpu0 (clock, reset);

    // clock and reset signal generation
    initial begin
        clock = 0;
        reset = 1;
    end

    always begin
        #5;  // that translates to a period of 10 units of time
        clock = ~clock;
    end

    // Initialize Register File with initial values.
    // cpu0 is the name of the cpu instance
    // cpu_regs is the name of the register file instance in the CPU verilog file
    // data[] is the register file array
    initial begin
        for (i = 0; i < 32; i = i + 1)
            cpu0.DataPath_0.cpu_regs.data[i] = i;  // Note that R0 = 0 in MIPS

        // Initialize Data Memory. You have to develop "program.hex" as a text file
        // which contains the instruction opcodes as 32-bit hexadecimal values.
        //$readmemh("program.hex", cpu0.cpu_IMem.data);
        $readmemh("program.hex", cpu0.DataPath_0.Memory_0.data);

        // monitor the outputs

        $finish;
    end  // initial
endmodule

// Edw, to "program.hex" einai ena arxeio pou prepei na brisketai sto
// directory pou trexete th Verilog kai na einai ths morfhs:

// @0    00000000
// @4    20100009
// @8    00000000
// @C    00000000
// ...

// H aristerh sthlh, meta to @, exei th dieythynsh ths mnhmhs (hex),
// kai h deksia sthlh ta dedomena sth dieythynsh ayth (pali hex).
// Sto paradeigma pio panw, oi lekseis stis dieythynseis 0, 8 kai 12
// einai 0, kai sth dieythynsh 4 exei thn timh 32'h20100009. An o PC
// diabasei thn dieythynsh 4, h timh ekei exei thn entolh
//   addi $16 <- $0 + 9

// To deytero orisma ths $readmemh einai pou akribws brisketai h mnhmh
// pou tha arxikopoihthei. Sto paradeigma, to "dat0" einai to onoma pou
// dwsame sto instance tou datapath. To "mem" einai to onoma pou exei
// to instance ths mnhmhs MESA sto datapath, kai to "data" einai to
// onoma pou exei to pragmatiko array ths mhnhs mesa sto module ths.
// An exete dwsei diaforetika onomata, allakste thn $readmemh.

// Enallaktika, an sas boleyei perissotero, yparxei h entolh $readmemb
// me thn akribws idia syntaksh. H aristerh sthlh tou arxeiou exei
// thn idia morfh (dieythynseis se hex), alla h deksia sthlh exei
// ta dedomena sto dyadiko. Etsi h add mporouse na einai:

// @4    00100000000100000000000000001001

// ... h kai akoma kalytera:

// @4    001000_00000_10000_0000000000001001

// (h Verilog epitrepei diaxwristika underscores).
