# makefile
# Dimitrios Paraschas (paraschas@gmail.com)

# convert the assembly program to its binary representation, synthesize the
# design, generate the dumpfile, and load GTKWave using the saved configuration.

CC = iverilog
FLAGS = -Wall -Winfloop

default: test_8

test_7: dumpfile_7.vcd
	gtkwave dumpfile_7.vcd config_7.gtkw &

dumpfile_7.vcd: testbench_7 program_7.mbin
	vvp testbench_7

testbench_7: library.v CPU.v testbench_7.v
	$(CC) $(FLAGS) library.v CPU.v testbench_7.v -o testbench_7

program_7.mbin: program_7.masm
	masmbin -c -f filter_7.txt program_7.masm program_7.mbin

test_8: dumpfile_8.vcd
	gtkwave dumpfile_8.vcd config_8.gtkw &

dumpfile_8.vcd: testbench_8 program_8.mbin
	vvp testbench_8

testbench_8: library.v CPU.v testbench_8.v
	$(CC) $(FLAGS) library.v CPU.v testbench_8.v -o testbench_8

program_8.mbin: program_8.masm
	masmbin -c -f filter_8.txt program_8.masm program_8.mbin

.PHONY: clean
clean: clean_7 clean_8

clean_7:
	rm -f testbench_7 dumpfile_7.vcd

clean_8:
	rm -f testbench_8 dumpfile_8.vcd
