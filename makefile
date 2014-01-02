CC = iverilog
FLAGS = -Wall -Winfloop

7: library.v CPU.v testbench_7.v
	$(CC) $(FLAGS) library.v CPU.v testbench_7.v -o testbench_7

	masmbin -c -f translated_instructions.txt program_7.masm program_7.mbin

	vvp testbench

	gtkwave dumpfile.vcd config_7.gtkw &
