CC = iverilog
FLAGS = -Wall -Winfloop

synthesize: library.v CPU.v testbench.v
	$(CC) $(FLAGS) library.v CPU.v testbench.v -o testbench

	masmbin -c -f translated_instructions.txt program_7.masm program_7.mbin

	vvp testbench

	gtkwave dumpfile.vcd config.gtkw &
