CC = iverilog
FLAGS = -Wall -Winfloop

synthesize: library.v CPU.v testbench.v
	$(CC) $(FLAGS) library.v CPU.v testbench.v -o testbench

	masmbin -c -f translated_instructions.txt program.masm program.mbin

	vvp testbench

	gtkwave dumpfile.vcd config.gtkw &
