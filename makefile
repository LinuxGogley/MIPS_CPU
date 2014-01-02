CC = iverilog
FLAGS = -Wall -Winfloop

test_7: dumpfile_7.vcd
	gtkwave dumpfile_7.vcd config_7.gtkw &

dumpfile_7.vcd: testbench_7 program_7.mbin
	vvp testbench_7

testbench_7: library.v CPU.v testbench_7.v
	$(CC) $(FLAGS) library.v CPU.v testbench_7.v -o testbench_7

program_7.mbin: program_7.masm
	masmbin -c -f translated_instructions_7.txt program_7.masm program_7.mbin

.PHONY: clean
clean: clean_7

clean_7:
	rm -f program_7.mbin translated_instructions_7.txt testbench_7 dumpfile_7.vcd
