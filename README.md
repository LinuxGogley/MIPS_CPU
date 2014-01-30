MIPS CPU in Verilog
===
32 bit MIPS CPU with a five stages pipeline in Verilog

install Icarus Verilog and GTKWave
```sh
sudo apt-get install iverilog gtkwave
```

run testbench and show waveforms for programs 7, 8, 9
```sh
make 7

make 8

make 9
```

supported instructions
---

**R-format instructions**

add

sub

and

or

slt

**I-format instructions**

sw

lw

beq

bne

author
---
Dimitrios Paraschas (paraschas@gmail.com)

Created for the Computer Organization and Design (ce232) course of the Department of Electrical and Computer Engineering of University of Thessaly.

http://www.inf.uth.gr/

license
---
The MIT License, see LICENSE.md.
