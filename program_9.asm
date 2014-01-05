.text

.globl main

main:
    #li $zero, 0  # $zero is read-only
    li $at, 1  # reserved for assembler
    li $v0, 2
    li $v1, 3
    li $a0, 4
    li $a1, 5
    li $a2, 6
    li $a3, 7
    li $t0, 8
    li $t1, 9
    li $t2, 10
    li $t3, 11
    li $t4, 12
    li $t5, 13
    li $t6, 14
    li $t7, 15
    li $s0, 16
    li $s1, 17
    li $s2, 18
    li $s3, 19
    li $s4, 20
    li $s5, 21
    li $s6, 22
    li $s7, 23
    li $t8, 24
    li $t9, 25
    li $k0, 26  # reserved for OS kernel
    li $k1, 27  # reserved for OS kernel
    li $gp, 28
    li $sp, 29
    li $fp, 30
    li $ra, 31

    # an offset of 0x10010000 is required
    add $t0, $t0, $s0
    sw $ra, 0x10010002($t2)
    lw $t5, 0x10010002($t2)
    sub $t1, $t1, $a0
    or $t6, $t7, $t5
    and $s3, $s0, $s2
    lw $t6, 0x10010002($t2)
    sw $gp, 0x10010008($zero)
    lw $v0, 0x10010008($zero)
    and $a0, $v0, $t5
    or $a0, $a0, $t2
    add $t1, $a0, $v0
    slt $sp, $a0, $t1

exit:
    #li $v0, 10
    #syscall
