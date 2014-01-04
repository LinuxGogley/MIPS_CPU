.text

.globl main

main:
    # li $zero, 0  # $zero is read-only
    # li $at, 1
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

    add $t0, $t0, $s0
    sub $t1, $t1, $a0
    or $t7, $t7, $t5
    and $s1, $s0, $s2
    slt $s2, $s1, $t7

exit:
    #li $v0, 10
    #syscall
