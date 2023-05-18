.data
    n: .word 10
    
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
      j main
rec:
    addi sp, sp, -8
    sw x1, 4(sp)
    sw a0, 0(sp)    # push n in stack
    
    addi t1, a0, 0  # t1 = n
    addi t2, x0, 2  # t2 = 2
    bge t1, t2, L1  # if (n >= 2), go to L1
    
    addi a0, x0, 5  # else, set return value to 5
    addi t0, a0, 0
    addi sp, sp, 8  # pop stack (no restore)
    jalr x0, 0(x1)
    
L1:
    srli a0, a0, 1  # n = n/2
    jal x1, rec     # call T(n/2) 
    addi t1, a0, 0  # move result of T(n/2) to t1
    lw x1, 4(sp)    # restore caller's return address
    lw a0, 0(sp)    # restore caller's n
    slli t2, t1, 2  # t2 = 4T(n/2)
    slli t3, a0, 1  # t3 = 2n
    add t2, t2, t3  # t2 = 4T(n/2) + 2n
    addi a0, t2, 7  # a0 = 4T(n/2) + 2n + 7
    addi sp, sp, 8
    jalr x0, 0(x1)
    
main:
    jal x1, rec
    addi t0, a0, 0

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall