.syntax unified
.cpu cortex-m0plus
.thumb

.text

.global launch_kocherga_bin
.thumb_func
launch_kocherga_bin:
    push {r0-r7, lr} // Save the regs from C into the stack

    bkpt 0xF0

    ldr r0, =(0x10040000 + 0x040) // place the addr to the beginning of the binary into r naught
    ldr r1, =(0xE000E000 + 0x0000ed08) // load the addr of the vtor into r one
    str r0, [r1] // store the contents of 0x040 into the vector table in ram

    ldmia r0, {r0, r1} // load into reg naught and one from addr in reg naught
    msr msp, r0 // load reg naught into stack pointer

    bx r1 // branch to addr in reg one

    pop {r0-r7, pc}
