.syntax unified
.cpu cortex-m0plus
.thumb

.text

/*
    This Launches the copy to ram version of autoloader
*/

.global auto_launch
.thumb_func
auto_launch:
    push {r0-r7, lr} // Save the regs from C into the stack

    ldr r0, =(0x10000000 + 0x100) // place the addr to the beginning of the binary into r naught
    ldr r1, =(0xE0000000 + 0x0000ed08) // load the addr of the vtor into reg one
    str r0, [r1] // store the contents of 0x100 into the vector table in ram

    ldmia r0, {r0, r1} // load into reg naught and one from addr in reg naught
    msr msp, r0 // load reg naught into stack pointer

    bx r1 // branch to addr in reg one

    pop {r0-r7, pc}
