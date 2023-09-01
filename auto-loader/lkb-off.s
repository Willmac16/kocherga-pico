.syntax unified
.cpu cortex-m0plus
.thumb

.text

/*
    This launches the Offset XIP version of autoloader
*/

.global auto_launch
.thumb_func
auto_launch:
    push {r0-r7, lr} // Save the regs from C into the stack

    ldr r0, =(0x10040000 + 0x100) // place the addr to the vectors section of the binary into reg naught
    ldr r1, =(0xE0000000) // place the addr of the vtor into reg one
    ldr r2, =(0x0000ed08)
    str r0, [r1, r2] // Set the VTOR to the vectors section

    ldmia r0, {r0, r1} // load into reg naught and one from addr in reg naught
    msr msp, r0 // load reg naught into stack pointer

    bx r1 // branch to addr in reg one

    pop {r0-r7, pc}
