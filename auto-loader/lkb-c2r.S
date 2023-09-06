.syntax unified
.cpu cortex-m0plus
.thumb

#include "auto-loader.hpp"
#include "pico/asm_helper.S"
#include "hardware/regs/m0plus.h"

.text

/*
    This is a modified version of exit_from_boot2
    that Launches the copy to ram version of autoloader
*/

.global auto_launch
.thumb_func
auto_launch:
    push {r0-r7, lr} // Save the regs from C into the stack

    ldr r0, =(XIP_BASE + 0x100) // place the addr to the vectors section of the binary into reg naught
    ldr r1, =(PPB_BASE + M0PLUS_VTOR_OFFSET) // place the addr of the vtor into reg one
    str r0, [r1] // Set the VTOR to the vectors section


    ldmia r0, {r0, r1} // load into reg naught and one from addr in reg naught
    msr msp, r0 // load reg naught into stack pointer

    bx r1 // branch to addr in reg one

    pop {r0-r7, pc}
