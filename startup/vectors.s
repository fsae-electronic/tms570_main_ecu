/*
 * Exception vector table for TMS570LS1224. Must sit at address 0x00000000
 * (see linker/link.ld).
 *
 * SVC and IRQ route directly to this port's handlers (vPortSVCHandler,
 * vPortTickISR) -- no jump-table indirection, since there's exactly one
 * use for each: SVC is always an explicit task yield, IRQ is always the
 * RTI tick. That removes the whole class of bug the previous port hit
 * (an interrupt-table entry at the wrong array index).
 */

        .syntax unified
        .cpu cortex-r4
        .arm

        .section .intvecs, "a", %progbits
        .type resetEntry, %object
        .size resetEntry, .-resetEntry

        .extern _c_int00
        .extern vPortSVCHandler
        .extern _dabort
        .weak resetEntry

resetEntry:
        B       _c_int00
undefEntry:
        B       undefEntry              @ undefined instruction: no recovery, spin so it's visible under a debugger
svcEntry:
        B       vPortSVCHandler
prefetchEntry:
        B       prefetchEntry           @ prefetch abort: no recovery, spin so it's visible under a debugger
        B       _dabort
        B       undefEntry              @ reserved vector slot -- never actually taken by hardware
        LDR     PC, [PC, #-0x1B0]       @ IRQ: fetch handler address from VIM's IRQVECREG
        LDR     PC, [PC, #-0x1B0]       @ FIQ: same trick (unused -- no channel is FIQ-routed)
