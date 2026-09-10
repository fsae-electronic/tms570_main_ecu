/*
 * Cortex-R4F context switch, SVC yield, and tick ISR for TMS570LS1224.
 * GCC/GNU-as, big-endian. No MPU wrappers -- every task is privileged.
 *
 * Design notes (why this looks the way it does, after a previous port hit
 * real bugs here):
 *
 *  - Critical sections (vPortEnterCritical/vPortExitCritical/
 *    vPortDisableInterrupts/vPortEnableInterrupts) execute CPSID/CPSIE
 *    directly. No SVC bounce. ARM's SVC exception entry does NOT itself
 *    mask IRQ (only IRQ/FIQ/Reset entry do) -- routing "disable interrupts"
 *    through an SVC leaves a real window, between taking the SVC and the
 *    handler's own SPSR fixup taking effect, where a tick IRQ can preempt
 *    while already inside SVC-mode exception handling. That's a genuine
 *    nested exception this port's IRQ-mode-stack-as-scratch-space context
 *    save can't survive. Since tasks are always privileged here, CPSID/
 *    CPSIE are valid wherever they're called from and take effect
 *    atomically as part of instruction execution -- no window at all.
 *
 *  - portYIELD is still SVC-based: SVC is synchronous and always taken
 *    regardless of the caller's I-bit, and the vector is a fixed, direct
 *    branch (no VIM channel table lookup) -- one less place to get a
 *    channel number wrong.
 *
 *  - FPU (D0-D15/FPSCR) context is saved/restored unconditionally for
 *    every task, rather than the previous port's opt-in-per-task
 *    (vPortTaskUsesFPU via yet another SVC) scheme. A few extra cycles per
 *    switch buys one less stateful mechanism to get wrong; this app isn't
 *    hard-real-time at the microsecond level.
 */

        .section .text
        .syntax unified
        .cpu cortex-r4
        .fpu vfpv3-d16
        .arm

        .global vTaskSwitchContext
        .global xTaskIncrementTick
        .global pxCurrentTCB
        .global ulCriticalNesting

@-------------------------------------------------------------------------
@ Save the context of the task currently running on entry to an IRQ or the
@ SVC-based yield. Must be entered in a privileged, non-User/System mode
@ (IRQ or SVC) with the interrupted task's registers still live.
.macro portSAVE_CONTEXT
        DSB

        STMDB   SP!, {R0}
        SUB     SP, SP, #4
        STMDA   SP, {SP}^              @ store the task's own (banked) SP into [SP]
        LDMIA   SP!, {R0}              @ R0 = task's SP; restore our own SP
        STMDB   R0!, {LR}              @ push return address onto the task's stack
        MOV     LR, R0
        LDMIA   SP!, {R0}              @ restore the R0 we pushed first
        STMDB   LR, {R0-LR}^           @ push R0-R14 (banked) onto the task's stack
        SUB     LR, LR, #60            @ LR -> base of that 15-word block
        MRS     R0, SPSR
        STMDB   LR!, {R0}
        VSTMDB  LR!, {D0-D15}
        VMRS    R0, FPSCR
        STMDB   LR!, {R0}
        LDR     R0, pxCurrentTCBConst
        LDR     R0, [R0]
        STR     LR, [R0]
.endm

@-------------------------------------------------------------------------
@ Restore the context of whichever task pxCurrentTCB now points to and
@ return to it. Must end the exception (IRQ or SVC) that called it.
.macro portRESTORE_CONTEXT
        LDR     R0, pxCurrentTCBConst
        LDR     R0, [R0]
        LDR     LR, [R0]

        LDMIA   LR!, {R0}
        VMSR    FPSCR, R0
        VLDMIA  LR!, {D0-D15}

        LDMIA   LR!, {R0}
        MSR     SPSR_cxsf, R0

        LDMIA   LR, {R0-R14}^
        LDR     LR, [LR, #+60]
        DSB
        SUBS    PC, LR, #4
.endm

@-------------------------------------------------------------------------
        .global vPortStartFirstTask
        .type   vPortStartFirstTask, %function
vPortStartFirstTask:
        CPSID   I
        CPS     #0x13
        portRESTORE_CONTEXT
        .size   vPortStartFirstTask, .-vPortStartFirstTask

@-------------------------------------------------------------------------
@ SVC vector target -- the only SVC use in this port is an explicit yield.
        .global vPortSVCHandler
        .type   vPortSVCHandler, %function
vPortSVCHandler:
        CPSID   I                       @ SVC entry does NOT auto-mask IRQ (only IRQ/FIQ/
                                         @ Reset entry do) -- without this, a tick IRQ can
                                         @ preempt mid-yield and collide with the IRQ-mode-
                                         @ stack-as-scratch-space trick portSAVE_CONTEXT/
                                         @ portRESTORE_CONTEXT both rely on. Safe to mask
                                         @ here unconditionally: portRESTORE_CONTEXT ends by
                                         @ restoring the target task's own SPSR, which always
                                         @ has IRQ enabled for a normally-running task.
        ADD     LR, LR, #4              @ SVC's own LR is already correct; compensate
                                         @ for portRESTORE_CONTEXT's unconditional -4.
        portSAVE_CONTEXT
        BL      vTaskSwitchContext
        portRESTORE_CONTEXT
        .size   vPortSVCHandler, .-vPortSVCHandler

@-------------------------------------------------------------------------
@ IRQ vector target -- the only IRQ this application uses is the RTI
@ block-1/compare-2 OS tick (VIM channel 4, see startup/vim.c and the
@ comment in freertos/port/port.c on why block 1 and not block 0).
        .global vPortTickISR
        .type   vPortTickISR, %function
vPortTickISR:
        portSAVE_CONTEXT

        LDR     R0, =0xFFFFFC88         @ RTI INTFLAG
        MOV     R1, #4                  @ compare-2 flag (bit 2)
        STR     R1, [R0]                @ clear the compare-0 flag

        BL      xTaskIncrementTick
        CMP     R0, #0
        BLNE    vTaskSwitchContext

        portRESTORE_CONTEXT
        .size   vPortTickISR, .-vPortTickISR

@-------------------------------------------------------------------------
@ Catch-all for any VIM channel this application doesn't assign a real
@ handler to. Deliberately does NOT spin forever (a C for(;;) loop here
@ previously did, on the theory that a spurious/misrouted interrupt should
@ hang visibly under a debugger rather than silently misbehave) -- that
@ turned a one-off spurious vector into a permanent hang whenever the
@ hardware's vectored IRQ fetch landed here instead of the real handler,
@ which happened on every single real IRQ during bring-up regardless of
@ which channel/peripheral was supposed to fire. A plain exception return
@ at least lets the system keep running so whatever's actually happening
@ can be observed, instead of masking it behind a dead stop.
        .global phantomInterrupt
        .type   phantomInterrupt, %function
phantomInterrupt:
        SUBS    PC, LR, #4
        .size   phantomInterrupt, .-phantomInterrupt

@-------------------------------------------------------------------------
        .global vPortYield
        .type   vPortYield, %function
vPortYield:
        SVC     #0
        BX      LR
        .size   vPortYield, .-vPortYield

@-------------------------------------------------------------------------
@ Direct (non-SVC) critical section / interrupt mask primitives -- valid
@ because every task runs privileged. See file header for why this is
@ safe here but wasn't with the previous SVC-based scheme.
        .global vPortEnterCritical
        .type   vPortEnterCritical, %function
vPortEnterCritical:
        CPSID   I
        LDR     R0, ulCriticalNestingConst
        LDR     R1, [R0]
        ADD     R1, R1, #1
        STR     R1, [R0]
        BX      LR
        .size   vPortEnterCritical, .-vPortEnterCritical

        .global vPortExitCritical
        .type   vPortExitCritical, %function
vPortExitCritical:
        LDR     R0, ulCriticalNestingConst
        LDR     R1, [R0]
        CMP     R1, #0
        BXEQ    LR
        SUBS    R1, R1, #1
        STR     R1, [R0]
        BXNE    LR
        CPSIE   I
        BX      LR
        .size   vPortExitCritical, .-vPortExitCritical

        .global vPortDisableInterrupts
        .type   vPortDisableInterrupts, %function
vPortDisableInterrupts:
        CPSID   I
        BX      LR
        .size   vPortDisableInterrupts, .-vPortDisableInterrupts

        .global vPortEnableInterrupts
        .type   vPortEnableInterrupts, %function
vPortEnableInterrupts:
        CPSIE   I
        BX      LR
        .size   vPortEnableInterrupts, .-vPortEnableInterrupts

@-------------------------------------------------------------------------
        .global ulPortCountLeadingZeros
        .type   ulPortCountLeadingZeros, %function
ulPortCountLeadingZeros:
        CLZ     R0, R0
        BX      LR
        .size   ulPortCountLeadingZeros, .-ulPortCountLeadingZeros

@-------------------------------------------------------------------------
pxCurrentTCBConst:      .word   pxCurrentTCB
ulCriticalNestingConst: .word   ulCriticalNesting
