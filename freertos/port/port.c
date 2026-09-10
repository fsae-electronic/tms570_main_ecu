/*
 * Minimal FreeRTOS port for TMS570LS1224 (Cortex-R4F, big-endian), GCC.
 * No MPU wrappers -- every task runs privileged in System mode.
 *
 * OS tick: RTI counter block 1 / compare-2 (VIM channel 4), NOT the more
 * "obvious" block 0 / compare-0 (channel 2). This matches the proven,
 * working configuration in ../adc2can_front's firmware (a different board
 * on this same TMS570 family, confirmed working in the field), which
 * starts rtiCOUNTER_BLOCK1 and drives its periodic tasks off COMPARE2/3.
 * Block 0 / compare-0 (the "default" choice HALCoGen and most reference
 * examples reach for) never delivered a single IRQ on this board despite
 * every software-visible register looking correctly configured -- see the
 * extensive debugging history for what was ruled out. Register offsets
 * below are the RTI1 peripheral's register frame (see include/reg_rti.h).
 */

#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
#include "FreeRTOS.h"
#include "os_task.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#define portRTI_GCTRL_REG        ( *( ( volatile uint32_t * ) 0xFFFFFC00 ) )
#define portRTI_TBCTRL_REG       ( *( ( volatile uint32_t * ) 0xFFFFFC04 ) )
#define portRTI_COMPCTRL_REG     ( *( ( volatile uint32_t * ) 0xFFFFFC0C ) )
#define portRTI_CNT1_FRC1_REG    ( *( ( volatile uint32_t * ) 0xFFFFFC30 ) )
#define portRTI_CNT1_UC1_REG     ( *( ( volatile uint32_t * ) 0xFFFFFC34 ) )
#define portRTI_CNT1_CPUC1_REG   ( *( ( volatile uint32_t * ) 0xFFFFFC38 ) )
#define portRTI_CNT2_COMP2_REG   ( *( ( volatile uint32_t * ) 0xFFFFFC60 ) )
#define portRTI_CNT2_UDCP2_REG   ( *( ( volatile uint32_t * ) 0xFFFFFC64 ) )
#define portRTI_SETINTENA_REG    ( *( ( volatile uint32_t * ) 0xFFFFFC80 ) )
#define portRTI_CLEARINTENA_REG  ( *( ( volatile uint32_t * ) 0xFFFFFC84 ) )
#define portRTI_INTFLAG_REG      ( *( ( volatile uint32_t * ) 0xFFFFFC88 ) )

/* Constants for the initial task stack frame. */
#define portINITIAL_SPSR        ( ( StackType_t ) 0x1F ) /* System mode, IRQ/FIQ enabled. */
#define portINSTRUCTION_SIZE    ( ( StackType_t ) 0x04 )
#define portTHUMB_MODE_BIT      ( ( StackType_t ) 0x20 )

/* Number of stack words reserved between the saved top-of-stack and R0
 * (R12..R1), matching the layout portRESTORE_CONTEXT expects. */
#define portSPACE_BETWEEN_TOS_AND_PARAMETERS ( 12 )

extern uint32_t _getCPSRValue_( void );
extern void vPortStartFirstTask( void );

/* Critical section nesting depth -- 0 once the first task starts. */
uint32_t ulCriticalNesting = 0xFFFFFFFF;

/* Set to 1 while a task is using the FPU, so the context switch knows
 * whether to save/restore D0-D15/FPSCR for it. */
uint32_t ulTaskHasFPUContext = 0;

/*-----------------------------------------------------------*/

StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
    StackType_t *pxOriginalTOS = pxTopOfStack;
    int i;

    /* This layout must match, field for field, what portRESTORE_CONTEXT in
     * portASM.s pops (in the opposite order) -- FPU context is saved/
     * restored unconditionally there, so the initial frame needs all of
     * it, not just a single "does this task use FPU" flag. */

    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pxCode + portINSTRUCTION_SIZE; /* return address, offset to look like an IRQ-entered PC */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) 0x00000000; /* R14 */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pxOriginalTOS; /* R13 -- SP the task starts with */
    pxTopOfStack--;
    pxTopOfStack -= portSPACE_BETWEEN_TOS_AND_PARAMETERS; /* R12..R1, left as whatever's already on the (zeroed) stack */

    *pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
    pxTopOfStack--;

    *pxTopOfStack = ( StackType_t ) ( ( _getCPSRValue_() & ~0xFFUL ) | portINITIAL_SPSR );
    if( ( ( uint32_t ) pxCode & 0x01UL ) != 0x00UL )
    {
        *pxTopOfStack |= portTHUMB_MODE_BIT;
    }
    pxTopOfStack--;

    /* D0-D15 (16 double registers = 32 words), zeroed. */
    for( i = 0; i < 32; i++ )
    {
        *pxTopOfStack = 0;
        pxTopOfStack--;
    }

    /* FPSCR -- the last word popped by portRESTORE_CONTEXT is the first
     * one here, at the final pxTopOfStack. */
    *pxTopOfStack = 0;

    return pxTopOfStack;
}

/*-----------------------------------------------------------*/

static void prvSetupTimerInterrupt( void )
{
    portRTI_GCTRL_REG &= 0xFFFFFFFCUL; /* stop both counter blocks while reconfiguring */
    portRTI_TBCTRL_REG = 0x00000000U;
    portRTI_COMPCTRL_REG = 0x00000100U; /* COMPSEL2 uses RTIFRC1 (block 1) */

    portRTI_CNT1_UC1_REG = 0x00000000U;
    portRTI_CNT1_FRC1_REG = 0x00000000U;
    portRTI_CNT1_CPUC1_REG = 0x00000001U;

    portRTI_CNT2_COMP2_REG = ( configCPU_CLOCK_HZ / 2 ) / configTICK_RATE_HZ;
    portRTI_CNT2_UDCP2_REG = ( configCPU_CLOCK_HZ / 2 ) / configTICK_RATE_HZ;

    portRTI_INTFLAG_REG = 0x0007000FU;
    portRTI_CLEARINTENA_REG = 0x00070F0FU;

    portRTI_SETINTENA_REG = 0x00000004U; /* enable compare-2 interrupt */
    portRTI_GCTRL_REG |= 0x00000002U;    /* start counter block 1 */
}

/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
    prvSetupTimerInterrupt();

    ulCriticalNesting = 0;

    vPortStartFirstTask(); /* never returns */

    return pdFAIL;
}

/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not supported on this target. */
    for( ;; )
    {
    }
}
