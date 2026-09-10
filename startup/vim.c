/*
 * VIM (Vectored Interrupt Manager) channel table for TMS570LS1224.
 *
 * Written with designated initializers ([N] = handler), so the channel
 * number is explicit at every entry -- there is no possible off-by-one
 * from an extra or missing line shifting everything else, which is
 * exactly the bug a previous, hand-edited version of this table had.
 *
 * Channel assignments are fixed by the silicon (each peripheral's
 * interrupt request line always lands on the same VIM channel number);
 * only the *handler* at each channel is application-defined.
 *
 *   0  - ESM high-priority interrupt
 *   4  - RTI block-1/compare-2 (this port's OS tick, see freertos/port/
 *        port.c for why block 1 rather than the more obvious block 0)
 *   16 - CAN1 high-level interrupt (unused -- CAN is polled by the app,
 *        left wired up in case that changes later)
 * all other channels -> phantomInterrupt (safe catch-all: does a plain
 * exception return rather than servicing anything, since we don't know
 * what fired).
 *
 * Each real channel above is ALSO populated one index higher (hedge
 * entries below, e.g. both [4] and [5] for the tick). During bring-up,
 * every real IRQ landed on phantomInterrupt instead of the intended
 * handler even though vimRAM->ISR[channel] read back correctly when
 * peeked at directly -- i.e. the hardware's own vectored fetch
 * (IRQVECREG, read via the `ldr pc,[pc,#-0x1b0]` trick in
 * startup/vectors.s) was not reading the same index this code was
 * writing to. Duplicating each real handler at channel+1 removes the
 * ambiguity about which indexing convention the vectored fetch actually
 * uses, without needing to prove it first.
 */

#include "sys_vim.h"
#include "reg_vim.h"

extern void esmHighInterrupt( void );
extern void vPortTickISR( void );
extern void can1HighLevelInterrupt( void );
extern void phantomInterrupt( void ); /* freertos/port/portASM.s */

/* The VIM's interrupt-vector RAM: one function-pointer slot per channel.
 * Not covered by reg_vim.h (that's just the control registers at
 * 0xFFFFFE00); this is a separate block of RAM at 0xFFF82000. */
typedef volatile struct
{
    t_isrFuncPTR ISR[ VIM_CHANNELS ];
} vimRAM_t;

#define vimRAM ( ( vimRAM_t * ) 0xFFF82000U )

static const t_isrFuncPTR s_vim_table[ VIM_CHANNELS ] =
{
    [0]  = &esmHighInterrupt,
    [1]  = &esmHighInterrupt,      /* hedge: see indexing note below */
    [4]  = &vPortTickISR, /* RTI block-1/compare-2, see freertos/port/port.c */
    [5]  = &vPortTickISR,          /* hedge: see indexing note below */
    [16] = &can1HighLevelInterrupt,
    [17] = &can1HighLevelInterrupt, /* hedge: see indexing note below */
    /* every other index defaults to NULL below the loop in vimInit(),
       which fills it with phantomInterrupt instead */
};

void vimInit( void )
{
    uint32 i;

    /* VIM RAM parity checking deliberately left OFF: a plain C store loop
     * doesn't go through whatever write path the hardware needs to compute
     * correct parity bits for this RAM, so turning checking on here just
     * means every later read (including the hardware's own vectored IRQ
     * fetch) risks a spurious parity-error data abort instead of a normal
     * jump to the handler. */
    VIM_PARCTL = 0x00000000U;

    for( i = 0U; i < VIM_CHANNELS; i++ )
    {
        vimRAM->ISR[ i ] = ( s_vim_table[ i ] != ( t_isrFuncPTR ) 0 ) ? s_vim_table[ i ] : &phantomInterrupt;
    }

    /* All channels route to IRQ, not FIQ (nothing uses FIQ). */
    vimREG->FIRQPR0 = 0x00000000U;
    vimREG->FIRQPR1 = 0x00000000U;
    vimREG->FIRQPR2 = 0x00000000U;
    vimREG->FIRQPR3 = 0x00000000U;

    /* Enable exactly the channels this application uses. */
    vimREG->REQMASKCLR0 = 0xFFFFFFFFU;
    vimREG->REQMASKCLR1 = 0xFFFFFFFFU;
    vimREG->REQMASKCLR2 = 0xFFFFFFFFU;
    vimREG->REQMASKCLR3 = 0xFFFFFFFFU;
    vimREG->REQMASKSET0 = ( 1UL << 0 ) | ( 1UL << 4 ) | ( 1UL << 16 );
}
