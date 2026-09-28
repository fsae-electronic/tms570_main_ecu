#include "charger.h"

#include "os_task.h"

#include "can.h"

/* CAN2 mailboxes (charger bus, used only by this task). */
#define CHARGER_RX_MAILBOX      canMESSAGE_BOX1
#define CHARGER_TX_MAILBOX      canMESSAGE_BOX2

/* CAN1 mailboxes (vehicle bus) -- canInit() uses boxes 1-24 and 27 there. */
#define CHARGER_CMD_MAILBOX     canMESSAGE_BOX25
#define CHARGER_DATA_MAILBOX    canMESSAGE_BOX26

/* CAN2 bit timing for 250 kbit/s from VCLK1 = 80 MHz (see system.h):
 * 80 MHz / 20 = 4 MHz -> 16 tq per bit = 1 sync + 13 TSeg1 + 2 TSeg2,
 * sample point at 87.5 %, SJW = 2 tq. */
#define CAN2_BRP                20U
#define CAN2_TSEG1              13U
#define CAN2_TSEG2              2U
#define CAN2_SJW                2U

/* IFxARB / IFxMSK / IFxMCTL bits */
#define CAN_ARB_MSGVAL          0x80000000U
#define CAN_ARB_XTD             0x40000000U
#define CAN_ARB_DIR_TX          0x20000000U
#define CAN_EXT_ID_MSK          (0xC0000000U | 0x1FFFFFFFU)
#define CAN_MCTL_UMASK          0x00001000U
#define CAN_MCTL_EOB            0x00000080U

#define STACK_CHARGER           256U

typedef struct
{
    uint16_t actual_voltage_dv;     /* Voltaje de salida (lo que ve el cargador) */
    uint16_t actual_current_da;     /* Corriente de salida (lo que ve el cargador) */
    uint8_t  status_flags;

    bool charging;                  /* Frame del cargador dentro de CHARGER_RX_TIMEOUT_MS */
    TickType_t last_rx_tick;
} Charger_t;

static Charger_t s_charger;

static StaticTask_t s_charger_task_buffer;
static StackType_t s_charger_stack[STACK_CHARGER];

/* ---- Low-level CAN ---------------------------------------------------- */

static void config_mailbox(canBASE_t *node, uint32 box, uint32 arb, uint32 msk, uint32 mctl)
{
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status check" */
    while ((node->IF1STAT & 0x80U) == 0x80U)
    {
    } /* Wait */

    node->IF1MSK  = msk;
    node->IF1ARB  = arb;
    node->IF1MCTL = mctl;
    node->IF1CMD  = (uint8)0xF8U;
    node->IF1NO   = (uint8)box;

    while ((node->IF1STAT & 0x80U) == 0x80U)
    {
    } /* Wait */
}

/* Same register sequence canInit() uses for CAN1, but at 250 kbit/s and
 * with one 29-bit RX and one 29-bit TX mailbox for the charger. */
static void can2_init(void)
{
    /* Init + CCE, parity off, auto-retransmission on (HALCoGen default
     * minus IE0 -- see the comment in canInit()). */
    canREG2->CTL = (uint32)((uint32)0x00000005U << 10U) | (uint32)0x00020041U;
    canREG2->ES |= 0xFFFFFFFFU;
    canREG2->INTMUXx[0U] = 0U;
    canREG2->INTMUXx[1U] = 0U;
    canREG2->ABOTR = 0U;

    config_mailbox(canREG2, CHARGER_RX_MAILBOX,
                   CAN_ARB_MSGVAL | CAN_ARB_XTD | CHARGER_CAN_ID_RX,
                   CAN_EXT_ID_MSK,
                   CAN_MCTL_UMASK | CAN_MCTL_EOB | 8U);

    config_mailbox(canREG2, CHARGER_TX_MAILBOX,
                   CAN_ARB_MSGVAL | CAN_ARB_XTD | CAN_ARB_DIR_TX | CHARGER_CAN_ID_TX,
                   CAN_EXT_ID_MSK,
                   CAN_MCTL_EOB | 8U);

    canREG2->BTR = (uint32)((uint32)0U << 16U) |
                   (uint32)((uint32)(CAN2_TSEG2 - 1U) << 12U) |
                   (uint32)((uint32)(CAN2_TSEG1 - 1U) << 8U) |
                   (uint32)((uint32)(CAN2_SJW - 1U) << 6U) |
                   (uint32)(CAN2_BRP - 1U);

    /* CAN2TX/CAN2RX in functional mode */
    canREG2->TIOC = (uint32)((uint32)1U << 18U) | (uint32)((uint32)1U << 3U) |
                    (uint32)((uint32)1U << 2U)  | (uint32)((uint32)1U << 1U);
    canREG2->RIOC = (uint32)((uint32)1U << 18U) | (uint32)((uint32)1U << 3U);

    canREG2->CTL &= ~(uint32)(0x00000041U);
}

static void can1_report_mailboxes_init(void)
{
    config_mailbox(canREG1, CHARGER_CMD_MAILBOX,
                   CAN_ARB_MSGVAL | CAN_ARB_DIR_TX | canSTD_ID(CHARGER_CAN1_ID_CMD),
                   canSTD_ID_MSK, CAN_MCTL_EOB | 8U);

    config_mailbox(canREG1, CHARGER_DATA_MAILBOX,
                   CAN_ARB_MSGVAL | CAN_ARB_DIR_TX | canSTD_ID(CHARGER_CAN1_ID_DATA),
                   canSTD_ID_MSK, CAN_MCTL_EOB | 8U);
}

/* CAN1's IF1 is shared with the drivers task (canopen.c), so keep other
 * tasks off it while this transmit is loading the interface registers. */
static void can1_transmit(uint32 box, const uint8 data[8])
{
    vTaskSuspendAll();
    (void)canTransmit(canREG1, box, data);
    (void)xTaskResumeAll();
}

/* ---- Charger protocol ----------------------------------------------- */

static bool charger_receive(Charger_t *charger)
{
    uint8 raw[8];

    if (canGetData(canREG2, CHARGER_RX_MAILBOX, raw) == 0U)
    {
        return false;
    }

    charger->actual_voltage_dv = (uint16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    charger->actual_current_da = (uint16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    charger->status_flags = raw[4];
    charger->last_rx_tick = xTaskGetTickCount();
    charger->charging = true;
    return true;
}

/* Byte-for-byte the frame send_can2charger() builds on the LPC board:
 * B2 fixed at 0x10 and the B5 rolling counter are needed by the charger. */
static void charger_send(float voltage, float current)
{
    static uint8 data[8] = {0U};
    uint16_t voltage_x10 = (uint16_t)(voltage * 10.0f);
    uint16_t current_x10 = (uint16_t)(current * 10.0f);

    data[0] = (uint8)((voltage_x10 >> 8) & 0xFFU); /* Output Voltage High Byte */
    data[1] = (uint8)(voltage_x10 & 0xFFU);        /* Output Voltage Low Byte */
    data[2] = 0x10; /* Output Current High Byte */
    data[3] = 0x00;        /* Output Current Low Byte */
    data[4] = 0x00U;                               /* STATUS Flags */

    if (data[5] < 0xFFU)
    {
        data[5]++;
    }
    else
    {
        data[5] = 0x00U;
    }

    (void)canTransmit(canREG2, CHARGER_TX_MAILBOX, data);
}

static void report_send_cmd(const Charger_t *charger)
{
    uint8 data[8] = {0U};

    data[0] = charger->charging ? 1U : 0U;
    data[1] = charger->status_flags;
    can1_transmit(CHARGER_CMD_MAILBOX, data);
}

static void report_send_data(const Charger_t *charger)
{
    uint8 data[8];
    uint16_t voltage_x10 = (uint16_t)(CHARGER_VOLTAGE * 10.0f);
    uint16_t current_x10 = (uint16_t)(CHARGER_CURRENT * 10.0f);

    data[0] = (uint8)(voltage_x10 & 0xFFU);
    data[1] = (uint8)(voltage_x10 >> 8);
    data[2] = (uint8)(current_x10 & 0xFFU);
    data[3] = (uint8)(current_x10 >> 8);
    data[4] = (uint8)(charger->actual_voltage_dv & 0xFFU);
    data[5] = (uint8)(charger->actual_voltage_dv >> 8);
    data[6] = (uint8)(charger->actual_current_da & 0xFFU);
    data[7] = (uint8)(charger->actual_current_da >> 8);
    can1_transmit(CHARGER_DATA_MAILBOX, data);
}

/* ---- Task ------------------------------------------------------------- */

static portTASK_FUNCTION(vChargerTask, pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_tx = last_wake;
    TickType_t last_report = last_wake;
    bool was_charging = false;
    (void)pvParameters;

    for (;;)
    {
        TickType_t now;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CHARGER_TASK_PERIOD_MS));
        now = xTaskGetTickCount();

        /* Frame fijo al cargador, como SysTick_RegisterCallback(test, 950) */
        if ((now - last_tx) >= pdMS_TO_TICKS(CHARGER_TX_PERIOD_MS))
        {
            last_tx = now;
            charger_send(CHARGER_VOLTAGE, CHARGER_CURRENT);
        }

        if (!charger_receive(&s_charger) && s_charger.charging &&
            ((now - s_charger.last_rx_tick) >= pdMS_TO_TICKS(CHARGER_RX_TIMEOUT_MS)))
        {
            /* Cargador desconectado */
            s_charger.charging = false;
        }

        if (s_charger.charging)
        {
            if (!was_charging || ((now - last_report) >= pdMS_TO_TICKS(CHARGER_REPORT_PERIOD_MS)))
            {
                last_report = now;
                report_send_cmd(&s_charger);
                report_send_data(&s_charger);
            }
            was_charging = true;
        }
        else if (was_charging)
        {
            /* Aviso a la on-board ECU que dejo de cargar */
            report_send_cmd(&s_charger);
            was_charging = false;
        }
    }
}

/* ---- Public API ------------------------------------------------------- */

void charger_init(UBaseType_t priority)
{
    s_charger.charging = false;

    can2_init();
    can1_report_mailboxes_init();

    xTaskCreateStatic(vChargerTask, "Charger", STACK_CHARGER, NULL,
                      priority, s_charger_stack, &s_charger_task_buffer);
}
