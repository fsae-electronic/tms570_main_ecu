#include "drivers.h"

#include "FreeRTOS.h"
#include "os_task.h"

#include "canopen.h"

#define DRIVER_NODE_1 1U
#define DRIVER_NODE_2 2U
#define RPDO1_BASE_ID 0x200U

#define CONTROLWORD_DISABLE_VOLTAGE 0x0000U
#define CONTROLWORD_SHUTDOWN 0x0006U
#define CONTROLWORD_SWITCH_ON 0x0007U
#define CONTROLWORD_ENABLE_OPERATION 0x000FU

/* Gap between the frames of the init sequence: the drives only service
 * their own RPDO1 mailbox once per control cycle, so back-to-back frames
 * with no gap at all can overwrite each other there before the drive
 * reads them -- it only ever sees the last one. */
#define INIT_STEP_GAP_MS 5U

/* Torque RPDO rate once a drive is in DRIVE: run_drivers() itself is
 * polled much faster (see DRIVERS_PERIOD_MS in main.c) so the state
 * machine stays responsive, but sending torque at that same rate for both
 * nodes was needlessly loading the bus -- 25 Hz is plenty for a torque
 * setpoint. */
#define TORQUE_SEND_PERIOD_MS 40U

typedef enum
{
    DRIVER_STATE_INITIALIZING,
    DRIVER_STATE_DRIVE,
    DRIVER_STATE_STOP
} DriverState_t;

typedef struct
{
    uint16_t node_id;
    DriverState_t state;
} DriverChannel_t;

static DriverChannel_t s_drivers[2];

static bool send_controlword(uint16_t node_id, uint16_t controlword)
{
    uint8_t data[8] = {0U};

    data[0] = (uint8_t)(controlword & 0x00FFU);
    data[1] = (uint8_t)(controlword >> 8U);
    return canopen_send_pdo(RPDO1_BASE_ID + node_id, data);
}

static bool send_torque(uint16_t node_id, int16_t torque)
{
    uint8_t data[8] = {0U};
    uint16_t torque_bits = (uint16_t)torque;

    data[0] = (uint8_t)(CONTROLWORD_ENABLE_OPERATION & 0x00FFU);
    data[1] = (uint8_t)(CONTROLWORD_ENABLE_OPERATION >> 8U);
    data[2] = (uint8_t)(torque_bits & 0x00FFU);
    data[3] = (uint8_t)(torque_bits >> 8U);
    return canopen_send_pdo(RPDO1_BASE_ID + node_id, data);
}

static void stop_drivers(void)
{
    uint32_t index;

    for (index = 0U; index < 2U; index++)
    {
        if (s_drivers[index].state != DRIVER_STATE_STOP)
        {
            send_nmt_command(NMT_CMD_ENTER_PRE_OPERATIONAL, s_drivers[index].node_id);
            s_drivers[index].state = DRIVER_STATE_STOP;
        }
    }
}

/* Runs the whole OPERATIONAL -> CiA-402 enable sequence, pacing each frame
 * INIT_STEP_GAP_MS apart, and moves straight to DRIVE. */
static void advance_driver(DriverChannel_t *driver)
{
    switch (driver->state)
    {
    case DRIVER_STATE_INITIALIZING:
        // send_nmt_command(NMT_CMD_ENTER_PRE_OPERATIONAL, driver->node_id);
        send_nmt_command(NMT_CMD_ENTER_OPERATIONAL, driver->node_id);
        vTaskDelay(pdMS_TO_TICKS(INIT_STEP_GAP_MS));
        (void)send_controlword(driver->node_id, CONTROLWORD_SHUTDOWN);
        vTaskDelay(pdMS_TO_TICKS(INIT_STEP_GAP_MS));
        (void)send_controlword(driver->node_id, CONTROLWORD_SWITCH_ON);
        vTaskDelay(pdMS_TO_TICKS(INIT_STEP_GAP_MS));
        (void)send_controlword(driver->node_id, CONTROLWORD_ENABLE_OPERATION);
        driver->state = DRIVER_STATE_DRIVE;
        break;

    case DRIVER_STATE_DRIVE:
        /* Torque is sent from run_drivers() below. */
        break;

    case DRIVER_STATE_STOP:
    default:
        break;
    }
}

void init_drivers(void)
{
    s_drivers[0].node_id = DRIVER_NODE_1;
    s_drivers[0].state = DRIVER_STATE_STOP;
    s_drivers[1].node_id = DRIVER_NODE_2;
    s_drivers[1].state = DRIVER_STATE_STOP;
}

void run_drivers(bool drive_enabled, const int16_t torque_request[2])
{
    uint32_t index;
    static TickType_t s_last_torque_tick = 0U;
    TickType_t now;
    bool send_torque_now;

    if (!drive_enabled)
    {
        stop_drivers();
        return;
    }

    now = xTaskGetTickCount();
    send_torque_now = ( ( now - s_last_torque_tick ) >= pdMS_TO_TICKS( TORQUE_SEND_PERIOD_MS ) );
    if (send_torque_now)
    {
        s_last_torque_tick = now;
    }

    for (index = 0U; index < 2U; index++)
    {
        if (s_drivers[index].state == DRIVER_STATE_STOP)
        {
            s_drivers[index].state = DRIVER_STATE_INITIALIZING;
        }

        advance_driver(&s_drivers[index]);

        if ((s_drivers[index].state == DRIVER_STATE_DRIVE) && send_torque_now)
        {
            (void)send_torque(s_drivers[index].node_id, (int16_t)(-1)*torque_request[index]);
        }
    }
}
