#include "drivers.h"

#include "canopen.h"

#define DRIVER_NODE_1 1U
#define DRIVER_NODE_2 2U
#define RPDO1_BASE_ID 0x200U

#define CONTROLWORD_DISABLE_VOLTAGE 0x0000U
#define CONTROLWORD_SHUTDOWN 0x0006U
#define CONTROLWORD_SWITCH_ON 0x0007U
#define CONTROLWORD_ENABLE_OPERATION 0x000FU

typedef enum
{
    DRIVER_STATE_OFF,
    DRIVER_STATE_PRE_OPERATIONAL,
    DRIVER_STATE_OPERATIONAL,
    DRIVER_STATE_SHUTDOWN,
    DRIVER_STATE_SWITCHED_ON,
    DRIVER_STATE_ENABLED
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
        if (s_drivers[index].state != DRIVER_STATE_OFF)
        {
            (void)send_controlword(s_drivers[index].node_id, CONTROLWORD_DISABLE_VOLTAGE);
            send_nmt_command(NMT_CMD_ENTER_PRE_OPERATIONAL, s_drivers[index].node_id);
            s_drivers[index].state = DRIVER_STATE_OFF;
        }
    }
}

static void advance_driver(DriverChannel_t *driver)
{
    switch (driver->state)
    {
    case DRIVER_STATE_OFF:
        send_nmt_command(NMT_CMD_ENTER_PRE_OPERATIONAL, driver->node_id);
        driver->state = DRIVER_STATE_PRE_OPERATIONAL;
        break;

    case DRIVER_STATE_PRE_OPERATIONAL:
        send_nmt_command(NMT_CMD_ENTER_OPERATIONAL, driver->node_id);
        driver->state = DRIVER_STATE_OPERATIONAL;
        break;

    case DRIVER_STATE_OPERATIONAL:
        if (send_controlword(driver->node_id, CONTROLWORD_SHUTDOWN))
        {
            driver->state = DRIVER_STATE_SHUTDOWN;
        }
        break;

    case DRIVER_STATE_SHUTDOWN:
        if (send_controlword(driver->node_id, CONTROLWORD_SWITCH_ON))
        {
            driver->state = DRIVER_STATE_SWITCHED_ON;
        }
        break;

    case DRIVER_STATE_SWITCHED_ON:
        if (send_controlword(driver->node_id, CONTROLWORD_ENABLE_OPERATION))
        {
            driver->state = DRIVER_STATE_ENABLED;
        }
        break;

    case DRIVER_STATE_ENABLED:
    default:
        break;
    }
}

void init_drivers(void)
{
    s_drivers[0].node_id = DRIVER_NODE_1;
    s_drivers[0].state = DRIVER_STATE_OFF;
    s_drivers[1].node_id = DRIVER_NODE_2;
    s_drivers[1].state = DRIVER_STATE_OFF;
}

void run_drivers(bool start_requested, int16_t torque_request)
{
    uint32_t index;
    bool drivers_enabled = true;

    if (!start_requested)
    {
        stop_drivers();
        return;
    }

    for (index = 0U; index < 2U; index++)
    {
        advance_driver(&s_drivers[index]);
        if (s_drivers[index].state != DRIVER_STATE_ENABLED)
        {
            drivers_enabled = false;
        }
    }

    if (drivers_enabled)
    {
        for (index = 0U; index < 2U; index++)
        {
            (void)send_torque(s_drivers[index].node_id, torque_request);
        }
    }
}