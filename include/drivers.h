#ifndef DRIVERS_H_
#define DRIVERS_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the two CANopen motor nodes in the disabled state.
 */
void init_drivers(void);

/**
 * @brief Advance the motor start sequence or send the torque command.
 *
 * Keep @p start_requested true to execute PRE-OPERATIONAL, OPERATIONAL and
 * the CiA-402 enable sequence. Once both motors are enabled, each call sends
 * an RPDO1 torque command to each node. Set @p start_requested false to
 * disable both motors and return them to PRE-OPERATIONAL.
 *
 * @param start_requested State of the single start button.
 * @param torque_request Signed target torque per node ([0] = node 1/left,
 *        [1] = node 2/right), written to RPDO1 bytes 2 and 3.
 */
void run_drivers(bool start_requested, const int16_t torque_request[2]);

#endif /* DRIVERS_H_ */