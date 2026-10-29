/*
 * can_open.c
 *
 *  Created on: 9 sept 2024
 *      Author: asolari
 */

#include "canopen.h"
#include "can.h"
#include "FreeRTOS.h"

#define CANOPEN_NMT_TX_MAILBOX canMESSAGE_BOX5
#define CANOPEN_SDO_TX_MAILBOX canMESSAGE_BOX6
#define CANOPEN_PDO_TX_MAILBOX canMESSAGE_BOX8
#define CANOPEN_RX_MAILBOX canMESSAGE_BOX16
/* Xtd stays clear (standard 11-bit COB-IDs) and the identifier goes into
 * bits 28:18 of IFxARB -- see canSTD_ID() in can.h. */
#define CANOPEN_STANDARD_TX_ARB(id) (0x20000000U | canSTD_ID(id))
#define CANOPEN_STANDARD_RX_ARB(id) (canSTD_ID(id))

/* Longest wait for the previous frame of a mailbox to go out. A frame takes
 * ~0.13 ms at 1 Mbit/s; if it is still pending after this the bus is stuck
 * (bus-off, nobody acking) and the send is dropped instead of hanging the
 * caller (the drivers task). The time comes from RTI counter 1, which only
 * runs once the scheduler started, so CANOPEN_TX_MAX_POLLS also bounds the
 * wait before that (init_drivers()). */
#define CANOPEN_TX_TIMEOUT_MS 2U
#define CANOPEN_TX_TIMEOUT_COUNTS ((RUN_TIME_COUNTER_HZ / 1000U) * CANOPEN_TX_TIMEOUT_MS)
#define CANOPEN_TX_MAX_POLLS 100000U

static bool canopen_transmit(uint32 mailbox, uint16_t id, const uint8_t *data)
{
	uint32_t start = portGET_RUN_TIME_COUNTER_VALUE();
	uint32_t polls = 0U;

	while (canIsTxMessagePending(canREG1, mailbox) != 0U)
	{
		polls++;
		if (((portGET_RUN_TIME_COUNTER_VALUE() - start) > CANOPEN_TX_TIMEOUT_COUNTS) ||
			(polls > CANOPEN_TX_MAX_POLLS))
		{
			return false;
		}
	}

	canUpdateID(canREG1, mailbox, CANOPEN_STANDARD_TX_ARB(id));
	return canTransmit(canREG1, mailbox, data) != 0U;
}

bool canopen_send_pdo(uint16_t cob_id, const uint8_t data[8])
{
	return canopen_transmit(CANOPEN_PDO_TX_MAILBOX, cob_id, data);
}

bool canopen_read(uint16_t cob_id, uint8_t data[8])
{
	if (canIsRxMessageArrived(canREG1, CANOPEN_RX_MAILBOX) == 0U)
	{
		return false;
	}

	if (canGetData(canREG1, CANOPEN_RX_MAILBOX, data) == 0U)
	{
		return false;
	}

	return canGetID(canREG1, CANOPEN_RX_MAILBOX) == cob_id;
}

static void canopen_prepare_sdo_response(uint16_t node_id)
{
	canUpdateID(canREG1, CANOPEN_RX_MAILBOX,
			CANOPEN_STANDARD_RX_ARB(0x580U + node_id));
}



////////////////////////////////////////////////////
//                 SDO WRITE                      //
////////////////////////////////////////////////////

/***
 * @brief Send a SDO Write command
 * bool send_sdo_command(uint8_t command, uint16_t od_index,
 * uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 * @return bool : 1 = send success
 * command: 0x23 = Write 4 bytes
 * command: 0x2F = Write 3 bytes
 * command: 0x2B = Write 2 bytes
 * command: 0x27 = Write 1 byte
 */

bool send_sdo_write_command (uint8_t command, uint16_t od_index, uint8_t od_sub_index, uint32_t data, uint16_t node_id)
{
	uint8_t sdo_data[8];
	uint16_t cob_id = 0x600U + node_id;  // COB-ID del SDO

	sdo_data[0] = command;  // Comando SDO
	sdo_data[1] = (uint8_t)(od_index & 0x00FFU);  // Índice bajo
	sdo_data[2] = (uint8_t)((od_index >> 8U) & 0x00FFU);   // Índice alto
	sdo_data[3] = od_sub_index;     // Subíndice
	sdo_data[4] = (uint8_t)(data & 0x000000FFU);      // Byte 0 del valor
	sdo_data[5] = (uint8_t)((data >> 8U) & 0x000000FFU);  // Byte 1 del valor
	sdo_data[6] = (uint8_t)((data >> 16U) & 0x000000FFU); // Byte 2 del valor
	sdo_data[7] = (uint8_t)((data >> 24U) & 0x000000FFU); // Byte 3 del valor

	canopen_prepare_sdo_response(node_id);
	return canopen_transmit(CANOPEN_SDO_TX_MAILBOX, cob_id, sdo_data);
}


/**
 * @brief Receive a SDO Write message
 *	bool recive_sdo_command(uint8_t command, uint16_t od_index,
 *	 uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 * 	@return 1 = Recive success
 */
bool recive_sdo_write_command(uint8_t command, uint16_t od_index, uint8_t od_sub_index, int32_t data, uint16_t node_id)
{
	uint8_t sdo_reply[8];

	(void)command;
	(void)data;
	if (canopen_read(0x580U + node_id, sdo_reply))
	{
		if ((sdo_reply[0] == 0x60U)
			&& (sdo_reply[1] == (uint8_t)(od_index & 0x00FFU))
			&& (sdo_reply[2] == (uint8_t)((od_index >> 8) & 0x00FFU))
			&& (sdo_reply[3] == od_sub_index))
		{
			return true;
		}
	}

	return false;
}

////////////////////////////////////////////////////
//                 SDO READ                       //
////////////////////////////////////////////////////
/**
 * @brief Send a SDO Read command
 * bool send_sdo_command(uint8_t command, uint16_t od_index,
 * uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 * @return bool : 1 = send success
 */
bool send_sdo_read_command(uint16_t od_index, uint8_t od_sub_index, uint16_t node_id)
{
	uint8_t sdo_data[8];
	uint16_t cob_id = 0x600U + node_id;  // COB-ID del SDO

	sdo_data[0] = 0x40U;  // Comando SDO
	sdo_data[1] = (uint8_t)(od_index & 0x00FFU);  // Índice bajo
	sdo_data[2] = (uint8_t)((od_index >> 8U) & 0x00FFU);   // Índice alto
	sdo_data[3] = od_sub_index;     // Subíndice
	sdo_data[4] = 0x00U;      // Byte 0 del valor
	sdo_data[5] = 0x00U;  // Byte 1 del valor
	sdo_data[6] = 0x00U; // Byte 2 del valor
	sdo_data[7] = 0x00U; // Byte 3 del valor

	canopen_prepare_sdo_response(node_id);
	return canopen_transmit(CANOPEN_SDO_TX_MAILBOX, cob_id, sdo_data);
}

/**
 * @brief Receive a SDO Read message
 *	bool recive_sdo_command(uint8_t command, uint16_t od_index,
 *	 uint8_t od_sub_index, uint32_t data, uint8_t node_id)
 *
 *	 0x43 : 4 bytes sent
 *	 0x47 : 3 bytes sent
 *	 0x4B : 2 bytes sent
 *	 0x4F : 1 byte sent
 *
 * 	@return data : 4 bytes
 */
int32_t recive_sdo_read_command(uint8_t command, uint16_t od_index, uint8_t od_sub_index, uint16_t node_id)
{
	uint8_t sdo_reply[8];

	(void)command;
	if (canopen_read(0x580U + node_id, sdo_reply))
	{
		if ((sdo_reply[1] == (uint8_t)(od_index & 0x00FFU))
			&& (sdo_reply[2] == (uint8_t)((od_index >> 8) & 0x00FFU))
			&& (sdo_reply[3] == od_sub_index))
		{
			if ((sdo_reply[0] == 0x43U) || (sdo_reply[0] == 0x47U) ||
				(sdo_reply[0] == 0x4BU) || (sdo_reply[0] == 0x4FU))
			{
				return (int32_t)((uint32)sdo_reply[4] |
						((uint32)sdo_reply[5] << 8U) |
						((uint32)sdo_reply[6] << 16U) |
						((uint32)sdo_reply[7] << 24U));
			}
		}
	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Send a NMT CAN message
 * void send_nmt_command(uint8_t command, uint8_t node_id)
 *
 * @param command NMT command :
 * 0x01 for Operational
 * 0x02 for Stop
 * 0x80 for Pre-operational
 * 0x81 for Reset Node
 * 0x82 for Reset Communication
 * @param node_id Node ID of the slave
 * @return void
 */
bool send_nmt_command(uint8_t command, uint16_t node_id) {
	uint8_t nmt_data[8] = {0U};
	nmt_data[0] = command; // NMT command (0x01 for Operational, 0x80 for Pre-operational)
	nmt_data[1] = (uint8_t) node_id;  // Node ID of the slave

	return canopen_transmit(CANOPEN_NMT_TX_MAILBOX, 0x000U, nmt_data);
}
