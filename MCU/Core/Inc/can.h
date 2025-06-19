#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include "motor_control.h"
#include "hall_sensor.h"
#include "can.h"
#include "stdbool.h"

// Buffers for CAN RX and TX payloads (8 bytes each)
extern uint8_t RxData[8];
extern uint8_t TxData[8];

// CAN message header structures for incoming and outgoing frames
extern CAN_RxHeaderTypeDef RxHeader;
extern CAN_TxHeaderTypeDef TxHeader;

// Mailbox handle for transmitting messages
extern uint32_t TxMailbox;

// HAL CAN peripheral handle
extern CAN_HandleTypeDef hcan;

// Set up acceptance filter so we can receive messages
void setCanFilter(void);

// Pack and send current motor data over CAN
void sendcan(void);

// Callback when a new CAN message arrives in FIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

// Initialize Hall sensors (forward declaration, also in hall_sensor.h)
void init_hall_sensors(void);

#endif /* INC_CAN_H_ */
