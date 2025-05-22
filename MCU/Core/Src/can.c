#include "can.h"
#include "hall_sensor.h"
#include "motor_control.h"

// RX and TX data arrays
uint8_t RxData[8];
uint8_t TxData[8];

// CAN headers and mailbox
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;

// Variables to hold setpoints received via CAN
double fl = 0;
double fr = 0;
double rl = 0;
double rr = 0;

/**
 * @brief  Called by HAL when a CAN message is pending in FIFO0.
 *         Reads the message, extracts four 16-bit values into fl, fr, rl, rr.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
        Error_Handler();
    }
    // Combine two bytes per value, interpret as signed int16
    fl = (int16_t)((RxData[0] << 8) | RxData[1]);
    fr = (int16_t)((RxData[2] << 8) | RxData[3]);
    rl = (int16_t)((RxData[4] << 8) | RxData[5]);
    rr = (int16_t)((RxData[6] << 8) | RxData[7]);
}

/**
 * @brief  Gather motor positions, pack into TxData, and send over CAN.
 */
void sendcan(void) {
    // Get each motor's position (could switch to speed if desired)
    int16_t pos1 = get_motor_position(1);
    int16_t pos2 = get_motor_position(2);
    int16_t pos3 = get_motor_position(3);
    int16_t pos4 = get_motor_position(4);

    // Prepare CAN header for extended ID, data frame, 8 bytes
    TxHeader.ExtId = 0x00000001;
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.IDE   = CAN_ID_EXT;
    TxHeader.DLC   = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    // Split each 16-bit value into two bytes
    TxData[0] = (uint8_t)(pos1 >> 8);
    TxData[1] = (uint8_t)(pos1);
    TxData[2] = (uint8_t)(pos2 >> 8);
    TxData[3] = (uint8_t)(pos2);
    TxData[4] = (uint8_t)(pos3 >> 8);
    TxData[5] = (uint8_t)(pos3);
    TxData[6] = (uint8_t)(pos4 >> 8);
    TxData[7] = (uint8_t)(pos4);

    // Queue the message for transmission
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  Configure CAN filter to accept all IDs and start CAN with interrupts.
 */
void setCanFilter(void) {
    CAN_FilterTypeDef CAN_FilterInitStructure;

    CAN_FilterInitStructure.FilterBank           = 0;
    CAN_FilterInitStructure.FilterMode           = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStructure.FilterScale          = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStructure.FilterIdHigh         = 0x0000;
    CAN_FilterInitStructure.FilterIdLow          = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdHigh     = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdLow      = 0x0000;
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterInitStructure.FilterActivation     = ENABLE;
    CAN_FilterInitStructure.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &CAN_FilterInitStructure) != HAL_OK) {
        Error_Handler();
    }

    // Enable RX interrupt (new message) and TX mailbox empty interrupt
    if (HAL_CAN_ActivateNotification(&hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
        Error_Handler();
    }

    // Start CAN peripheral
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }
}
