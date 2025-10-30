/*
 * can.c
 *
 *  Created on: Oct 10, 2025
 *      Author: konno
 */
#include "main.h"
#include "stdio.h"
#include "string.h"

uint8_t loopbackTestPassed = 0;
volatile uint8_t messageReceived = 0;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;

typedef struct {
	uint8_t TxData[8];
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
} CANMessage;

static uint8_t canOK = 1;
static HAL_StatusTypeDef lastStatus = HAL_OK;

// Must be called after MX_CAN_Init();
uint8_t CANinit(CAN_HandleTypeDef hcan) {
	// Deinit
	if (HAL_CAN_DeInit(&hcan) != HAL_OK) {
		canOK = 0;
		Error_Handler();
	}
	// Change to loopback mode.
	hcan.Init.Mode = CAN_MODE_LOOPBACK;
	// Reinit
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		canOK = 0;
		Error_Handler();
	}
	// Configure loopback filder
	 CAN_FilterTypeDef loopbackFilter;
	  loopbackFilter.FilterBank = 0;
	  loopbackFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	  loopbackFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	  loopbackFilter.FilterIdHigh = 0x0000;
	  loopbackFilter.FilterIdLow = 0x0000;
	  loopbackFilter.FilterMaskIdHigh = 0x0000;
	  loopbackFilter.FilterMaskIdLow = 0x0000;
	  loopbackFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	  loopbackFilter.FilterActivation = ENABLE;
	  loopbackFilter.SlaveStartFilterBank = 14;

	  if (HAL_CAN_ConfigFilter(&hcan, &loopbackFilter) != HAL_OK) {
		  canOK = 0;
	      Error_Handler();
	  }

	  // Start CAN with notifications enabled
	  if (HAL_CAN_Start(&hcan) != HAL_OK) {
		  canOK = 0;
	      Error_Handler();
	  }

	  // Activate RX notification
	  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		  canOK = 0;
	      Error_Handler();
	  }

	  // Configure test message
	  TxHeader.StdId = 0x123;  // Standard ID for loopback test
	  TxHeader.ExtId = 0x00;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.DLC = 8;
	  TxHeader.TransmitGlobalTime = DISABLE;

	  // Fill test data
	  for (int i = 0; i < 8; i++) {
	      TxData[i] = i + 1;  // Data: 1, 2, 3, 4, 5, 6, 7, 8
	  }

	  // Reset flag
	  messageReceived = 0;

	  // Send loopback test message
	  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
	      // Wait for loopback reception (timeout after 200ms)
	      uint32_t startTick = HAL_GetTick();
	      while (!messageReceived && (HAL_GetTick() - startTick) < 200) {
	          HAL_Delay(1);
	      }

	      // Check if message was received
	      if (messageReceived) {
	          // Verify the received message matches what we sent
	          if (RxHeader.StdId == 0x123 && RxHeader.DLC == 8) {
	              uint8_t dataMatch = 1;
	              for (int i = 0; i < 8; i++) {
	                  if (RxData[i] != (i + 1)) {
	                      dataMatch = 0;
	                      break;
	                  }
	              }

	              if (dataMatch) {
	                  loopbackTestPassed = 1;  // Test PASSED
	              }
	          }
	      }
	  }
	  // Stop the CAN Peripheral
	  if (HAL_CAN_Stop(&hcan) != HAL_OK) {
	  	  		canOK = 0;
	  	  		Error_Handler();
	  	  	}
	  // Deinit
	  if (HAL_CAN_DeInit(&hcan) != HAL_OK) {
	  		canOK = 0;
	  		Error_Handler();
	  	}
	  	// Change to normal mode.
	  	hcan.Init.Mode = CAN_MODE_NORMAL;
	  	// Reinit
	  	if (HAL_CAN_Init(&hcan) != HAL_OK) {
	  		canOK = 0;
	  		Error_Handler();
	  	}
	  	// Restart
	  	if (HAL_CAN_Start(&hcan) != HAL_OK) {
	  		  	  		canOK = 0;
	  		  	  		Error_Handler();
	  		  	  	}
	  // loopbackTestPassed will be 1 if test passed, 0 if failed
	  return loopbackTestPassed;
}

static HAL_StatusTypeDef sendCANFrame(CAN_HandleTypeDef* hcan, CANMessage* msg) {

	msg->TxHeader.IDE = CAN_ID_EXT;
	msg->TxHeader.ExtId = 0x18FF0001; // 0x18FF0001
	msg->TxHeader.RTR = CAN_RTR_DATA;


	lastStatus = HAL_CAN_AddTxMessage(hcan, &msg->TxHeader, msg->TxData, &msg->TxMailbox);
	if (lastStatus != HAL_OK) {
		canOK = 0;
		return lastStatus;
	}
	return lastStatus;
}

/**
 * @brief Send string message over CAN
 * @param hcan: Pointer to CAN handle
 * @param str: Null-terminated string to send
 *
 * Sends complete string in one transmission
 * Automatically handles length calculation
 */
HAL_StatusTypeDef CANSendMessage(CAN_HandleTypeDef* hcan, const char* str) {
    CANMessage msg;

    if (str == NULL) {
        canOK = 0;
        lastStatus = HAL_ERROR;
        return lastStatus;
    }

    size_t len = strlen(str);
    msg.TxHeader.DLC = len;
    memcpy(msg.TxData, str, len); // Copy data into the array


    return sendCANFrame(hcan, &msg);
}

/**
 * @brief Send sensor distance reading over CAN
 * @param huart: Pointer to CAN handle
 * @param mm: Distance in millimeters (0-65535)
 *
 * Formats and sends: "Distance: XXXmm\r\n"
 * Uses CANSendMessage internally
 */
HAL_StatusTypeDef CANSendSensor(CAN_HandleTypeDef* hcan, uint16_t mm) {
    char buffer[32];

    /* Format the distance message */
    snprintf(buffer, sizeof(buffer), "Distance: %umm\r\n", mm);

    return CANSendMessage(hcan, buffer);
}

void CANSendData() {

}

uint8_t CANok() {
	return canOK;
}

uint8_t CANMessageRecieved() {
	return messageReceived;
}

uint8_t CANLoopbackPassed() {
	return loopbackTestPassed;
}

HAL_StatusTypeDef CANGetLastStatus() {
	return lastStatus;
}

void CANClearErrors(CAN_HandleTypeDef* hcan) {
	if (hcan == NULL) {
		return;
	}

	hcan->ErrorCode = HAL_CAN_ERROR_NONE;
	canOK = 1;
}

// CAN RX callback - called when message is received
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // Get the received message and store in global variables
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        messageReceived = 1;  // Signal that a message was received
    }
}
