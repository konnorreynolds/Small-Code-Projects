/*
 * can.h
 *
 *  Created on: Oct 10, 2025
 *      Author: konno
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

/* Includes */
#include "main.h"

uint8_t CANinit();

/**
 * @brief Send string message over CAN
 * @param hcan: Pointer to CAN handle
 * @param str: Null-terminated string to send
 *
 * Sends complete string in one transmission
 * Automatically handles length calculation
 */
HAL_StatusTypeDef CANSendMessage(CAN_HandleTypeDef* hcan, const char* str);

HAL_StatusTypeDef CANSendSensor(CAN_HandleTypeDef* hcan, uint16_t mm);

uint8_t CANok();

uint8_t CANMessageRecieved();

HAL_StatusTypeDef CANSendData();

void CANClearErrors(CAN_HandleTypeDef* hcan);


#endif /* INC_CAN_H_ */
