/*
 * protocol.h
 *
 *  Created on: Aug 4, 2025
 *      Author: Asato
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include "stdint.h"

#define PACKET_COMMAND		1
#define PACKET_TEXT			2
#define PACKET_PING			3
#define PACKET_FIRMWARE		4
#define PACKET_DUMMY		5

#define FIRMWARE_UPLOADING	1
#define FIRMWARE_EOL		2

typedef struct {
	uint8_t type;
	uint8_t checksum;
	uint8_t	mode;
	uint8_t payload[16];
	uint8_t reserved[13];
} __attribute__((packed)) Firmware_t;

typedef struct {
	uint8_t type;
	uint8_t checksum;
	uint8_t reserved[30];
} __attribute__((packed)) Dummy_t;

#define PING_SEND			1
#define PING_CONFIRMED		2

typedef struct {
	uint8_t type;
	uint8_t checksum;
	uint8_t ack;
	uint8_t reserved[29];
} __attribute__((packed)) Ping_t;

typedef struct {
	uint8_t type;
	uint8_t checksum;
	uint8_t motor_address;
	uint8_t direction;
	uint8_t motor_pwm;
	uint8_t main_motor_X;
	uint8_t main_motor_Y;
	uint8_t reserved[25];
} __attribute__((packed)) Command_t;

#define STATUS_TEXT_SENDING			1
#define STATUS_TEXT_EOL				2

typedef struct {
	uint8_t type;
	uint8_t checksum;
	uint32_t len;
	uint8_t status;
	char payload[25];
} __attribute__((packed)) Message_t;

#endif
