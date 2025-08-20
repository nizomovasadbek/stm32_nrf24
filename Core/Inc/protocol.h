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
#define PACKET_CHANGE		6
#define PACKET_STATUS		7

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

typedef struct {

	uint8_t type;
	uint8_t checksum;
	uint8_t reserved[30];

} __attribute__((packed)) Ping_t;

typedef struct {

	uint8_t type;
	uint8_t checksum;
	uint8_t M1;
	uint8_t M2;
	uint8_t M3;
	uint8_t M4;
	uint8_t M5;
	uint8_t M6;
	uint8_t M7;
	uint8_t M8;
	uint8_t M9;

	uint8_t reserved[21];

} __attribute__((packed)) Command_t;

#define STATUS_TEXT_SENDING			1
#define STATUS_TEXT_EOL				2

typedef struct {

	uint8_t type;
	uint8_t checksum;
	uint32_t len;
	uint8_t status;
	uint8_t payload[25];

} __attribute__((packed)) Message_t;

#define CHANGE_TRANSMITTER	1
#define CHANGE_RECEIVER		2

typedef struct {

	uint8_t type;
	uint8_t checksum;
	uint8_t mode;
	uint8_t reserved[29];

} __attribute__((packed)) Change_t;


typedef struct {
	uint8_t type;
	uint8_t checksum;
	uint16_t battery;
	uint16_t temperature;
	uint8_t reserved[28];
} __attribute__((packed))  Status_t;

#endif
