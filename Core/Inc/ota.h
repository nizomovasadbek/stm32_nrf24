/*
 * ota.h
 *
 *  Created on: Aug 5, 2025
 *      Author: Asato
 */

#ifndef INC_OTA_H_
#define INC_OTA_H_

#include "type.h"

typedef struct {
	volatile u32 FLASH_ACR;
	volatile u32 FLASHKEY;
	volatile u32 FLASHOPTKEY;
	volatile u32 FLASH_SR;
	volatile u32 FLASH_CR;
	volatile u32 FLASH_OPTCR;
} FLASH_t;

typedef struct {
	u32 MODER;
	u32 OTYPER;
	u32 RESERVED0;
	u32 OSPEEDR;
	u32 PUPDR;
	u32 IDR;
	u32 RESERVED1;
	u32 ODR;
	u32 RESERVED2;
	u32 BSRR;
	u32 LCKR;
	u32 AFRL;
	u32 AFRH;
} GPIO_t;

typedef struct {
	u16 SPI_CR1;
	u16 RESERVED0;
	u16 SPI_CR2;
	u16 RESERVED1;
	u16 SPI_SR;
	u16 RESERVED2;
	u16 SPI_DR;
	u16 RESERVED3;
	u16 SPI_CRCPR;
	u16 RESERVED4;
	u16 SPI_RXCRCR;
	u16 RESERVED5;
	u16 SPI_TXCRCR;
	u16 RESERVED6;
	u16 SPI_I2SCFGR;
	u16 RESERVED7;
	u16 SPI_I2SPR;
	u16 RESERVED8;
} SPI_t;

#define kSPI1	(  *(  volatile SPI_t*  ) 0x40013000  )
#define kGPIOB	(  *(  volatile GPIO_t*  ) 0x40020400  )
#define kFLASH	(  *(  volatile FLASH_t*  ) 0x40023C00  )
#define DBGMCUIDCODE	0xE0042000

// SPI_CR1
#define CPHA			0
#define CPOL			1
#define MSTR			2
#define BR				3
#define SPE				6
#define LSBFIRST		7
#define SSI				8
#define SSM				9
#define RXONLY			10
#define DFF				11
#define CRCNEXT			12
#define CRCEN			13
#define BIDIOE			14
#define BIDIMODE		15


// SPI_CR2
#define RXDMAEN			0
#define TXDMAEN			1
#define SSOE			2
#define FRF				4
#define ERRIE			5
#define RXNEIE			6
#define TXEIE			7

//SPI_SR
#define RXNE			0
#define TXE				1
#define CHSIDE			2
#define UDR				3
#define CRCERR			4
#define MODF			5
#define OVR				6
#define BSY				7
#define FRE				8

// SPI_I2SCFGR
#define CHLEN			0
#define DATLEN			1
#define CKPOL			3
#define I2SSTD			4
#define PCMSYNC			7
#define I2SCFG			8
#define I2SE			10
#define I2SMOD			11

// SPI_I2SPR
#define ODD				8
#define MCKOE			9

#define _50MHz			(  0 << BR  )
#define	_25MHz			(  1 << BR  )
#define _12v5MHz		(  2 << BR  )
#define _6v25MHz		(  3 << BR  )
#define _3v125MHz		(  4 << BR  )
#define _1v56MHz		(  5 << BR  )
#define _781KHz			(  6 << BR  )
#define _390KHz			(  7 << BR  )

#define F_KEY1		0x45670123U
#define F_KEY2		0xCDEF89ABU

// FLASH_ACR register
#define LATENCY			0
#define PRFTEN			8
#define ICEN			9
#define DCEN			10
#define ICRST			11
#define DCRST			12

// FLASH_SR register
#define EOP				0
#define OPERR			1
#define WRPERR			4
#define PGAERR			5
#define PGPERR			6
#define PGSERR			7
#define RDERR			8
#define FLASH_BUSY		16


// FLASH_CR register
#define PG				0
#define SER				1
#define MER				2
#define SNB				3
#define PSIZE			8
#define STRT			16
#define EOPIE			24
#define FERRIE			25
#define LOCK			31

// FLASH_OPTCR register
#define OPTLOCK			0
#define OPTSTRT			1
#define BOR_LEV			2
#define WDG_SW			5
#define NRST_STOP		6
#define NRST_STDBY		7
#define RDP				8
#define NWRP			16
#define SPRMOD			31

#define OTAFUS	__attribute__((section(".ota_section")))

OTAFUS void flash_unlock(  void  );
OTAFUS void flash_lock(  void  );
OTAFUS void flash_erase(  void  );
OTAFUS void flash_write(  u32 addr, u16 data, u8 lock  );
OTAFUS void SPI1_transmit(  u8 data  );
OTAFUS u8 SPI1_receive(  void  );
OTAFUS void trigger_update(  void  );
OTAFUS void ota_memset(  void*, u8, u32  );

OTAFUS extern void hardware_reset(  void  );
OTAFUS extern void software_reset(  void  ); //deprecated
OTAFUS extern void disable_interrupt(  void  );
OTAFUS extern void enable_interrupt(  void  );

extern void copy_text(  void  );

#endif
