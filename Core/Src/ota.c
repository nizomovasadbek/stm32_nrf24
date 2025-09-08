/*
 * ota.c
 *
 *  Created on: Aug 5, 2025
 *      Author: Asato
 */


#include "ota.h"

OTAFUS void SPI1_transmit(  u16 data  ) {

	while(  !(  kSPI1.SPI_SR & (  1 << TXE  )  )  );

	kSPI1.SPI_DR = data;

	while(  (  kSPI1.SPI_SR & (  1 << BSY  )  )  );

}

OTAFUS u16 SPI1_receive(  void  ) {

	u16 result;

	while(  !(  kSPI1.SPI_SR  &  (  1 << RXNE  )  )  );

	result = kSPI1.SPI_DR;

	return result;

}

OTAFUS void flash_unlock(  void  ) {

	while(  kFLASH.FLASH_SR & (  1 << FLASH_BUSY  )  );


	if(  kFLASH.FLASH_CR  &  (  1 << LOCK  )  ) {
		kFLASH.FLASHKEY = F_KEY1;
		kFLASH.FLASHKEY = F_KEY2;
	}

	kFLASH.FLASH_CR |= (  1 << PSIZE  );

}

OTAFUS void flash_lock( void ) {

	while( kFLASH.FLASH_SR & ( 1 << FLASH_BUSY ) );
	kFLASH.FLASH_CR |= ( 1 << LOCK );

}
/*
 * Any action before firmware trigger interrupt must be disabled
 */
OTAFUS void flash_erase() {

	flash_unlock();

	kFLASH.FLASH_CR &= ~( 1 << SER );
	kFLASH.FLASH_CR |= ( 1 << MER );
	kFLASH.FLASH_CR |= ( 1 << STRT );

	while( kFLASH.FLASH_SR & ( 1 << FLASH_BUSY ) );
	kFLASH.FLASH_CR &= ~( 1 << MER );

	flash_lock();

}

OTAFUS void flash_write( u32 addr, u16 data, u8 lock ) {

	if( lock )
		flash_unlock();

	kFLASH.FLASH_CR &= ~( 3 << PSIZE );
	kFLASH.FLASH_CR |= ( 1 << PSIZE );
	kFLASH.FLASH_CR |= ( 1 << PG );
	*(  (  u16*  ) addr  ) = data;
	kFLASH.FLASH_CR &= ~( 1 << PG );

	if( lock )
		flash_lock();

	while( kFLASH.FLASH_SR & ( 1 << FLASH_BUSY ) );

}


OTAFUS void ota_memset(  void* ptr, u8 value, u32 size  ) {

	while(  size--  ) {

		*(  (  u32ptr_t  ) ptr  ) = value;
		ptr++;

	}

}

OTAFUS void trigger_update() {

	disable_interrupt();

}
