/*
 * ota.c
 *
 *  Created on: Aug 5, 2025
 *      Author: Asato
 */


#include "ota.h"

OTAFUS void SPI1_transmit( u8 data ) {

	while( !( kSPI1.SPI_SR & ( 1 << TXE ) ) );

	kSPI1.SPI_DR = data;

	while( !( kSPI1.SPI_SR & ( 1 << TXE ) ) );

}

OTAFUS u8 SPI1_receive( void ) {

	u8 result;
	while( !( kSPI1.SPI_SR & ( 1 << RXNE ) ) );

	result = kSPI1.SPI_DR;

	return result;

}

OTAFUS void flash_unlock( void ) {

	while( kFLASH.FLASH_SR & ( 1 << FLASH_BUSY ) );


	if( kFLASH.FLASH_CR & ( 1 << LOCK ) ) {
		kFLASH.FLASHKEY = F_KEY1;
		kFLASH.FLASHKEY = F_KEY2;
	}

	kFLASH.FLASH_CR |= ( 1 << PSIZE );

}

OTAFUS void flash_lock( void ) {

	while( kFLASH.FLASH_SR & ( 1 << FLASH_BUSY ) );
	kFLASH.FLASH_CR |= ( 1 << LOCK );

}

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
	*( ( u16* ) addr ) = data;
	kFLASH.FLASH_CR &= ~( 1 << PG );

	if( lock )
		flash_lock();

	while( kFLASH.FLASH_SR & ( 1 << FLASH_BUSY ) );

}

OTAFUS void trigger_update() {
}
