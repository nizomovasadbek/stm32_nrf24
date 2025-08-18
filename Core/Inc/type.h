/*
 * type.h
 *
 *  Created on: Aug 7, 2025
 *      Author: Asato
 */

#ifndef INC_TYPE_H_
#define INC_TYPE_H_

typedef signed char i8;
typedef unsigned char u8;
typedef signed short i16;
typedef unsigned short u16;
typedef signed int i32;
typedef unsigned int u32;
typedef signed long long i64;
typedef unsigned long long u64;

typedef i8*	i8ptr_t;
typedef u8* u8ptr_t;
typedef i16* i16ptr_t;
typedef u16* u16ptr_t;
typedef i32* i32ptr_t;
typedef u32* u32ptr_t;
typedef i64* i64ptr_t;
typedef u64* u64ptr_t;
typedef char* string;

typedef u8 boolean;

#define true		1
#define false		0
#define null		( void* ) 0

#endif
