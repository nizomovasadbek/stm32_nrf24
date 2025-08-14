.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.section .ota_function, "ax", %progbits
.global hardware_reset
.type hardware_reset, %function
hardware_reset:

	ldr r0, =0xE000ED0C
	ldr r1, =0x05FA0004
	str r1, [r0]
	b .

.type software_reset, %function


software_reset:
	ldr r0, =0x08000000
	ldr r1, =_estack
	msr msp, r1

	ldr r2, [r0, #4]
	bx r2
	b .
.size software_reset, .-software_reset
