.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global hardware_reset
.global software_reset
.section .ota_section, "ax", %progbits

.type hardware_reset, %function
hardware_reset:

	ldr r0, =0xE000ED0C
	ldr r1, =0x05FA0004
	str r1, [r0]
	b .
.size hardware_reset, .-hardware_reset

.type software_reset, %function
software_reset:
	ldr r0, =_estack
	msr msp, r0
	ldr r2, =0x08000004
	bx r2
	b .
.size software_reset, .-software_reset
