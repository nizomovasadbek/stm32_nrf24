.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global software_reset
.global hardware_reset
.section .ota_section

software_reset:

	ldr r0, =0x08000000
	ldr r1, =_estack
	msr msp, r1

	ldr r2, [r0, #4]
	bx r2


hardware_reset:

	ldr r0, =0xE000ED0C
	ldr r1, =0x05FA0004
	str r1, [r0]

hardware_reset_loop:
	b hardware_reset_loop
