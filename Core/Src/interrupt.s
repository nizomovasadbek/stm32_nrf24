.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global enable_interrupt
.global disable_interrupt
.section .ota_section, "ax", %progbits
.type enable_interrupt, %function
enable_interrupt:

	cpsie i
	bx lr
.size enable_interrupt, .-enable_interrupt

.type disable_interrupt, %function
disable_interrupt:

	cpsid i
	bx lr
.size disable_interrupt, .-disable_interrupt
