.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global copy_text
.section .text

copy_text:

	push {r0, r1, r2, r3}

	ldr r0, =_ota_loadaddr
	ldr r1, =_ota_begin
	ldr r2, =_ota_end
	movs r3, #0

copy_text_loop:
	ldr r3, [r0]
	str r3, [r1]
	add r0, r0, #4
	add r1, r1, #4
	cmp r1, r2
	blt copy_text_loop

	pop {r0, r1, r2, r3}
	bx lr

.size copy_text, .-copy_text
