;@ Right now only the Noise generator from the VLM5030.

#ifdef __arm__

	.global vlm5030ClockNoise

.equ PFEED_SN,	0x0200			;@ Periodic Noise Feedback
.equ WFEED_SN,	0x0240			;@ White Noise Feedback

	.syntax unified
	.arm

	.section .itcm
	.align 2
;@----------------------------------------------------------------------------
vlm5030ClockNoise:
;@----------------------------------------------------------------------------
	ldr r0,lfsr10bit
	movs r0,r0,lsr#1
	eorcs r0,r0,#WFEED_SN
	str r0,lfsr10bit
	and r0,r0,#1

	bx lr
;@----------------------------------------------------------------------------

lfsr10bit:
	.long PFEED_SN

;@----------------------------------------------------------------------------
	.end
#endif // #ifdef __arm__
