/*
 *
 * Copyright (C) STMicroelectronics SA 2017
 * Author(s): Olivier Bideau <olivier.bideau@st.com> for STMicroelectronics.
 *
 * License terms: GPL V2.0.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 */

#include <linux/linkage.h>
#include <asm/cp15.h>

ENTRY(stm32mp1_suspend_exec)
	push	{r4 - r11, lr}
	ldr	r6, =stm32mp1_suspend_on_eram

 THUMB( orr     r6, r6, #1 )
	blx	r6
	pop	{r4 - r11, pc}
ENDPROC(stm32mp1_suspend_exec)
.balign	4

ENTRY(stm32mp1_suspend_on_eram)
        mov     r6, lr          @ save lr on r6
        wfi
        mov     pc, r6          @ return
ENDPROC(stm32mp1_suspend_on_eram)

/*
 * stm32mp1_defrost_kernel is where cpu0 starts executing after standby wakeup
 * this only happens in case the DDR was maintained in self refresh
 */
ENTRY(stm32mp1_defrost_kernel)
	adr	r3, 1f		@r3 => Current PA
	ldmia	r3, {r4, r8}	@r4 => Current VA, @r8 => VA of cpu_resume
	sub	r4, r3, r4	@r4 => Diff of PA - VA
	add	r8, r8, r4	@r8 => PA of cpu_resume

	bx 	r8
ENDPROC(stm32mp1_defrost_kernel)
	.ltorg
1:	.long .			@this word contains the current VA
	.long cpu_resume	@this word contains the VA of cpu_resume
