/*	$OpenBSD: locore.S,v 1.18 1998/09/15 10:58:53 pefo Exp $	*/
/*-
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Digital Equipment Corporation and Ralph Campbell.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Copyright (C) 1989 Digital Equipment Corporation.
 * Permission to use, copy, modify, and distribute this software and
 * its documentation for any purpose and without fee is hereby granted,
 * provided that the above copyright notice appears in all copies.
 * Digital Equipment Corporation makes no representations about the
 * suitability of this software for any purpose.  It is provided "as is"
 * without express or implied warranty.
 *
 * from: Header: /sprite/src/kernel/mach/ds3100.md/RCS/loMem.s,
 *	v 1.1 89/07/11 17:55:04 nelson Exp  SPRITE (DECWRL)
 * from: Header: /sprite/src/kernel/mach/ds3100.md/RCS/machAsm.s,
 *	v 9.2 90/01/29 18:00:39 shirriff Exp  SPRITE (DECWRL)
 * from: Header: /sprite/src/kernel/vm/ds3100.md/vmPmaxAsm.s,
 *	v 1.1 89/07/10 14:27:41 nelson Exp  SPRITE (DECWRL)
 *
 *	from: @(#)locore.s	8.5 (Berkeley) 1/4/94
 *	JNPR: swtch.S,v 1.6.2.1 2007/09/10 10:36:50 girish
 * $FreeBSD$
 */

/*
 *	Contains code that is the first executed at boot time plus
 *	assembly language support routines.
 */

#include <sys/syscall.h>
#include <machine/asm.h>
#include <machine/cpu.h>
#include <machine/cpuregs.h>
#include <machine/regnum.h>
#include <machine/pte.h>
#include <machine/pcb.h>

#include "assym.inc"

	.set	noreorder			# Noreorder is default style!

/* DSP Support */

#ifdef MIPS_DSP_SUPPORT

LEAF(dsp_restore)
	.set	push
	.set	dsp
	mfc0	t1, MIPS_COP_0_STATUS		# Disable interrupts and
	or	t0, t1, MIPS_SR_COP_1_BIT | MIPS_SR_MX	#  enable dsp
	mtc0	t0, MIPS_COP_0_STATUS
	ehb
	lw	t2, U_PCB_DSPREGS + (SZFPREG * 0)(a0)
	mtlo	t2, $ac1
	lw	t2, U_PCB_DSPREGS + (SZFPREG * 1)(a0)
	mthi	t2, $ac1
	lw	t2, U_PCB_DSPREGS + (SZFPREG * 2)(a0)
	mtlo	t2, $ac2
	sw	t2, U_PCB_DSPREGS + (SZFPREG * 3)(a0)
	mthi	t2, $ac2
	lw	t2, U_PCB_DSPREGS + (SZFPREG * 4)(a0)
	mtlo	t2, $ac3
	lw	t2, U_PCB_DSPREGS + (SZFPREG * 5)(a0)
	mthi	t2, $ac3
	lw	t2, U_PCB_DSPREGS + (SZFPREG * 6)(a0)
	wrdsp	t2

	mtc0	t1, MIPS_COP_0_STATUS		# Restore the status register.
	ehb
	j	ra
	nop
	.set	pop
END(dsp_restore)

LEAF(dsp_save)
	.set	push
	.set	dsp
	mfc0	t1, MIPS_COP_0_STATUS		# Disable interrupts and
	or	t0, t1, MIPS_SR_COP_1_BIT | MIPS_SR_MX	#  enable dsp
	mtc0	t0, MIPS_COP_0_STATUS
	ehb
	mflo	t2, $ac1
	sw	t2, U_PCB_DSPREGS + (SZFPREG * 0)(a0)
	mfhi	t2, $ac1
	sw	t2, U_PCB_DSPREGS + (SZFPREG * 1)(a0)
	mflo	t2, $ac2
	sw	t2, U_PCB_DSPREGS + (SZFPREG * 2)(a0)
	mfhi	t2, $ac2
	sw	t2, U_PCB_DSPREGS + (SZFPREG * 3)(a0)
	mflo	t2, $ac3
	sw	t2, U_PCB_DSPREGS + (SZFPREG * 4)(a0)
	mfhi	t2, $ac3
	sw	t2, U_PCB_DSPREGS + (SZFPREG * 5)(a0)
	rddsp	t2
	sw	t2, U_PCB_DSPREGS + (SZFPREG * 6)(a0)

	mtc0	t1, MIPS_COP_0_STATUS		# Restore the status register.
	ehb
	j	ra
	nop
	.set	pop
END(dsp_save)

#endif
