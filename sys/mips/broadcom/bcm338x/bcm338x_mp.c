/*-
 * Copyright (c) 2015 Alexander Kabaev <kan@FreeBSD.org>
 * Copyright (c) 2004-2010 Juli Mallett <jmallett@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/smp.h>
#include <sys/systm.h>

#include <machine/cpufunc.h>
#include <machine/cpuregs.h>
#include <machine/hwfunc.h>
#include <machine/md_var.h>
#include <machine/smp.h>

#include <mips/broadcom/bcm338x/bcm3383reg.h>

#define BCM338X_MAXCPU	2

void bcm338x_mpentry(void);

/* prototype is ../include/hwfunc.h */

void
platform_ipi_send(int cpuid)
{
	uint32_t reg;

//	printf("=%d.%d,", platform_processor_id(), cpuid);

	reg = mips_rd_cause();
	if(cpuid == 1)
		reg |= MIPS_SOFT_INT_MASK_0;
	else
		reg |= MIPS_SOFT_INT_MASK_1;
	mips_wr_cause(reg);
}

void
platform_ipi_clear(void)
{
	uint32_t reg;

//	printf("@%d,", platform_processor_id());

	reg = mips_rd_cause();
	if(platform_processor_id() == 1)
		reg &= ~(MIPS_SOFT_INT_MASK_0);
	else
		reg &= ~(MIPS_SOFT_INT_MASK_1);
	mips_wr_cause(reg);
}

int
platform_processor_id(void)
{

	return (read_c0_brcm_cmt_local() & (1 << 31) ? 1 : 0);
}

int
platform_ipi_hardintr_num(void)
{

	return (-1);
}

int
platform_ipi_softintr_num(void)
{

	/* use 0 and 1 software interrupt */

	return (2);
}

/* call from mpentry in mips/mpboot.S */

void
platform_init_ap(int cpuid)
{
	uint32_t clock_int_mask;
	uint32_t ipi_intr_mask;
	int i;

	/*
	 * Unmask the ipi interrupts.
	 */
	ipi_intr_mask = soft_int_mask(0) | soft_int_mask(1);
	clock_int_mask = hard_int_mask(5);
	set_intr_mask(ipi_intr_mask | clock_int_mask);
}

void
platform_cpu_mask(cpuset_t *mask)
{
	uint32_t i, m;

	CPU_ZERO(mask);
	for (i = 0, m = 1 ; i < BCM338X_MAXCPU; i++, m <<= 1)
		CPU_SET(i, mask);
}

struct cpu_group *
platform_smp_topo(void)
{
	return (smp_topo_none());
}

/*
 * Spin up the second code. The code is roughly modeled after
 * similar routine in Linux.
 */
int
platform_start_ap(int cpuid)
{
	uint32_t reg;
	int i;

	memcpy((void *)0xa0000200, &bcm338x_mpentry, 0x10);

	reg = mips_rd_cause();
	reg |= MIPS_SOFT_INT_MASK_0;
	mips_wr_cause(reg);

	return (0);
}
