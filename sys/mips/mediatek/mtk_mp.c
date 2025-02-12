/*-
 * Copyright (c) 2025 Hiroki Mori
 * Copyright (c) 2016 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Portions of this software were developed by SRI International and the
 * University of Cambridge Computer Laboratory under DARPA/AFRL contract
 * FA8750-10-C-0237 ("CTSRD"), as part of the DARPA CRASH research programme.
 *
 * Portions of this software were developed by the University of Cambridge
 * Computer Laboratory as part of the CTSRD Project, with support from the
 * UK Higher Education Innovation Fund (HEIF).
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
#include <machine/hwfunc.h>
#include <machine/md_var.h>
#include <machine/smp.h>

void	_start(__register_t a0, __register_t a1,  __register_t a2,
	    __register_t a3);

#define	VPECONF0_VPA	(1 << 0)
#define	MVPCONTROL_VPC	(1 << 1)
#define	MVPCONF0_PVPE_SHIFT	10
#define	MVPCONF0_PVPE_MASK	(0xf << MVPCONF0_PVPE_SHIFT)
#define	TCSTATUS_A	(1 << 13)

unsigned malta_ap_boot = ~0;

#define	C_SW0		(1 << 8)
#define	C_SW1		(1 << 9)
#define	C_IRQ0		(1 << 10)
#define	C_IRQ1		(1 << 11)
#define	C_IRQ2		(1 << 12)
#define	C_IRQ3		(1 << 13)
#define	C_IRQ4		(1 << 14)
#define	C_IRQ5		(1 << 15)

void send_ipi(unsigned int);
void clear_ipi(unsigned int);

void
platform_ipi_send(int cpuid)
{

//	printf("=%d.%d,", platform_processor_id(), cpuid);

	send_ipi(cpuid + 60);
}

void
platform_ipi_clear(void)
{

//	printf("@");

	clear_ipi(platform_processor_id() + 60);
}

int
platform_ipi_hardintr_num(void)
{

	return (2);
}

int
platform_ipi_softintr_num(void)
{

	return (-1);
}

void
platform_init_ap(int cpuid)
{
	uint32_t clock_int_mask;
	uint32_t ipi_intr_mask;

	/*
	 * Unmask the clock and ipi interrupts.
	 */
	if (cpuid == 0) {
		ipi_intr_mask = hard_int_mask(platform_ipi_hardintr_num());
		clock_int_mask = hard_int_mask(5);
		clock_int_mask |= hard_int_mask(0);
		set_intr_mask(ipi_intr_mask | clock_int_mask);
	} else {
		ipi_intr_mask = hard_int_mask(platform_ipi_hardintr_num());
		clock_int_mask = hard_int_mask(1);
		set_intr_mask(ipi_intr_mask | clock_int_mask);
	}

	mips_wbflush();
}

void
platform_cpu_mask(cpuset_t *mask)
{
	uint32_t i, ncpus, reg;

	ncpus = smp_threads_per_core * mp_ncores;

	CPU_ZERO(mask);
	for (i = 0; i < ncpus; i++)
		CPU_SET(i, mask);
}

struct cpu_group *
platform_smp_topo(void)
{

	return (smp_topo_none());
}

#define CPULAUNCH	0x00000f00
#define LAUNCHSIZE	32

#define LAUNCH_PC	0
#define LAUNCH_FLAGS	7

#define LAUNCH_FGO	2

int
platform_start_ap(int cpuid)
{
	uint32_t *launch;
	uint32_t reg;

	launch = MIPS_PHYS_TO_KSEG0(CPULAUNCH + cpuid * LAUNCHSIZE);
	*(launch + LAUNCH_PC) = mpentry;
	wmb();
	reg = *(launch + LAUNCH_FLAGS);
	reg |= LAUNCH_FGO;
	*(launch + LAUNCH_FLAGS) = reg;
	wmb();

	return (0);
}
