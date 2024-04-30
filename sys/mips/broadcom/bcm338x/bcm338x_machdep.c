/*-
 * Copyright (c) 2024, Hiroki Mori
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_ddb.h"
//#include "opt_bcm338x.h"

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cons.h>
#include <sys/kdb.h>
#include <sys/reboot.h>
#include <sys/boot.h>

#include <vm/vm.h>
#include <vm/vm_page.h>

#include <net/ethernet.h>

#include <machine/clock.h>
#include <machine/cpu.h>
#include <machine/cpuregs.h>
#include <machine/hwfunc.h>
#include <machine/md_var.h>
#include <machine/trap.h>
#include <machine/vmparam.h>

#include <mips/broadcom/bcm338x/bcm3383reg.h>

//#include <mips/broadcom/bcm338x/ar5315_setup.h>
//#include <mips/broadcom/bcm338x/ar5315_cpudef.h>

extern char edata[], end[];

int ar531x_soc;   // dummy

void
platform_cpu_init()
{
	/* Nothing special */
}

void
platform_reset(void)
{
//	bcm338x_device_reset();
	/* Wait for reset */
	while(1)
		;
}

void
platform_start(__register_t a0 __unused, __register_t a1 __unused, 
    __register_t a2 __unused, __register_t a3 __unused)
{
	uint64_t platform_counter_freq;
	int argc = 0, i;
	char **argv = NULL;
	vm_offset_t kernend;

	/* 
	 * clear the BSS and SBSS segments, this should be first call in
	 * the function
	 */
	kernend = (vm_offset_t)&end;
	memset(&edata, 0, kernend - (vm_offset_t)(&edata));

	mips_postboot_fixup();

	/* Initialize pcpu stuff */
	mips_pcpu0_init();

	/*
	 * Until some more sensible abstractions for uboot/redboot
	 * environment handling, we have to make this a compile-time
	 * hack.  The existing code handles the uboot environment
	 * very incorrectly so we should just ignore initialising
	 * the relevant pointers.
	 */
	/* 
	 * Protect ourselves from garbage in registers 
	 */
/*
	if (MIPS_IS_VALID_PTR(envp)) {
		for (i = 0; envp[i]; i += 2) {
			if (strcmp(envp[i], "memsize") == 0)
				realmem = btoc(strtoul(envp[i+1], NULL, 16));
		}
	}
*/

//	ar5315_detect_sys_type();

	/*
	 * Just wild guess. RedBoot let us down and didn't reported 
	 * memory size
	 */
	if (realmem == 0)
		realmem = btoc(128*1024*1024);

	/*
	 * Allow build-time override in case Redboot lies
	 * or in other situations (eg where there's u-boot)
	 * where there isn't (yet) a convienent method of
	 * being told how much RAM is available.
	 *
	 * This happens on at least the Ubiquiti LS-SR71A
	 * board, where redboot says there's 16mb of RAM
	 * but in fact there's 32mb.
	 */

	/* phys_avail regions are in bytes */
	phys_avail[0] = MIPS_KSEG0_TO_PHYS(kernel_kseg0_end);
	phys_avail[1] = ctob(realmem);

	dump_avail[0] = phys_avail[0];
	dump_avail[1] = phys_avail[1] - phys_avail[0];

	physmem = realmem;

	/*
	 * ns8250 uart code uses DELAY so ticker should be inititalized 
	 * before cninit. And tick_init_params refers to hz, so * init_param1 
	 * should be called first.
	 */
	init_param1();
	boothowto |= (RB_SERIAL | RB_MULTIPLE); /* Use multiple consoles */
//	boothowto |= RB_VERBOSE;
//	boothowto |= (RB_SINGLE);
	bootverbose = 1;

	/* Detect the system type - this is needed for subsequent chipset-specific calls */


//	bcm338x_device_soc_init();
//	bcm338x_detect_sys_frequency();

//	platform_counter_freq = bcm338x_cpu_freq();
	platform_counter_freq = 500 * 1000 * 1000;
	mips_timer_init_params(platform_counter_freq, 1);
	cninit();
/*
	init_static_kenv(boot1_env, sizeof(boot1_env));

	printf("CPU platform: %s\n", ar5315_get_system_type());
	printf("CPU Frequency=%d MHz\n", bcm338x_cpu_freq() / 1000000);
	printf("CPU DDR Frequency=%d MHz\n", bcm338x_ddr_freq() / 1000000);
	printf("CPU AHB Frequency=%d MHz\n", bcm338x_ahb_freq() / 1000000); 

	printf("platform frequency: %lld\n", platform_counter_freq);
	printf("arguments: \n");
	printf("  a0 = %08x\n", a0);
	printf("  a1 = %08x\n", a1);
	printf("  a2 = %08x\n", a2);
	printf("  a3 = %08x\n", a3);

	strcpy(cpu_model, ar5315_get_system_type());
*/

	init_param2(physmem);
	mips_cpu_init();
	pmap_bootstrap();
	mips_proc0_init();
	mutex_init();

//	bcm338x_device_start();

	kdb_init();
#ifdef KDB
	if (boothowto & RB_KDB)
		kdb_enter(KDB_WHY_BOOTFLAGS, "Boot flags requested debugger");
#endif

}

#if defined(EARLY_PRINTF)
static void
bcm_early_putc(int c)
{
	unsigned int *ptr;
	unsigned int tx_level;

	ptr = BCM3383_UART0_BASE + 0x08;
	while(1)
	{
		tx_level = *ptr;
		tx_level = tx_level >> 24;
		tx_level &= 0x1f;
		if(tx_level < 14)
			break;
	}

	ptr = BCM3383_UART0_BASE + 0x14;
	*ptr = c;

}
early_putc_t *early_putc = bcm_early_putc;
#endif

int
bcm_get_platform(void)
{
	return 0;
}

int
bcm_get_uart_rclk(int dummy)
{
	return 0;
}
