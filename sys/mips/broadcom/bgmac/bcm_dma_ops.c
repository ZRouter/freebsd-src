/*-
 * Copyright (c) 2009-2010 Weongyo Jeong <weongyo@freebsd.org>
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include "bcm_dma_desc.h"
#include "bcm_dma_reg.h"
#include "bcm_dma_ops.h"
#include "bcm_dma_ringvar.h"

/*
 * Generic functions
 */

void
bcm_dma_suspend(struct bcm_dma_ring *dr)
{

	BCM_DMA_WRITE(dr, BCM_DMA_CTL,
	    BCM_DMA_READ(dr, BCM_DMA_CTL) | BCM_DMA_CTL_SUSPEND);
}

void
bcm_dma_resume(struct bcm_dma_ring *dr)
{

	BCM_DMA_WRITE(dr, BCM_DMA_CTL,
	    BCM_DMA_READ(dr, BCM_DMA_CTL) & ~BCM_DMA_CTL_SUSPEND);
}

/*
 * 32-bit functions
 */

void
bcm_dma_32_getdesc(struct bcm_dma_ring *dr, int slot,
    struct bcm_dmadesc_generic **gdesc, struct bcm_dmadesc_meta **meta)
{
	struct bcm_dmadesc32 *desc;

	*meta = &(dr->dr_meta[slot]);
	desc = dr->dr_ring_descbase;
	desc = &(desc[slot]);

	*gdesc = (struct bcm_dmadesc_generic *)desc;
}

void
bcm_dma_32_setdesc(struct bcm_dma_ring *dr,
    struct bcm_dmadesc_generic *desc, bus_addr_t dmaaddr, uint16_t bufsize,
    int start, int end, int irq)
{
	struct bcm_dmadesc32 *descbase = dr->dr_ring_descbase;
	uint32_t addr, addrext, ctl;
	int slot;

	slot = (int)(&(desc->dma.dma32) - descbase);
	KASSERT(slot >= 0 && slot < dr->dr_numslots,
	    ("%s:%d: fail", __func__, __LINE__));

	addr = (uint32_t) (dmaaddr & ~BCM_DMA_ADDR_MASK);
	addrext = (uint32_t) (dmaaddr & BCM_DMA_ADDR_MASK) >> 30;
	addr |= 0x40000000;
	ctl = bufsize & BCM_DMA32_DCTL_BYTECNT;
	if (slot == dr->dr_numslots - 1)
		ctl |= BCM_DMA32_DCTL_DTABLEEND;
	if (start)
		ctl |= BCM_DMA32_DCTL_FRAMESTART;
	if (end)
		ctl |= BCM_DMA32_DCTL_FRAMEEND;
	if (irq)
		ctl |= BCM_DMA32_DCTL_IRQ;
	ctl |= (addrext << BCM_DMA32_DCTL_ADDREXT_SHIFT)
	    & BCM_DMA32_DCTL_ADDREXT_MASK;

	desc->dma.dma32.control = htole32(ctl);
	desc->dma.dma32.address = htole32(addr);
}

void
bcm_dma_32_start_transfer(struct bcm_dma_ring *dr, int slot)
{

	BCM_DMA_WRITE(dr, BCM_DMA32_INDEX,
	    (uint32_t)(slot * sizeof(struct bcm_dmadesc32)));
}

int
bcm_dma_32_get_curslot(struct bcm_dma_ring *dr)
{
	uint32_t val;

	val = BCM_DMA_READ(dr, BCM_DMA32_STATUS);
	val &= BCM_DMA32_RXDPTR;

	return (val / sizeof(struct bcm_dmadesc32));
}

void
bcm_dma_32_set_curslot(struct bcm_dma_ring *dr, int slot)
{

	BCM_DMA_WRITE(dr, BCM_DMA32_INDEX,
	    (uint32_t) (slot * sizeof(struct bcm_dmadesc32)));
}

/**********************************************************************
 * 	64-bit DMA operations
 **********************************************************************/

void
bcm_dma_64_getdesc(struct bcm_dma_ring *dr, int slot,
    struct bcm_dmadesc_generic **gdesc, struct bcm_dmadesc_meta **meta)
{
	struct bcm_dmadesc64 *desc;

	desc = dr->dr_ring_descbase;
	desc = &(desc[slot]);
	*meta = &(dr->dr_meta[slot]);
	*gdesc = (struct bcm_dmadesc_generic *)desc;
}

void
bcm_dma_64_setdesc(struct bcm_dma_ring *dr, struct bcm_dmadesc_generic *desc,
    bus_addr_t dmaaddr, uint16_t bufsize, int start, int end, int irq)
{
	struct bcm_dmadesc64	*descbase;
	int 			 slot;
	uint32_t 		 ctl0, ctl1;
	uint32_t		 addrlo, addrhi, addrext;

	descbase = dr->dr_ring_descbase;
	ctl0 = ctl1 = 0;

	slot = (int)(&(desc->dma.dma64) - descbase);
	KASSERT(slot >= 0 && slot < dr->dr_numslots,
	    ("%s:%d: fail on slot = %d", __func__, __LINE__, slot));

	addrlo = (uint32_t) (dmaaddr & 0xffffffff);
	addrhi = (((uint64_t) dmaaddr >> 32) & ~BCM_DMA_ADDR_MASK);
	addrext = (((uint64_t) dmaaddr >> 32) & BCM_DMA_ADDR_MASK) >> 30;

	/* XXX: why we need it? */
	addrhi |= (0x40000000 << 1);

	if (slot == dr->dr_numslots - 1)
		ctl0 |= BCM_DMA64_DCTL0_DTABLEEND;
	if (start)
		ctl0 |= BCM_DMA64_DCTL0_FRAMESTART;
	if (end)
		ctl0 |= BCM_DMA64_DCTL0_FRAMEEND;
	if (irq)
		ctl0 |= BCM_DMA64_DCTL0_IRQ;
	ctl1 |= bufsize & BCM_DMA64_DCTL1_BYTECNT;
	ctl1 |= (addrext << BCM_DMA64_DCTL1_ADDREXT_SHIFT)
	    & BCM_DMA64_DCTL1_ADDREXT_MASK;

	desc->dma.dma64.control0 = htole32(ctl0);
	desc->dma.dma64.control1 = htole32(ctl1);
	desc->dma.dma64.address_low = htole32(addrlo);
	desc->dma.dma64.address_high = htole32(addrhi);
}

void
bcm_dma_64_start_transfer(struct bcm_dma_ring *dr, int slot)
{

	BCM_DMA_WRITE(dr, BCM_DMA64_INDEX,
	    (uint32_t)(slot * sizeof(struct bcm_dmadesc64)));
}

int
bcm_dma_64_get_curslot(struct bcm_dma_ring *dr)
{
	uint32_t val;

	val = BCM_DMA_READ(dr, BCM_DMA64_STATUS);
	val &= BCM_DMA64_RXSTATDPTR;

	return (val / sizeof(struct bcm_dmadesc64));
}

void
bcm_dma_64_set_curslot(struct bcm_dma_ring *dr, int slot)
{

	BCM_DMA_WRITE(dr, BCM_DMA64_INDEX,
	    (uint32_t)(slot * sizeof(struct bcm_dmadesc64)));
}
