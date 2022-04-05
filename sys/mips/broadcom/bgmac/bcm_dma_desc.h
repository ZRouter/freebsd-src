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

#ifndef _BCM_DMA_DESC_H_
#define _BCM_DMA_DESC_H_

struct bcm_dmadesc_meta {
	bus_dmamap_t			mt_dmap;
	bus_addr_t			mt_paddr;
	struct mbuf			*mt_m;
};

#define	BCM_DMA32_DCTL_BYTECNT		0x00001fff
#define	BCM_DMA32_DCTL_ADDREXT_MASK	0x00030000
#define	BCM_DMA32_DCTL_ADDREXT_SHIFT	16
#define	BCM_DMA32_DCTL_DTABLEEND	0x10000000
#define	BCM_DMA32_DCTL_IRQ		0x20000000
#define	BCM_DMA32_DCTL_FRAMEEND		0x40000000
#define	BCM_DMA32_DCTL_FRAMESTART	0x80000000
struct bcm_dmadesc32 {
	uint32_t			control;
	uint32_t			address;
} __packed;

#define	BCM_DMA64_DCTL0_DTABLEEND	0x10000000
#define	BCM_DMA64_DCTL0_IRQ		0x20000000
#define	BCM_DMA64_DCTL0_FRAMEEND	0x40000000
#define	BCM_DMA64_DCTL0_FRAMESTART	0x80000000
#define	BCM_DMA64_DCTL1_BYTECNT		0x00001fff
#define	BCM_DMA64_DCTL1_ADDREXT_MASK	0x00030000
#define	BCM_DMA64_DCTL1_ADDREXT_SHIFT	16
struct bcm_dmadesc64 {
	uint32_t			control0;
	uint32_t			control1;
	uint32_t			address_low;
	uint32_t			address_high;
} __packed;

struct bcm_dmadesc_generic {
	union {
		struct bcm_dmadesc32 dma32;
		struct bcm_dmadesc64 dma64;
	} __packed dma;
} __packed;

#define	BCM_DMADESC_DUMP(_desc)							\
	BHND_DEBUG("\tctl0=0x%x\n\tctl1=0x%x\n\taddrlow=0x%x\n\taddrhi=0x%x\n", \
	    (_desc)->dma.dma64.control0, (_desc)->dma.dma64.control1,		\
	    (_desc)->dma.dma64.address_low, (_desc)->dma.dma64.address_high);

#endif /* _BCM_DMA_DESC_H_ */
