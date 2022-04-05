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

#ifndef _BCM_DMA_OPS_H_
#define _BCM_DMA_OPS_H_

struct bcm_dma_ring;

void	bcm_dma_suspend(struct bcm_dma_ring *);
void	bcm_dma_resume(struct bcm_dma_ring *);

void	bcm_dma_32_getdesc(struct bcm_dma_ring *, int,
	    struct bcm_dmadesc_generic **, struct bcm_dmadesc_meta **);
void	bcm_dma_32_setdesc(struct bcm_dma_ring *, struct bcm_dmadesc_generic *,
	    bus_addr_t, uint16_t, int, int, int);

void	bcm_dma_32_start_transfer(struct bcm_dma_ring *, int);
void	bcm_dma_32_resume(struct bcm_dma_ring *);
int	bcm_dma_32_get_curslot(struct bcm_dma_ring *);
void	bcm_dma_32_set_curslot(struct bcm_dma_ring *, int);

void	bcm_dma_64_getdesc(struct bcm_dma_ring *,
		    int, struct bcm_dmadesc_generic **,
		    struct bcm_dmadesc_meta **);
void	bcm_dma_64_setdesc(struct bcm_dma_ring *,
		    struct bcm_dmadesc_generic *, bus_addr_t, uint16_t, int,
		    int, int);
void	bcm_dma_64_start_transfer(struct bcm_dma_ring *, int);
int	bcm_dma_64_get_curslot(struct bcm_dma_ring *);
void	bcm_dma_64_set_curslot(struct bcm_dma_ring *, int);

#endif /* _BCM_DMA_OPS_H_ */
