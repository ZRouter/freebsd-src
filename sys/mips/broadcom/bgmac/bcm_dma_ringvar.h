/*-
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

#ifndef SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_RINGVAR_H_
#define SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_RINGVAR_H_

#include "bcm_dma_desc.h"
#include "bcm_dma.h"

#define	BCM_DMA_WRITE(ring, offset, value)				\
	bus_write_4(ring->res, ring->dr_base + offset, value)
#define	BCM_DMA_READ(ring, offset)					\
	bus_read_4(ring->res, ring->dr_base + offset)

struct bcm_dma_ring;

struct bcm_dma_ring {
	struct resource			*res;
	struct bcm_dma			*dma;
	const struct bcm_dma_ops	*dr_ops;
	struct bcm_dmadesc_meta		*dr_meta;
	void				*dr_txhdr_cache;
	bus_dma_tag_t			 dr_ring_dtag;
	bus_dmamap_t			 dr_spare_dmap; /* only for RX */
	bus_dmamap_t			 dr_ring_dmap;
	bus_addr_t			 dr_txring_paddr;
	void				*dr_ring_descbase;
	bus_addr_t			 dr_ring_dmabase;
	int				 dr_numslots;
	int				 dr_tail_slot;
	int				 dr_head_slot;
	uint32_t			 dr_frameoffset;
	uint16_t			 dr_rx_bufsize;
	uint16_t			 dr_base;
	int				 dr_index;
	uint8_t				 dr_is_tx; 	/* 1 - TX; 0 - RX */
	uint8_t				 dr_stop;
	int				 dr_type;

	void				(*getdesc)(struct bcm_dma_ring *,
					    int, struct bcm_dmadesc_generic **,
					    struct bcm_dmadesc_meta **);
	void				(*setdesc)(struct bcm_dma_ring *,
					    struct bcm_dmadesc_generic *,
					    bus_addr_t, uint16_t, int, int,
					    int);
	void				(*start_transfer)(struct bcm_dma_ring *,
					    int);
	void				(*suspend)(struct bcm_dma_ring *);
	void				(*resume)(struct bcm_dma_ring *);
	int				(*get_curslot)(struct bcm_dma_ring *);
	void				(*set_curslot)(struct bcm_dma_ring *,
					    int);
};

/*
 * Used in bcm_dma.c
 */
struct bcm_dma_ring *	bcm_dma_ring_alloc(struct bcm_dma *dma,
			    struct resource *res,
			    int ctl_index,
			    int for_tx,
			    int type);
void			bcm_dma_ring_free(struct bcm_dma_ring **dr);
void			bcm_dma_ring_load(struct bcm_dma_ring *dr);
int			bcm_dma_ring_unload(struct bcm_dma_ring *dr);

int			bcm_dma_ring_get_tail(struct bcm_dma_ring *dr);
int			bcm_dma_ring_get_head(struct bcm_dma_ring *dr);
int			bcm_dma_ring_get_nextslot(struct bcm_dma_ring *dr,
			    int slot);


#define	BGMAC_ASSERT_RING_LOCKED(ring) 		\
do {						\
	struct bgmac_softc	*sc;		\
	device_t		 dev;		\
	dev = rman_get_device((ring)->res);	\
	sc = device_get_softc(dev);		\
	BGMAC_ASSERT_LOCKED(sc);		\
}						\
while (0);

#endif /* SYS_DEV_BHND_CORES_BGMAC_BCM_DMA_RINGVAR_H_ */
