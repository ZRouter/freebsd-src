/*-
 * Copyright (c) 2009-2010 Weongyo Jeong <weongyo@freebsd.org>
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
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/bus.h>

#include <machine/bus.h>

#include "bcm_dma.h"
#include "bcm_dma_ringvar.h"

#include <dev/bhnd/bhnd_debug.h>

static int	bcm_dma_create_tags(device_t dev, struct bcm_dma *dma);
static void	bcm_dma_destroy_tags(struct bcm_dma *dma);

void		bcm_dma_stop(struct bcm_dma *dma);
void		bcm_dma_free(struct bcm_dma *dma);

int
bcm_dma_attach(device_t dev, struct resource *res, struct bcm_dma *dma)
{
	int error;

	/* GMAC supports only 64bit mode */
	dma->dmatype = BCM_DMA_64BIT;
	error = bcm_dma_create_tags(dev, dma);
	if (error) {
		return (error);
	}

	dma->rx = bcm_dma_ring_alloc(dma, res, 0, 0, dma->dmatype);
	if (!dma->rx) {
		error = ENXIO;
		goto fail;
	}

	for (int i = 0; i < 4; i++) {
		dma->wme[i] = bcm_dma_ring_alloc(dma, res, i, 1, dma->dmatype);
		if (!dma->wme[i]) {
			/* TODO: add cleanup */
			error = ENXIO;
			goto fail;
		}
	}

//	dma->mcast = bcm_dma_ringsetup(mac, 4, 1, dma->dmatype);
//	if (!dma->mcast)
//		goto fail6;

	/* setup RX DMA channel. */
	bcm_dma_ring_load(dma->rx);
	/* setup TX DMA channels. */
	for (int i = 0; i < 4; i++) {
		bcm_dma_ring_load(dma->wme[i]);
	}

//	bcm_dma_ring_load(dma->mcast);

	return (0);
fail:
	bcm_dma_detach(dma);
	return (error);
}

void
bcm_dma_detach(struct bcm_dma *dma)
{
	bcm_dma_stop(dma);
	bcm_dma_free(dma);
	bcm_dma_destroy_tags(dma);
}

static int
bcm_dma_create_tags(device_t dev, struct bcm_dma *dma)
{
	int 		error;
	bus_addr_t 	lowaddr;

	if (dma->dmatype == BCM_DMA_32BIT)
		lowaddr = BUS_SPACE_MAXADDR_32BIT;
	else
		lowaddr = BUS_SPACE_MAXADDR;

	/*
	 * Create top level DMA tag
	 */
	error = bus_dma_tag_create(bus_get_dma_tag(dev),	/* parent */
		    BCM_DMA_ALIGN, 0,		/* alignment, bounds */
		    lowaddr,			/* lowaddr */
		    BUS_SPACE_MAXADDR,		/* highaddr */
		    NULL, NULL,			/* filter, filterarg */
		    BUS_SPACE_MAXSIZE,		/* maxsize */
		    BUS_SPACE_UNRESTRICTED,	/* nsegments */
		    BUS_SPACE_MAXSIZE,		/* maxsegsize */
		    0,				/* flags */
		    NULL, NULL,			/* lockfunc, lockarg */
		    &dma->parent_dtag);
	if (error) {
		BHND_ERROR_DEV(dev, "can't create parent DMA tag\n");
		return (error);
	}

	/*
	 * Create TX/RX mbuf DMA tag
	 */
	error = bus_dma_tag_create(dma->parent_dtag,
				1,
				0,
				BUS_SPACE_MAXADDR,
				BUS_SPACE_MAXADDR,
				NULL, NULL,
				MCLBYTES,
				1,
				BUS_SPACE_MAXSIZE_32BIT,
				0,
				NULL, NULL,
				&dma->rxbuf_dtag);
	if (error) {
		BHND_ERROR_DEV(dev, "can't create mbuf DMA tag\n");
		goto fail;
	}

	error = bus_dma_tag_create(dma->parent_dtag,
				1,
				0,
				BUS_SPACE_MAXADDR,
				BUS_SPACE_MAXADDR,
				NULL, NULL,
				MCLBYTES,
				BCM_TXRING_SLOTS,
				BUS_SPACE_MAXSIZE_32BIT,
				0,
				NULL, NULL,
				&dma->txbuf_dtag);
	if (error) {
		BHND_ERROR_DEV(dev, "can't create mbuf DMA tag\n");
		goto fail;
	}

	return (0);
fail:
	bcm_dma_destroy_tags(dma);
	return (error);
}

static void
bcm_dma_destroy_tags(struct bcm_dma *dma)
{
	printf("bcm_dma_destroy_tags: %p\n", dma);
	if(dma->txbuf_dtag != NULL)
		bus_dma_tag_destroy(dma->txbuf_dtag);
	if(dma->rxbuf_dtag != NULL)
		bus_dma_tag_destroy(dma->rxbuf_dtag);
	if(dma->parent_dtag != NULL)
		bus_dma_tag_destroy(dma->parent_dtag);
}

void
bcm_dma_stop(struct bcm_dma *dma)
{
	int	i;

	if (dma->rx != NULL)
		bcm_dma_ring_unload(dma->rx);

	for (i = 0; i < 4; i++)
		if (dma->wme[i] != NULL)
			bcm_dma_ring_unload(dma->wme[i]);

	if (dma->mcast != NULL)
		bcm_dma_ring_unload(dma->mcast);
}

void
bcm_dma_free(struct bcm_dma *dma)
{
	int	i;

	if (dma->rx != NULL)
		bcm_dma_ring_free(&dma->rx);

	for (i = 0; i < 4; i++)
		if (dma->wme[i] != NULL)
			bcm_dma_ring_free(&dma->wme[i]);

	if (dma->mcast != NULL)
		bcm_dma_ring_free(&dma->mcast);
}

void
bcm_dmamap_callback(void *arg, bus_dma_segment_t *seg, int nseg, int error)
{

	if (!error) {
		KASSERT(nseg == 1, ("too many segments(%d)\n", nseg));
		*((bus_addr_t *)arg) = seg->ds_addr;
	}
}

void
bcm_dmamap_callback_mbuf(void *arg, bus_dma_segment_t *seg, int nseg,
		 bus_size_t mapsz __unused, int error)
{

	bcm_dmamap_callback(arg, seg, nseg, error);
}

uint16_t
bcm_dma_base(int type, int controller_idx, int is_tx)
{
	uint16_t		 base;
	const uint16_t	 	*map;
	int			 size;

	static const uint16_t map64[] = {
		BCM_DMA64_BASE0,
		BCM_DMA64_BASE1,
		BCM_DMA64_BASE2,
		BCM_DMA64_BASE3,
		BCM_DMA64_BASE4,
		BCM_DMA64_BASE5,
	};

	static const uint16_t map32[] = {
		BCM_DMA32_BASE0,
		BCM_DMA32_BASE1,
		BCM_DMA32_BASE2,
		BCM_DMA32_BASE3,
		BCM_DMA32_BASE4,
		BCM_DMA32_BASE5,
	};

	if (type == BCM_DMA_64BIT) {
		size = N(map64);
		map = map64;
		base = (is_tx == 0) ? BCM_DMA64_BASE_RX_SHIFT : 0;
	} else {
		size = N(map32);
		map = map32;
		base = (is_tx == 0) ? BCM_DMA32_BASE_RX_SHIFT : 0;
	}

	/*
	 * Check if controller index belongs to mapXX
	 */
	KASSERT(controller_idx >= 0 && controller_idx < size,
	    ("%s:%d: controller index is out of band", __func__, __LINE__));

	base += map[controller_idx];
	return (base);
}
