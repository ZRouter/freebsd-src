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

#include "bgmac.h"

#include "bcm_dma_reg.h"
#include "bcm_dma_desc.h"
#include "bcm_dma_ops.h"
#include "bcm_dma_ringvar.h"

/*
 * Internal
 */

static int	bcm_dma_ring_alloc_dma(struct bcm_dma_ring *dr);
static void	bcm_dma_ring_free_memory(struct bcm_dma_ring *dr);
static void	bcm_dma_ring_free_descbufs(struct bcm_dma_ring *dr);

struct bcm_dma_ring *
bcm_dma_ring_alloc(struct bcm_dma *dma, struct resource *res, int ctl_id,
		int is_tx, int type)
{
	struct bcm_dma_ring		*dr;
	struct bcm_dmadesc_generic	*desc;
	struct bcm_dmadesc_meta		*mt;
	int				 error, i;
	device_t			 dev;

	dev = rman_get_device(res);

	CTR3(KTR_BGMAC, "%s: setup of ring: %s[%d]", device_get_nameunit(dev),
	    (is_tx) ? "TX" : "RX",  ctl_id);

	/*
	 * Allocate memory for new ring
	 */
	dr = malloc(sizeof(*dr), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (dr == NULL)
		return (dr);

	/*
	 * Allocate meta's for descriptors / slots
	 */
	dr->dr_numslots = (is_tx > 0) ? BCM_TXRING_SLOTS : BCM_RXRING_SLOTS;
	dr->dr_meta = malloc(dr->dr_numslots * sizeof(struct bcm_dmadesc_meta),
	    M_DEVBUF, M_NOWAIT | M_ZERO);
	if (dr->dr_meta == NULL)
		goto fail0;

	dr->dr_type = type;
	dr->res = res;
	dr->dma = dma;
	KASSERT(dr->res != NULL, ("panic"));
	dr->dr_base = bcm_dma_base(type, ctl_id, is_tx);
	dr->dr_index = ctl_id;

	dr->suspend = bcm_dma_suspend;
	dr->resume = bcm_dma_resume;

	if (type == BCM_DMA_64BIT) {
		dr->getdesc = bcm_dma_64_getdesc;
		dr->setdesc = bcm_dma_64_setdesc;
		dr->start_transfer = bcm_dma_64_start_transfer;
		dr->get_curslot = bcm_dma_64_get_curslot;
		dr->set_curslot = bcm_dma_64_set_curslot;
	} else {
		dr->getdesc = bcm_dma_32_getdesc;
		dr->setdesc = bcm_dma_32_setdesc;
		dr->start_transfer = bcm_dma_32_start_transfer;
		dr->get_curslot = bcm_dma_32_get_curslot;
		dr->set_curslot = bcm_dma_32_set_curslot;
	}

	if (is_tx) {
		dr->dr_is_tx = 1;
		dr->dr_head_slot = 0;
		dr->dr_tail_slot = 0;
	} else {
		KASSERT(dr->dr_index == 0, ("%s:%d: fail", __func__, __LINE__));
		dr->dr_rx_bufsize = MCLBYTES;
		dr->dr_frameoffset = BCM_FRAME_OFFSET;
	}

	error = bcm_dma_ring_alloc_dma(dr);
	if (error)
		goto fail1;

	if (is_tx) {
		/* Create TX buffer DMA maps */
		for (i = 0; i < dr->dr_numslots; i++) {
			dr->getdesc(dr, i, &desc, &mt);
			mt->mt_m = NULL;
			error = bus_dmamap_create(dma->txbuf_dtag, 0,
			    &mt->mt_dmap);
			if (error) {
				BHND_ERROR_DEV(dev,
				    "can't create TX buf DMA map");
				goto out; /* XXX wrong! */
			}
		}
	} else {
		error = bus_dmamap_create(dma->rxbuf_dtag, 0,
		    &dr->dr_spare_dmap);
		if (error) {
			BHND_ERROR_DEV(dev, "can't create RX buf DMA map");
			goto out;		/* XXX wrong! */
		}

		/* Create RX buffer DMA maps & mbufs */
		for (i = 0; i < dr->dr_numslots; i++) {
			dr->getdesc(dr, i, &desc, &mt);
			error = bus_dmamap_create(dma->rxbuf_dtag, 0,
			    &mt->mt_dmap);
			if (error) {
				BHND_ERROR_DEV(dev,
				    "can't create RX buf DMA map");
				goto out;	/* XXX wrong! */
			}
			error = bcm_dma_rx_newbuf(dr, desc, mt, 1);
			if (error) {
				BHND_ERROR_DEV(dev,
				    "failed to allocate RX buf");
				goto out;	/* XXX wrong! */
			}
		}

		bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap,
		    BUS_DMASYNC_PREWRITE);
	}

out:
	return (dr);
fail1:
	free(dr->dr_meta, M_DEVBUF);
fail0:
	free(dr, M_DEVBUF);
	return (NULL);
}

static int
bcm_dma_ring_alloc_dma(struct bcm_dma_ring *dr)
{
	struct bcm_dma		*dma;
	bus_dma_tag_t		*tagp;
	bus_dmamap_t		*mapp;
	void			**vaddrp;
	bus_addr_t		*paddrp;
	int			 error;
	device_t		 dev;

	dma = dr->dma;
	dev = rman_get_device(dr->res);

	BHND_DEBUG_DEV(dev, "allocate & load DMA descriptor"
	    " ring %s[%d]", (dr->dr_is_tx) ? "TX" : "RX", dr->dr_index);

	tagp = &dr->dr_ring_dtag;
	mapp = &dr->dr_ring_dmap;
	vaddrp = &dr->dr_ring_descbase;
	paddrp = &dr->dr_ring_dmabase;

	/*
	 * Create DMA tag for description ring
	 */
	error = bus_dma_tag_create(dma->parent_dtag,
			    BCM_DMA_ALIGN, 0,
			    BUS_SPACE_MAXADDR,
			    BUS_SPACE_MAXADDR,
			    NULL, NULL,
			    BCM_DMA_RINGMEMSIZE,
			    1,
			    BUS_SPACE_MAXSIZE_32BIT,
			    0,
			    NULL, NULL,
			    tagp);
	if (error) {
		BHND_ERROR_DEV(dev, "can't create TX ring DMA tag: TODO frees");
		return (error);
	}

	error = bus_dmamem_alloc(*tagp, vaddrp, BUS_DMA_WAITOK | BUS_DMA_ZERO,
			mapp);
	if (error) {
		BHND_ERROR_DEV(dev, "can't allocate DMA mem: TODO frees");
		return (error);
	}

	error = bus_dmamap_load(*tagp, *mapp, *vaddrp, BCM_DMA_RINGMEMSIZE,
	    bcm_dmamap_callback, paddrp, BUS_DMA_NOWAIT);
	if (error) {
		BHND_ERROR_DEV(dev, "can't load DMA mem: TODO free");
		return (error);
	}

	return (0);
}

void
bcm_dma_ring_free(struct bcm_dma_ring **dr)
{

	if (dr == NULL)
		return;

	bcm_dma_ring_free_descbufs(*dr);
	bcm_dma_ring_free_memory(*dr);
	free((*dr)->dr_meta, M_DEVBUF);
	free(*dr, M_DEVBUF);

	*dr = NULL;
}

static void
bcm_dma_ring_free_memory(struct bcm_dma_ring *dr)
{

	bus_dmamap_unload(dr->dr_ring_dtag, dr->dr_ring_dmap);
	bus_dmamem_free(dr->dr_ring_dtag, dr->dr_ring_descbase,
	    dr->dr_ring_dmap);
}

static void
bcm_dma_ring_free_descbufs(struct bcm_dma_ring *dr)
{
	struct bcm_dmadesc_generic 	*desc;
	struct bcm_dmadesc_meta 	*meta;
	bus_dma_tag_t			 tag;
	device_t			 dev;
	int				 i;

	if (!dr->dr_tail_slot)
		return;

	dev = rman_get_device(dr->res);
	tag = (dr->dr_is_tx) ? dr->dma->txbuf_dtag : dr->dma->rxbuf_dtag;

	for (i = 0; i < dr->dr_numslots; i++) {
		dr->getdesc(dr, i, &desc, &meta);

		if (meta->mt_m == NULL) {
			if (!dr->dr_is_tx)
				BHND_ERROR_DEV(dev, "RX has empty buffer");
			continue;
		}
		bus_dmamap_unload(tag, meta->mt_dmap);
		m_freem(meta->mt_m);
		meta->mt_m = NULL;
	}
}

void
bcm_dma_ring_load(struct bcm_dma_ring *dr)
{
	uint64_t	ring64;
	uint32_t 	addrext, ring32, value;
	uint32_t 	trans;
	device_t	dev;
	int		i;

	dev = rman_get_device(dr->res);
	trans = 0;

	dr->dr_head_slot = dr->dr_tail_slot = 0;

	if (dr->dr_type == BCM_DMA_64BIT) {
		ring64 = (uint64_t)(dr->dr_ring_dmabase);
		BHND_DEBUG_DEV(dev, "dr_ring_dmabase: 0x%jx", ring64);

		/* try to suspend DMA code */
		BCM_DMA_WRITE(dr, BCM_DMA_CTL, BCM_DMA_CTL_SUSPEND);
		i = 10;
		do {
			DELAY(30);
			value = BCM_DMA_READ(dr, BCM_DMA64_STATUS);
			value = (value && BCM_DMA64_STATE) >> BCM_DMA64_STATE_SHIFT;
		} while (value != BCM_DMA_STAT_DISABLED &&
			 value != BCM_DMA_STAT_IDLEWAIT &&
			 value != BCM_DMA_STAT_STOPPED && --i);
		BHND_DEBUG_DEV(dev, "dr_ring_dmabase: susp %i %x", i, value);
		/* try to off DMA code */
		BCM_DMA_WRITE(dr, BCM_DMA_CTL, 0);
		i = 10;
		do {
			DELAY(300);
			value = BCM_DMA_READ(dr, BCM_DMA64_STATUS);
			value = (value && BCM_DMA64_STATE) >> BCM_DMA64_STATE_SHIFT;
		} while (value != BCM_DMA_STAT_DISABLED && --i);
		BHND_DEBUG_DEV(dev, "dr_ring_dmabase: off %i %x", i, value);

		addrext = ((ring64 >> 32) & BCM_DMA_ADDR_MASK) >> 30;
		value = (dr->dr_is_tx) ? 0 :
			    BCM_DMA_SHIFT(dr->dr_frameoffset, BCM_DMA_CTL_RXFROFF);
		value |= BCM_DMA_CTL_ENABLE;
		value |= BCM_DMA_CTL_PARITYDISABLE;
		value |= BCM_DMA_CTL_OVERFLOWCONTINUE;
		value |= BCM_DMA_SHIFT(addrext, BCM_DMA_CTL_ADDREXT);
		BHND_DEBUG_DEV(dev, "CTL: 0x%x->0x%x", dr->dr_base + BCM_DMA_CTL,
		    value);
		BCM_DMA_WRITE(dr, BCM_DMA_CTL, value);

		BCM_DMA_WRITE(dr, BCM_DMA64_RINGLO, (ring64 & 0xffffffff));
		BCM_DMA_WRITE(dr, BCM_DMA64_RINGHI, ((ring64 >> 32) &
		    ~BCM_DMA_ADDR_MASK) | (trans << 1));

		if (!dr->dr_is_tx)
			BCM_DMA_WRITE(dr, BCM_DMA64_INDEX, dr->dr_numslots *
			    sizeof(struct bcm_dmadesc64));

	} else {
		ring32 = (uint32_t)(dr->dr_ring_dmabase);
		addrext = (ring32 & BCM_DMA_ADDR_MASK) >> 30;

		BCM_DMA_WRITE(dr, BCM_DMA_CTL, 0);
		DELAY(30);

		value = (dr->dr_is_tx) ? 0 :
			    (dr->dr_frameoffset << BCM_DMA_CTL_RXFROFF_SHIFT);
		value |= BCM_DMA_CTL_ENABLE;
		value |= BCM_DMA_CTL_PARITYDISABLE;
		value |= BCM_DMA_SHIFT(addrext, BCM_DMA_CTL_ADDREXT);
		BCM_DMA_WRITE(dr, BCM_DMA_CTL, value);

		BCM_DMA_WRITE(dr, BCM_DMA32_RING,
		    (ring32 & ~BCM_DMA_ADDR_MASK) | trans);
		if (!dr->dr_is_tx)
			BCM_DMA_WRITE(dr, BCM_DMA32_INDEX, dr->dr_numslots *
			    sizeof(struct bcm_dmadesc32));
	}
}

int
bcm_dma_ring_unload(struct bcm_dma_ring *dr)
{
	uint32_t	value;
	uint16_t	offset;
	int		i, type;

	if (dr == NULL)
		return (EINVAL);

	type = dr->dr_type;

	offset = (type == BCM_DMA_64BIT) ? BCM_DMA64_STATUS : BCM_DMA32_STATUS;

	if (dr->dr_is_tx) {
		/* Try to wait end of transit */
		i = BCM_DMA_RETRY_COUNT;
		for (; i > 0; i--) {
			value = BCM_DMA_READ(dr, offset);
			if (type == BCM_DMA_64BIT) {
				value &= BCM_DMA64_STATE;
				value >>= BCM_DMA64_STATE_SHIFT;
			} else {
				value &= BCM_DMA32_STATE;
				value >>= BCM_DMA32_STATE_SHIFT;
			}

			if (value == BCM_DMA_STAT_DISABLED ||
			    value == BCM_DMA_STAT_IDLEWAIT ||
			    value == BCM_DMA_STAT_STOPPED)
				break;
			DELAY(1000);
		}
	}

	/* Disable ring */
	BCM_DMA_WRITE(dr, BCM_DMA_CTL, 0);

	i = BCM_DMA_RETRY_COUNT;
	for (; i > 0; i--) {
		value = BCM_DMA_READ(dr, offset);
		if (type == BCM_DMA_64BIT) {
			value &= BCM_DMA64_STATE;
			value >>= BCM_DMA64_STATE_SHIFT;
		} else {
			value &= BCM_DMA32_STATE;
			value >>= BCM_DMA32_STATE_SHIFT;
		}

		if (value == BCM_DMA_STAT_DISABLED){
			if (type == BCM_DMA_64BIT) {
				BCM_DMA_WRITE(dr, BCM_DMA64_RINGLO, 0);
				BCM_DMA_WRITE(dr, BCM_DMA64_RINGHI, 0);
			} else
				BCM_DMA_WRITE(dr, BCM_DMA32_RING, 0);
			return (0);
		}

		DELAY(1000);
	}

	BHND_ERROR_DEV(rman_get_device(dr->res), "%s: timed out", __func__);
	return (ENODEV);
}

int
bcm_dma_ring_get_tail(struct bcm_dma_ring *dr)
{

	BGMAC_ASSERT_RING_LOCKED(dr);
	return (dr->dr_tail_slot);
}

int
bcm_dma_ring_get_head(struct bcm_dma_ring *dr)
{

	BGMAC_ASSERT_RING_LOCKED(dr);
	return (dr->dr_head_slot);
}

int
bcm_dma_ring_get_nextslot(struct bcm_dma_ring *dr, int slot)
{

	BGMAC_ASSERT_RING_LOCKED(dr);
	KASSERT(slot >= -1 && slot <= dr->dr_numslots - 1,
	    ("%s:%d: fail", __func__, __LINE__));
	if (slot == dr->dr_numslots - 1)
		return (0);
	return (slot + 1);
}
