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
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/errno.h>
#include <sys/endian.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include "bcm_dma_reg.h"
#include "bcm_dma_desc.h"
#include "bcm_dma_ops.h"
#include "bcm_dma_ringvar.h"

#define	BHND_LOGGING	BHND_INFO_LEVEL
#include <dev/bhnd/bhnd_debug.h>

static void	bcm_dma_rxeof(struct bcm_dma_ring *dr, int *slot);
static uint8_t	bcm_dma_check_redzone(struct bcm_dma_ring *dr, struct mbuf *m);
static void	bcm_dma_set_redzone(struct bcm_dma_ring *dr, struct mbuf *m);

void
bcm_dma_rx(struct bcm_dma_ring *dr)
{
	int	slot;
	int	curslot;

	/* Check if ring is RX */
	KASSERT(!dr->dr_is_tx, ("%s:%d: fail", __func__, __LINE__));

	curslot = dr->get_curslot(dr);
	KASSERT(curslot >= 0 && curslot < dr->dr_numslots,
	    ("%s:%d: curslot[%d] is out of range", __func__, __LINE__, curslot));

	slot = dr->dr_head_slot;
	for (; slot != curslot; slot = bcm_dma_ring_get_nextslot(dr, slot)) {
		BHND_DEBUG_DEV(rman_get_device(dr-res),
		    "rxeof: head=%d, curslot=%d", slot, curslot);
		bcm_dma_rxeof(dr, &slot);
	}
	bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap, BUS_DMASYNC_PREWRITE);

	dr->set_curslot(dr, slot);
	dr->dr_head_slot = slot;
}

static void
bcm_dma_rxeof(struct bcm_dma_ring *dr, int *slot)
{
	struct bcm_dma			*dma;
	struct bcm_dmadesc_generic 	*desc;
	struct bcm_dmadesc_meta 	*meta;
	struct bcm_rx_header		*rxhdr;
	struct mbuf 			*m;
	device_t			 dev;
	int32_t 			 tmp;
	int 				 cnt = 0;
	uint16_t 			 len;

	dev = rman_get_device(dr->res);
	dma = dr->dma;
	dr->getdesc(dr, *slot, &desc, &meta);
	bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap, BUS_DMASYNC_POSTREAD);
	m = meta->mt_m;

	/*
	 * Create new buffer and put it into ring instead of dirty
	 */
	if (bcm_dma_rx_newbuf(dr, desc, meta, 0)) {
		/* TODO: add counters */
#if 0
		counter_u64_add(sc->sc_ic.ic_ierrors, 1);
#endif
		BHND_ERROR_DEV(dev, "error on bcm_dma_rx_newbuf");
		return;
	}

	/*
	 * Process dirty received buffer
	 */
	rxhdr = mtod(m, struct bcm_rx_header *);
	len = le16toh(rxhdr->len);
	len -= 4; /* remove checksum */
#if 0
	BHND_INFO_DEV(dev, "len: %d", len);
#define HEXDUMP(_buf, _len) do { \
  { \
        size_t __tmp; \
        const char *__buf = (const char *)_buf; \
        for (__tmp = 0; __tmp < _len; __tmp++) \
                printf("%02hhx ", *__buf++); \
    printf("\n"); \
  } \
} while(0)

	HEXDUMP(rxhdr,0x5DC);
#undef HEXDUMP
#endif
	if (len <= 0) {
		/* TODO: add counters */
#if 0
		counter_u64_add(sc->sc_ic.ic_ierrors, 1);
#endif
		BHND_ERROR_DEV(dev, "len < 0");
		return;
	}

	if (bcm_dma_check_redzone(dr, m)) {
		BHND_ERROR_DEV(dev, "redzone error");
		bcm_dma_set_redzone(dr, m);
		bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap,
		    BUS_DMASYNC_PREWRITE);
		return;
	}

	if (len > dr->dr_rx_bufsize) {
		tmp = len;
		for (;;) {
			dr->getdesc(dr, *slot, &desc, &meta);
			bcm_dma_set_redzone(dr, meta->mt_m);
			bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap,
			    BUS_DMASYNC_PREWRITE);
			*slot = bcm_dma_ring_get_nextslot(dr, *slot);
			cnt++;
			tmp -= dr->dr_rx_bufsize;
			if (tmp <= 0)
				break;
		}
		BHND_WARN_DEV(dev, "too small buffer (len %u buf %u drop %d)\n",
		       len, dr->dr_rx_bufsize, cnt);
		return;
	}

	m->m_len = m->m_pkthdr.len = len + dr->dr_frameoffset;
	m_adj(m, dr->dr_frameoffset);

	/*
	 * Callback to MAC level rxeof
	 */
	bgmac_rxeof(rman_get_device(dr->res), m, rxhdr);
}


int
bcm_dma_rx_newbuf(struct bcm_dma_ring *dr, struct bcm_dmadesc_generic *desc,
    struct bcm_dmadesc_meta *meta, int init)
{
	struct bcm_dma		*dma;
	struct bcm_rx_header	*hdr;
	struct mbuf		*m;
	bus_dmamap_t		 map;
	bus_addr_t		 paddr;
	int			 error;

	KASSERT(dr != NULL, ("ring is not specified"));
	dma = dr->dma;
	/*
	 * Get new mbuf
	 */
	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL) {
		error = ENOBUFS;

		/*
		 * If the NIC is up and running, we need to:
		 * - Clear RX buffer's header.
		 * - Restore RX descriptor settings.
		 */
		if (init)
			return (error);
		else
			goto back;
	}
	m->m_len = m->m_pkthdr.len = MCLBYTES;

	bcm_dma_set_redzone(dr, m);

	/*
	 * Try to load RX buf into temporary DMA map
	 */
	KASSERT(dma->rxbuf_dtag != NULL, ("rxbuf_dtag isn't initialized"));
	error = bus_dmamap_load_mbuf(dma->rxbuf_dtag, dr->dr_spare_dmap, m,
	    bcm_dmamap_callback_mbuf, &paddr, BUS_DMA_NOWAIT);
	if (error) {
		m_freem(m);

		/*
		 * See the comment above
		 */
		if (init)
			return (error);
		else
			goto back;
	}

	if (!init)
		bus_dmamap_unload(dma->rxbuf_dtag, meta->mt_dmap);
	meta->mt_m = m;
	meta->mt_paddr = paddr;

	/*
	 * Swap RX buf's DMA map with the loaded temporary one
	 */
	map = meta->mt_dmap;
	meta->mt_dmap = dr->dr_spare_dmap;
	dr->dr_spare_dmap = map;

	/*
	 *  Clear RX buf header
	 */
back:
	hdr = mtod(meta->mt_m, struct bcm_rx_header *);
	bzero(hdr, sizeof(struct bcm_rx_header));
	bus_dmamap_sync(dma->rxbuf_dtag, meta->mt_dmap,
	    BUS_DMASYNC_PREWRITE);

	/*
	 * Setup RX buf descriptor
	 */
	dr->setdesc(dr, desc, meta->mt_paddr,
	    meta->mt_m->m_len - sizeof(struct bcm_rx_header), 0, 0, 0);
#if 0
	/*TODO: add debug configuration bits */
	BCM_DMADESC_DUMP(desc);
#endif
	return (error);
}

static void
bcm_dma_set_redzone(struct bcm_dma_ring *dr, struct mbuf *m)
{
	struct bcm_rx_header	*rxhdr;
	unsigned char		*frame;

	rxhdr = mtod(m, struct bcm_rx_header *);
	rxhdr->len = 0;
	rxhdr->flags = 0;

	frame = mtod(m, char *) + dr->dr_frameoffset;
	memset(frame, 0xff, MCLBYTES - dr->dr_frameoffset);
}

static uint8_t
bcm_dma_check_redzone(struct bcm_dma_ring *dr, struct mbuf *m)
{
	unsigned char *f = mtod(m, char *) + dr->dr_frameoffset;

	return ((f[0] & f[1] & f[2] & f[3] & f[4] & f[5] & f[6] & f[7])
	    == 0xff);
}
