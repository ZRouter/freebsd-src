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

#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/errno.h>
#include <sys/kdb.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#include <net/bpf.h>

#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

#include "bgmac.h"
#include "bgmacvar.h"
#include "bgmacreg.h"

#include "bcm_dma.h"
#include "bcm_dma_reg.h"
#include "bcm_dma_ringvar.h"

int
bcm_dma_tx_start(struct bcm_dma *dma, struct mbuf *m)
{
	device_t			 dev;
	struct bcm_dma_ring		*dr;
	struct bcm_dmadesc_generic	*desc;
	struct bcm_dmadesc_meta		*mt;
	int				 slot;
	int				 error;

	dr = dma->wme[0];
	dev = rman_get_device(dr->res);

	BGMAC_ASSERT_RING_LOCKED(dr);
	KASSERT(!dr->dr_stop, ("%s:%d: fail", __func__, __LINE__));

	slot = bcm_dma_ring_get_head(dr);
	dr->getdesc(dr, slot, &desc, &mt);

	CTR5(KTR_BGMAC, "sending TX slot[%d]: tag %p, map %p, mbuf %p size %d",
	    slot, dr->dr_ring_dtag, mt->mt_dmap, m, m->m_len);

	error = bus_dmamap_load_mbuf(dma->txbuf_dtag, mt->mt_dmap, m,
		bcm_dmamap_callback_mbuf, &mt->mt_paddr, BUS_DMA_NOWAIT);
	if (error) {
		BHND_ERROR_DEV(dev, "can't load TX buffer: %d", error);
		panic("can't load TX buffer");
		goto fail;
	}

	mt->mt_m = m;
	bus_dmamap_sync(dma->txbuf_dtag, mt->mt_dmap, BUS_DMASYNC_PREWRITE);

	/* Update NIC about new packet */
	dr->setdesc(dr, desc, mt->mt_paddr, m->m_pkthdr.len, 1, 1, 1);
	bus_dmamap_sync(dr->dr_ring_dtag, dr->dr_ring_dmap, BUS_DMASYNC_PREWRITE);
	dr->dr_head_slot = bcm_dma_ring_get_nextslot(dr, slot);
	if (dr->dr_head_slot == bcm_dma_ring_get_tail(dr)) {
		/* Force TX processing */
		bcm_dma_tx(dr);
		if (dr->dr_head_slot == bcm_dma_ring_get_tail(dr))
			CTR5(KTR_BGMAC, "ERROR: TX ring is full: TX slot[%d],"
					" tag %p, map %p, mbuf %p size %d",
			    slot, dr->dr_ring_dtag, mt->mt_dmap, m, m->m_len);
	}
	dr->start_transfer(dr, dr->dr_head_slot);
	return (0);
fail:
/* TODO: support fail */
//	dr->dr_curslot = backup[0];
//	dr->dr_usedslot = backup[1];
	return (error);
}

void
bcm_dma_tx(struct bcm_dma_ring *dr)
{
	struct bcm_dma			*dma;
	struct bcm_dmadesc_generic	*desc;
	struct bcm_dmadesc_meta		*meta;
	int				 slot, target;

	BGMAC_ASSERT_RING_LOCKED(dr);
	dma = dr->dma;
	KASSERT(dr->dr_is_tx > 0, ("%s:%d: fail", __func__, __LINE__));

	/* Get slot free for driver, i.e. device sent all data */
	slot = bcm_dma_ring_get_tail(dr);
	target = dr->get_curslot(dr);

	while (slot != target)
	{
		KASSERT(slot >= 0 && slot < dr->dr_numslots,
		    ("%s:%d: fail - %d", __func__, __LINE__, slot));
		dr->getdesc(dr, slot, &desc, &meta);

		bus_dmamap_sync(dma->txbuf_dtag, meta->mt_dmap, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(dma->txbuf_dtag, meta->mt_dmap);

		CTR5(KTR_BGMAC, "free mbuf TX slot[%d->%d]: tag %p, map %p, mbuf %p",
		    slot, target, dr->dr_ring_dtag, meta->mt_dmap, meta->mt_m);

		if (meta->mt_m != NULL)
			m_freem(meta->mt_m);
		meta->mt_m = NULL;

		slot = bcm_dma_ring_get_nextslot(dr, slot);
	}
	dr->dr_tail_slot = slot;
}
