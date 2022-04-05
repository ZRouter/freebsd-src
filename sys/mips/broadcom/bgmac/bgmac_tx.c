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
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <sys/kdb.h>

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

#include <machine/bus.h>
#include <machine/resource.h>

#include "bgmac.h"
#include "bgmacvar.h"
#include "bgmacreg.h"

#include "bcm_dma.h"

/* ***************************************************************
 * 	Internal prototypes
 * ***************************************************************
 */

static void	bgmac_start_locked(struct ifnet *ifp);

void
bgmac_if_start(struct ifnet *ifp)
{
	struct bgmac_softc *sc;

	sc = if_getsoftc(ifp);
	BGMAC_LOCK(sc);
	bgmac_start_locked(ifp);
	BGMAC_UNLOCK(sc);
}

static void
bgmac_start_locked(struct ifnet *ifp)
{
	struct bgmac_softc	*sc;
	struct mbuf		*m_head;
	struct mbuf		*m0;
	int			 count;
	int			 err;

	sc = if_getsoftc(ifp);
	BGMAC_ASSERT_LOCKED(sc);

	if ((if_getdrvflags(ifp) & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;

	for (count = 0; !if_sendq_empty(ifp);) {
		BHND_DEBUG_DEV(sc->dev, "TX queue has new data");

		/* TODO: check if outgoing HW queue is full */
//		if (sc->bge_txcnt > BGE_TX_RING_CNT - 16) {
//			if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
//			break;
//		}

		m_head = if_dequeue(ifp);
		if (m_head == NULL)
			break;

		/*
		 * Pack the data into the transmit ring. If we
		 * don't have room, set the OACTIVE flag and wait
		 * for the NIC to drain the ring.
		 */

		for (m0 = m_head; m0 != NULL; m0 = m0->m_next)
			CTR3(KTR_BGMAC, "%s: TX[      ] mbuf[%d] %p",
			    device_get_nameunit(sc->dev),m0->m_len, m_head);

		if (m_head->m_next != NULL) {
			/* packet is split into set of small mbufs. merge them */
			m0 = m_defrag(m_head, M_NOWAIT);
			if (m0 == NULL) {
				BHND_ERROR_DEV(sc->dev, "m_defrag failed");
				if_sendq_prepend(ifp, m_head);
				break;
			}
			/* m_defrag freed src mbuf chain, good by m_head */
			m_head = m0;
			for (m0 = m_head; m0 != NULL; m0 = m0->m_next)
				CTR2(KTR_BGMAC, "%s: TX[defrag] mbuf[%d]",
				    device_get_nameunit(sc->dev),m0->m_len);
		}

		err = bcm_dma_tx_start(sc->dma, m_head);
		if (err) {
			BHND_ERROR_DEV(sc->dev, "bcm_dma_tx_start error: %d",
					    err);
			if_sendq_prepend(ifp, m_head);
			if_setdrvflagbits(ifp, IFF_DRV_OACTIVE, 0);
			break;
		}
		++count;
		CTR1(KTR_BGMAC, "%s: TX: pkt sent", device_get_nameunit(sc->dev));

		/*
		 * If there's a BPF listener, bounce a copy of this frame
		 * to him.
		 */
		if_bpfmtap(ifp, m_head);
	}
}
