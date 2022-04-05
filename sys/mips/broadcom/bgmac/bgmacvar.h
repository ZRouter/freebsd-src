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

#ifndef _MIPS_BROADCOM_BGMAC_BGMACVAR_H_
#define _MIPS_BROADCOM_BGMAC_BGMACVAR_H_

#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/kernel.h>

#include <sys/time.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_media.h>

#define BGMAC_TIMEOUT 1000
#define BGMAC_MAX_RSPEC	3

MALLOC_DECLARE(M_BHND_BGMAC);

struct bgmac_softc {
	struct mtx		 sc_mtx;
	device_t 		 dev;
	device_t		 miibus;
	device_t		 mdio;
	struct resource		*mem;
	struct resource		*irq;
	void			*intrhand;
	struct ifnet		*ifp;
	u_char                   addr[6];
	struct bcm_dma		*dma;
	struct ifmedia		 ifmedia;
//	bus_dma_tag_t		 parent_tag, ring_tag;
//	bus_dmamap_t		 ring_map;
//	bus_addr_t		 rxdesc_ring_busaddr;
//	void			*buf;
//	void			*rxdesc_ring;
};

typedef enum{
	PHY_READ,
	PHY_WRITE
}  phymode ;

/* Concurrency macros */
#define	BGMAC_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev), \
	    MTX_NETWORK_LOCK, MTX_DEF)
#define	BGMAC_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->sc_mtx)
#define	BGMAC_LOCK(sc)			mtx_lock(&(sc)->sc_mtx)
#define	BGMAC_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define	BGMAC_ASSERT_LOCKED(sc)		mtx_assert(&(sc)->sc_mtx, MA_OWNED)

#endif /* _MIPS_BROADCOM_BGMAC_BGMACVAR_H_ */
