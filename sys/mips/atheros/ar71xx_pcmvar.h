/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2021 Hiroki Mori
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * ath-pcm.h -- ALSA PCM interface for the QCA Wasp based audio interface
 *
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * $FreeBSD$
 *
 */

#ifndef __AR71XX_PCMVAR_H__
#define __AR71XX_PCMVAR_H__

#define PCM_RX_RING_COUNT	4
#define PCM_RX_DMA_SIZE		(192 * 8 * 2) /* fit to spdif block boundary */

#define PCM_LOCK(_sc)		mtx_lock(&(_sc)->lock)
#define PCM_UNLOCK(_sc)	mtx_unlock(&(_sc)->lock)
#define PCM_LOCK_ASSERT(_sc)	mtx_assert(&(_sc)->lock, MA_OWNED)

/*
 * register space access macros
 */
#define PCM_WRITE(sc, reg, val)	do {	\
		bus_write_4(sc->res[0], (reg), (val)); \
	} while (0)

#define PCM_READ(sc, reg)	 bus_read_4(sc->res[0], (reg))

#define PCM_SET_BITS(sc, reg, bits)	\
	PCM_WRITE(sc, reg, PCM_READ(sc, (reg)) | (bits))

#define PCM_CLEAR_BITS(sc, reg, bits)	\
	PCM_WRITE(sc, reg, PCM_READ(sc, (reg)) & ~(bits))

#define	ATH_NCHANNELS		1

struct ar71xx_pcm_desc {
	unsigned int	OWN     :  1,    /* bit 31 */
			EOM     :  1,    /* bit 30 */
			rsvd1   :  5,    /* bit 29-25 */
			VUC     :  1,    /* bit 24 */
			size    : 12,    /* bit 23-12 */
			length  : 12,    /* bit 11-00 */
			rsvd2   :  4,    /* bit 31-28 */
			BufPtr  : 28,    /* bit 27-00 */
			rsvd3   :  4,    /* bit 31-28 */
			NextPtr : 28;    /* bit 27-00 */

	unsigned int Va[6];
	unsigned int Ua[6];
	unsigned int Ca[6];
	unsigned int Vb[6];
	unsigned int Ub[6];
	unsigned int Cb[6];
};

struct ar71xx_pcm_softc {
	device_t		dev;
	struct resource		*res[2];
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	void			*pcm_ih;
	struct mtx		*lock;
	int			pos;
	int			size;
	bus_dma_tag_t		parent_tag;
	bus_dma_tag_t		desc_tag;
	bus_dmamap_t		desc_map;
	bus_addr_t		desc_base_phys;
	uint32_t		*desc_base;
	bus_dma_tag_t		buf_tag;
//	bus_dmamap_t		buf_map;
	bus_addr_t		buf_base_phys;
//	uint32_t		*buf_base;
	uintptr_t		ar71xx_pcm_fifo_paddr;
	int			dma_size;
	struct ar71xx_pcm_rate		*sr;
	int			internal_codec;
};

/* Channel registers */
struct sc_chinfo {
	struct snd_dbuf		*buffer;
	struct pcm_channel	*channel;
	struct sc_pcminfo	*parent;

	/* Channel information */
	uint32_t	dir;
	uint32_t	format;

	/* Flags */
	uint32_t	run;
};

/* PCM device private data */
struct sc_pcminfo {
	device_t		dev;
	uint32_t		chnum;
	struct sc_chinfo	chan[ATH_NCHANNELS];
	struct ar71xx_pcm_softc	*sc;
};

static struct resource_spec ar71xx_pcm_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static int ar71xx_pcm_probe(device_t dev);
static int ar71xx_pcm_attach(device_t dev);
static int ar71xx_pcm_detach(device_t dev);
static int setup_dma(struct sc_pcminfo *scp);

struct ar71xx_pcm_rate {
        uint32_t speed;
};

static struct ar71xx_pcm_rate rate_map[] = {
	{ 32000 },
	{ 44100 },
	{ 48000 },
	{ 0 },
};

#endif	/* __AR71XX_PCMVAR_H__ */
