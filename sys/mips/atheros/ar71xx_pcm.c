/*-
 * Copyright (c) 2021 Hiroki Mori
 * All rights reserved.
 *
 * This software was developed by SRI International and the University of
 * Cambridge Computer Laboratory under DARPA/AFRL contract FA8750-10-C-0237
 * ("CTSRD"), as part of the DARPA CRASH research programme.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
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
 */

/* Athros AR71XX I2S/SPDIF-out Audio Interface. */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/sound/pcm/sound.h>
#include <dev/sound/chip.h>
#if 0
#include <mixer_if.h>
#endif

#include <mips/atheros/ar71xxreg.h>
#include <mips/atheros/ar933xreg.h>

#include <mips/atheros/ar71xx_cpudef.h>
#include <mips/atheros/ar71xx_setup.h>

#include <mips/atheros/ar71xx_chip.h>

#include <mips/atheros/ar71xx_pcmvar.h>

void ar934x_pcm_setpll(struct ar933x_pcm_softc *sc, int freq);
int ar934x_pcm_posedge(int freq);

#if 0
/*
 * Mixer interface.
 */
static int
ar71xx_pcmmixer_init(struct snd_mixer *m)
{
	struct sc_pcminfo *scp;
	struct ar71xx_pcm_softc *sc;
	int mask;

	scp = mix_getdevinfo(m);
	sc = scp->sc;

	if (sc == NULL)
		return -1;

	mask = SOUND_MASK_PCM;

	snd_mtxlock(sc->lock);
	pcm_setflags(scp->dev, pcm_getflags(scp->dev) | SD_F_SOFTPCMVOL);
	mix_setdevs(m, mask);
	snd_mtxunlock(sc->lock);

	return (0);
}

static int
ar71xx_pcmmixer_set(struct snd_mixer *m, unsigned dev,
    unsigned left, unsigned right)
{
	struct sc_pcminfo *scp;

	scp = mix_getdevinfo(m);

	/* Here we can configure hardware volume on our DAC */

	return (0);
}

static kobj_method_t ar71xx_pcmmixer_methods[] = {
	KOBJMETHOD(mixer_init,      ar71xx_pcmmixer_init),
	KOBJMETHOD(mixer_set,       ar71xx_pcmmixer_set),
	KOBJMETHOD_END
};
MIXER_DECLARE(ar71xx_pcmmixer);
#endif

/*
 * Channel interface.
 */
static void *
ar71xx_pcmchan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
    struct pcm_channel *c, int dir)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct ar71xx_pcm_softc *sc;

	scp = (struct sc_pcminfo *)devinfo;
	sc = scp->sc;

	snd_mtxlock(sc->lock);
	ch = &scp->chan[0];
	ch->dir = dir;
	ch->run = 0;
	ch->buffer = b;
	ch->channel = c;
	ch->parent = scp;
	snd_mtxunlock(sc->lock);

	if (sndbuf_alloc(ch->buffer, sc->buf_tag, 0, sc->dma_size) != 0) {
		device_printf(scp->dev, "Can't alloc sndbuf.\n");
		return NULL;
	}
	sc->buf_base_phys = sndbuf_getbufaddr(ch->buffer) & 0xfffffff;

	return (ch);
}

static int
ar71xx_pcmchan_free(kobj_t obj, void *data)
{
	struct sc_chinfo *ch = data;
	struct sc_pcminfo *scp = ch->parent;
	struct ar71xx_pcm_softc *sc = scp->sc;

	snd_mtxlock(sc->lock);
	/* TODO: free channel buffer */
	snd_mtxunlock(sc->lock);

	return (0);
}

static int
ar71xx_pcmchan_setformat(kobj_t obj, void *data, uint32_t format)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;

	ch = data;
	scp = ch->parent;

	ch->format = format;

	return (0);
}

static uint32_t
ar71xx_pcmchan_setspeed(kobj_t obj, void *data, uint32_t speed)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct ar71xx_pcm_rate *sr;
	struct ar71xx_pcm_softc *sc;
	int threshold;
	int i;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;

	sr = NULL;

	/* First look for equal frequency. */
	for (i = 0; rate_map[i].speed != 0; i++) {
		if (rate_map[i].speed == speed)
			sr = &rate_map[i];
	}

	/* If no match, just find nearest. */
	if (sr == NULL) {
		for (i = 0; rate_map[i].speed != 0; i++) {
			sr = &rate_map[i];
			threshold = sr->speed + ((rate_map[i + 1].speed != 0) ?
			    ((rate_map[i + 1].speed - sr->speed) >> 1) : 0);
			if (speed < threshold)
				break;
		}
	}

	sc->sr = sr;

	/* Clocks can be reconfigured here. */
	ar934x_pcm_setpll(sc, sr->speed);

	return (sr->speed);
}

static uint32_t
ar71xx_pcmchan_setblocksize(kobj_t obj, void *data, uint32_t blocksize)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct ar71xx_pcm_softc *sc;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;

	sndbuf_resize(ch->buffer, PCM_RX_RING_COUNT, PCM_RX_DMA_SIZE);

	return (sndbuf_getblksz(ch->buffer));
}

static void
ar71xx_pcm_intr(void *arg)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct ar71xx_pcm_softc *sc;
	struct ar71xx_pcm_desc *desc;
	int reg;
	int i;
	int empt, ful;

	scp = arg;
	sc = scp->sc;

	empt = 0;
	ful = 0;
	reg = ATH_READ_REG(AR71XX_MBOX_INT_STATUS);
	ATH_WRITE_REG(AR71XX_MBOX_INT_STATUS, reg);
	if (reg & (1 << 10)) {
		snd_mtxlock(sc->lock);
		ch = &scp->chan[0];
		desc = (struct ar71xx_pcm_desc *)sc->desc_base;
		bus_dmamap_sync(sc->desc_tag, sc->desc_map,
		     BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		for (i = 0; i < PCM_RX_RING_COUNT; ++i) {
			if (desc->OWN == 0) {
				desc->OWN = 1;
				if (empt == 0 || ful == 1)
					sc->pos = i;
				++empt;
				ful = 0;
			} else {
				ful = 1;
			}
			++desc;
		}
		snd_mtxunlock(sc->lock);
		bus_dmamap_sync(sc->desc_tag, sc->desc_map,
		     BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
		if (ch->run)
			for (i = 0; i < empt; ++i) {
				chn_intr(ch->channel);
				++sc->pos;
				sc->pos %= PCM_RX_RING_COUNT;
			}
	}
}

static int
setup_ring(struct sc_pcminfo *scp)
{
	struct ar71xx_pcm_softc *sc;
	struct sc_chinfo *ch;
	int fmt;
	int err;
	struct ar71xx_pcm_desc *desc;
	int i;

	ch = &scp->chan[0];
	sc = scp->sc;

	desc = (struct ar71xx_pcm_desc *)sc->desc_base;

	for (i = 0; i < PCM_RX_RING_COUNT; ++i) {
		memset(desc, 0, sizeof(struct ar71xx_pcm_desc *));

		desc->OWN = 1;
		desc->VUC = 1;
		desc->size = PCM_RX_DMA_SIZE;
		desc->length = PCM_RX_DMA_SIZE;
		desc->BufPtr = sc->buf_base_phys + PCM_RX_DMA_SIZE * i;
		if (i == PCM_RX_RING_COUNT - 1)
			desc->NextPtr = sc->desc_base_phys;
		else
			desc->NextPtr = sc->desc_base_phys + 
			    sizeof(struct ar71xx_pcm_desc) * (i + 1);
#if 0
		 /* 16 Bit, 48 KHz */
		desc->Ca[0] = 0x02100000;
		desc->Ca[1] = 0x000000d2;
		desc->Cb[0] = 0x02200000;
		desc->Cb[1] = 0x000000d2;
#endif
		/* For Dynamic Conf */
		desc->Ca[0] = 1 << 20;
		desc->Cb[0] = 2 << 20;

		++desc;
	}

	bus_dmamap_sync(sc->desc_tag, sc->desc_map,
	     BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	ATH_WRITE_REG(AR71XX_MBOX_DMA_RX_DESCRIPTOR_BASE, sc->desc_base_phys);

}

static int
ar71xx_pcm_start(struct sc_pcminfo *scp)
{
	struct ar71xx_pcm_softc *sc;
	int reg;

	sc = scp->sc;

	if (ar71xx_soc == AR71XX_SOC_AR9331)
		ar71xx_device_start((1 << 1) | 1);

	setup_ring(scp);

	/* set POSEDGE */
	reg = PCM_READ(sc, AR71XX_STEREO0_CONFIG);
	reg = reg & ~0xf;
	reg = reg | ar934x_pcm_posedge(sc->sr->speed);
	if (sc->bclk64fs != 0)
		reg = reg | (1 << 11);
	else
		reg = reg & ~(1 << 11);
	PCM_WRITE(sc, AR71XX_STEREO0_CONFIG, reg);

	ATH_WRITE_REG(AR71XX_RST_RESET, (1 << 1));
	ATH_WRITE_REG(AR71XX_MBOX_FIFO_RESET, (1 << 2));
	ATH_WRITE_REG(AR71XX_MBOX_DMA_POLICY, (1 << 1) | (6 << 4));
	ATH_WRITE_REG(AR71XX_MBOX_INT_ENABLE, (1 << 10));

	reg = ATH_READ_REG(AR71XX_MBOX_INT_STATUS);
	ATH_WRITE_REG(AR71XX_MBOX_INT_STATUS, reg);

	/* do RESET */
	reg = PCM_READ(sc, AR71XX_STEREO0_CONFIG);
	PCM_WRITE(sc, AR71XX_STEREO0_CONFIG, reg | (1 << 19));
	DELAY(100);

	ATH_WRITE_REG(AR71XX_MBOX_DMA_RX_CONTROL, 1 << 1);
	ATH_READ_REG(AR71XX_MBOX_DMA_RX_CONTROL);

	return (0);
}

static int
ar71xx_pcm_stop(struct sc_pcminfo *scp)
{
	struct ar71xx_pcm_softc *sc;
	int reg;

	sc = scp->sc;

	ATH_WRITE_REG(AR71XX_MBOX_INT_ENABLE, 0);
	ATH_WRITE_REG(AR71XX_MBOX_DMA_RX_CONTROL, 1 << 2);
	reg = ATH_READ_REG(AR71XX_MBOX_INT_STATUS);

	return (0);
}

static int
ar71xx_pcmchan_trigger(kobj_t obj, void *data, int go)
{
	struct sc_pcminfo *scp;
	struct sc_chinfo *ch;
	struct ar71xx_pcm_softc *sc;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;

	snd_mtxlock(sc->lock);

	switch (go) {
	case PCMTRIG_START:
		ch->run = 1;

		sc->pos = 0;

		ar71xx_pcm_start(scp);

		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		ch->run = 0;

		ar71xx_pcm_stop(scp);

		sc->pos = 0;

		break;
	}

	snd_mtxunlock(sc->lock);

	return (0);
}

static uint32_t
ar71xx_pcmchan_getptr(kobj_t obj, void *data)
{
	struct sc_chinfo *ch;
	struct sc_pcminfo *scp;
	struct ar71xx_pcm_softc *sc;

	ch = data;
	scp = ch->parent;
	sc = scp->sc;

	return (sc->pos * PCM_RX_DMA_SIZE);
}

static uint32_t ar71xx_pcm_pfmt[] = {
	SND_FORMAT(AFMT_S16_BE, 2, 0),
	0
};

static struct pcmchan_caps ar71xx_pcm_pcaps =
	{32000, 48000, ar71xx_pcm_pfmt, 0};

static struct pcmchan_caps *
ar71xx_pcmchan_getcaps(kobj_t obj, void *data)
{

	return (&ar71xx_pcm_pcaps);
}

static kobj_method_t ar71xx_pcmchan_methods[] = {
	KOBJMETHOD(channel_init,         ar71xx_pcmchan_init),
	KOBJMETHOD(channel_free,         ar71xx_pcmchan_free),
	KOBJMETHOD(channel_setformat,    ar71xx_pcmchan_setformat),
	KOBJMETHOD(channel_setspeed,     ar71xx_pcmchan_setspeed),
	KOBJMETHOD(channel_setblocksize, ar71xx_pcmchan_setblocksize),
	KOBJMETHOD(channel_trigger,      ar71xx_pcmchan_trigger),
	KOBJMETHOD(channel_getptr,       ar71xx_pcmchan_getptr),
	KOBJMETHOD(channel_getcaps,      ar71xx_pcmchan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(ar71xx_pcmchan);

struct ar71xx_pcm_dmamap_arg {
	bus_addr_t	pcm_busaddr;
};

static void
ar71xx_pcm_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int err)
{
	struct ar71xx_pcm_dmamap_arg *ctx;

	if (err)
		return;

	ctx = (bus_addr_t*)arg;
	ctx->pcm_busaddr = segs[0].ds_addr;
}

static int
ar71xx_pcm_dma_setup(struct ar71xx_pcm_softc *sc)
{
	struct ar71xx_pcm_dmamap_arg   ctx;
	device_t dev;
	int err;
	int desc_size;

	dev = sc->dev;

	err = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),
	    1, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    BUS_SPACE_MAXSIZE_32BIT, 0,	/* maxsize, nsegments */
	    BUS_SPACE_MAXSIZE_32BIT, 0,	/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->parent_tag);
	if (err) {
		device_printf(dev, "cannot create bus dma tag\n");
		return (-1);
	}

	desc_size = sizeof(struct ar71xx_pcm_desc);

	err = bus_dma_tag_create(
	    sc->parent_tag,
	    desc_size, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    desc_size*PCM_RX_RING_COUNT, 1,	/* maxsize, nsegments */
	    desc_size*PCM_RX_RING_COUNT, 0,	/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->desc_tag);
	if (err) {
		device_printf(dev, "cannot create bus dma tag\n");
		return (-1);
	}

	err = bus_dmamem_alloc(sc->desc_tag, (void **)&sc->desc_base,
	    BUS_DMA_WAITOK | BUS_DMA_COHERENT, &sc->desc_map);
	if (err) {
		device_printf(dev, "cannot allocate memory\n");
		return (-1);
	}

	err = bus_dmamap_load(sc->desc_tag, sc->desc_map, sc->desc_base,
	    desc_size*PCM_RX_RING_COUNT, ar71xx_pcm_dmamap_cb, &ctx, 0);
	if (err) {
		device_printf(dev, "cannot load DMA map\n");
		return (-1);
	}
	sc->desc_base_phys = ctx.pcm_busaddr & 0xfffffff;

	/* DMA buffer size. */
	sc->dma_size = PCM_RX_DMA_SIZE * PCM_RX_RING_COUNT;

	/*
	 * Must use dma_size boundary as modulo feature required.
	 * Modulo feature allows setup circular buffer.
	 */
	err = bus_dma_tag_create(
	    sc->parent_tag,
	    PCM_RX_DMA_SIZE, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    sc->dma_size, 1,	/* nsegments */
	    sc->dma_size, 0,	/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->buf_tag);
	if (err) {
		device_printf(dev, "cannot create bus dma tag\n");
		return (-1);
	}

	return (0);
}

static int
ar71xx_pcm_configure_clocks(struct ar71xx_pcm_softc *sc)
{
	uint64_t pcm_freq;
	device_t dev;
	int err;

	dev = sc->dev;

	if (ar71xx_soc == AR71XX_SOC_AR9331) {
		/* 48k for AR9331 */
		pcm_freq = 48000;
		PCM_WRITE(sc, AR933X_STEREO_CLK_DIV, (0x10 << 16) | 0x46AB);
	} else {
		/* XXX Other SOC support */
		pcm_freq = 48000;
		ar934x_pcm_setpll(sc, pcm_freq);
	}

	device_printf(dev, "Frequency ar71xx_pcm %d\n", (uint32_t)pcm_freq);

	return (0);
}

static int
ar71xx_pcm_configure(struct ar71xx_pcm_softc *sc)
{
	int reg;

//	reg = PCM_READ(sc, AR71XX_STEREO0_CONFIG);
	reg = (1 << 23) | (1 << 21) | (1 << 12) | (1 << 9) | (1 << 8);
	PCM_WRITE(sc, AR71XX_STEREO0_CONFIG, reg);

	return (0);
}

static int
ar71xx_pcm_probe(device_t dev)
{

	device_set_desc(dev, "Atheros AR71XX I2S/SPDIF-out Audio Interface");

	return (BUS_PROBE_DEFAULT);
}

static int
ar71xx_pcm_attach(device_t dev)
{
	char status[SND_STATUSLEN];
	struct sc_pcminfo *scp;
	struct ar71xx_pcm_softc *sc;
	int err;
	int reg;

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK | M_ZERO);
	sc->dev = dev;
	sc->pos = 0;

	/* Setup sound subsystem */
	sc->lock = snd_mtxcreate(device_get_nameunit(dev), "ar71xx_pcm softc");
	if (sc->lock == NULL) {
		device_printf(dev, "Can't create mtx.\n");
		return (ENXIO);
	}

	if (bus_alloc_resources(dev, ar71xx_pcm_spec, sc->res)) {
		device_printf(dev,
		    "could not allocate resources for device\n");
		return (ENXIO);
	}

	/* Memory interface */
	sc->bst = rman_get_bustag(sc->res[0]);
	sc->bsh = rman_get_bushandle(sc->res[0]);

	/* Setup PCM. */
	scp = malloc(sizeof(struct sc_pcminfo), M_DEVBUF, M_WAITOK | M_ZERO);
	scp->sc = sc;
	scp->dev = dev;

	reg = ATH_READ_REG(AR71XX_MBOX_INT_STATUS);
	/* Setup audio buffer. */
	err = ar71xx_pcm_dma_setup(sc);
	if (err != 0) {
		device_printf(dev, "Can't setup sound buffer.\n");
		return (ENXIO);
	}

	/* Setup clocks. */
	err = ar71xx_pcm_configure_clocks(sc);
	if (err != 0) {
		device_printf(dev, "Can't configure clocks.\n");
		return (ENXIO);
	}

	err = ar71xx_pcm_configure(sc);
	if (err != 0) {
		device_printf(dev, "Can't configure AIC.\n");
		return (ENXIO);
	}

	pcm_setflags(dev, pcm_getflags(dev) | SD_F_MPSAFE);

	/* Setup interrupt handler. */
	if (snd_setup_intr(dev, sc->res[1], 0, ar71xx_pcm_intr, 
	    scp, &sc->pcm_ih)) {
		device_printf(dev,
		    "WARNING: unable to register interrupt handler\n");
		return (ENXIO);
	}

	err = pcm_register(dev, scp, 1, 0);
	if (err) {
		device_printf(dev, "Can't register pcm.\n");
		return (ENXIO);
	}

	scp->chnum = 0;
	pcm_addchan(dev, PCMDIR_PLAY, &ar71xx_pcmchan_class, scp);
	scp->chnum++;

	snprintf(status, SND_STATUSLEN, "at apb");
	pcm_setstatus(dev, status);

#if 0
	mixer_init(dev, &ar71xx_pcmmixer_class, scp);
#endif

	/* Create device sysctl node. */
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "bclk64fs", CTLFLAG_RW, &sc->bclk64fs, 0,
	    "I2S bit clock is 64fs");

	return (0);
}

static int
ar71xx_pcm_detach(device_t dev)
{
	struct ar71xx_pcm_softc *sc;

	sc = device_get_softc(dev);

	bus_release_resources(dev, ar71xx_pcm_spec, sc->res);

	return (0);
}

static device_method_t ar71xx_pcm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ar71xx_pcm_probe),
	DEVMETHOD(device_attach,	ar71xx_pcm_attach),
	DEVMETHOD(device_detach,	ar71xx_pcm_detach),
	DEVMETHOD_END
};

static driver_t ar71xx_pcm_driver = {
	"pcm",
	ar71xx_pcm_methods,
	PCM_SOFTC_SIZE,
};

DRIVER_MODULE(ar71xx_pcm, apb, ar71xx_pcm_driver, pcm_devclass, 0, 0);
MODULE_DEPEND(ar71xx_pcm, sound, SOUND_MINVER, SOUND_PREFVER, SOUND_MAXVER);
MODULE_VERSION(ar71xx_pcm, 1);
