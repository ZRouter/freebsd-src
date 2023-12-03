/*-
 * Copyright (c) 2023 Hiroki Mori
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/watchdog.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mediatek/mtk_sysctl.h>
#include <mips/mediatek/mtk_soc.h>
#include <mips/mediatek/mtk_gdma.h>

/*
 * register space access macros
 */
#define GDMA_WRITE(sc, reg, val)	do {	\
		bus_write_4(sc->sc_mem_res, (reg), (val)); \
	} while (0)

#define GDMA_READ(sc, reg)	 bus_read_4(sc->sc_mem_res, (reg))

#define GDMA_SET_BITS(sc, reg, bits)	\
	GDMA_WRITE(sc, reg, GDMA_READ(sc, (reg)) | (bits))

#define GDMA_CLEAR_BITS(sc, reg, bits)	\
	GDMA_WRITE(sc, reg, GDMA_READ(sc, (reg)) & ~(bits))

#define MAXTIMER	0xffff
#define GDMA_MAXFRAGS	8
#define GDMA_TX_DMA_SIZE	1024
#define GDMA_RX_DMA_SIZE	1024

struct mtk_gdma_softc {
	device_t		dev;
	struct resource		*sc_mem_res;
	int debug;
	bus_dma_tag_t		gdma_parent_tag;
	bus_dma_tag_t		gdma_tx_tag;
	bus_dma_tag_t		gdma_rx_tag;
	bus_dmamap_t		gdma_tx_map;
	bus_dmamap_t		gdma_rx_map;
	char			*gdma_tx_buff;
	char			*gdma_rx_buff;
	bus_addr_t		gdma_tx_buff_paddr;
	bus_addr_t		gdma_rx_buff_paddr;
};

static const struct ofw_compat_data compat_data[] = {
	{ "ralink,rt2880-gdma",		1 },

	/* Sentinel */
	{ NULL,				0 }
};

static int
mtk_gdma_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MTK GDMA Controller");

	return (0);
}

struct gdma_dmamap_arg {
	bus_addr_t	gdma_busaddr;
};

static void
gdma_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct gdma_dmamap_arg  *ctx;

	if (error != 0)
		return;
	ctx = arg;
	ctx->gdma_busaddr = segs[0].ds_addr;
}

static int
mtk_gdma_alloc(struct mtk_gdma_softc *sc)
{
	struct gdma_dmamap_arg	ctx;
//	struct gdma_txdesc	*txd;
//	struct gdma_rxdesc	*rxd;
	int			error, i;
	int			gdma_tx_align, gdma_rx_align;

	/* Assume 4 byte alignment by default */
	gdma_tx_align = 4;
	gdma_rx_align = 4;

	/* Create parent DMA tag. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* parent */
	    1, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsize */
	    0,				/* nsegments */
	    BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->gdma_parent_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "failed to create parent DMA tag\n");
		goto fail;
	}

	/* Create tag for Tx buffers. */
	error = bus_dma_tag_create(
	    sc->gdma_parent_tag,	/* parent */
	    gdma_tx_align, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES,			/* maxsize */
	    GDMA_MAXFRAGS,		/* nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->gdma_tx_tag);
	if (error != 0) {
		device_printf(sc->dev, "failed to create Tx DMA tag\n");
		goto fail;
	}

	/* Create tag for Rx buffers. */
	error = bus_dma_tag_create(
	    sc->gdma_parent_tag,	/* parent */
	    gdma_rx_align, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR,		/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES,			/* maxsize */
	    GDMA_MAXFRAGS,		/* nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->gdma_rx_tag);
	if (error != 0) {
		device_printf(sc->dev, "failed to create Rx DMA tag\n");
		goto fail;
	}

	/* Allocate DMA'able memory and load the DMA map for Tx buffer. */
	error = bus_dmamem_alloc(sc->gdma_tx_tag,
	    (void **)&sc->gdma_tx_buff, BUS_DMA_WAITOK |
	    BUS_DMA_COHERENT | BUS_DMA_ZERO,
	    &sc->gdma_tx_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "failed to allocate DMA'able memory for Tx buffer\n");
		goto fail;
	}

	ctx.gdma_busaddr = 0;
	error = bus_dmamap_load(sc->gdma_tx_tag,
	    sc->gdma_tx_map, sc->gdma_tx_buff,
	    GDMA_TX_DMA_SIZE, gdma_dmamap_cb, &ctx, 0);
	if (error != 0 || ctx.gdma_busaddr == 0) {
		device_printf(sc->dev,
		    "failed to load DMA'able memory for Tx buffer\n");
		goto fail;
	}
	sc->gdma_tx_buff_paddr = ctx.gdma_busaddr;

	/* Allocate DMA'able memory and load the DMA map for Rx buffer. */
	error = bus_dmamem_alloc(sc->gdma_rx_tag,
	    (void **)&sc->gdma_rx_buff, BUS_DMA_WAITOK |
	    BUS_DMA_COHERENT | BUS_DMA_ZERO,
	    &sc->gdma_rx_map);
	if (error != 0) {
		device_printf(sc->dev,
		    "failed to allocate DMA'able memory for Rx buffer\n");
		goto fail;
	}

	ctx.gdma_busaddr = 0;
	error = bus_dmamap_load(sc->gdma_rx_tag,
	    sc->gdma_rx_map, sc->gdma_rx_buff,
	    GDMA_RX_DMA_SIZE, gdma_dmamap_cb, &ctx, 0);
	if (error != 0 || ctx.gdma_busaddr == 0) {
		device_printf(sc->dev,
		    "failed to load DMA'able memory for Rx buffer\n");
		goto fail;
	}
	sc->gdma_rx_buff_paddr = ctx.gdma_busaddr;

fail:
	return (error);
}

static void
mtk_gdma_free(struct mtk_gdma_softc *sc)
{
	int			i;

	/* Tx buffer. */
	if (sc->gdma_tx_tag) {
		if (sc->gdma_tx_buff_paddr)
			bus_dmamap_unload(sc->gdma_tx_tag,
			    sc->gdma_tx_map);
		if (sc->gdma_tx_buff)
			bus_dmamem_free(sc->gdma_tx_tag,
			    sc->gdma_tx_buff,
			    sc->gdma_tx_map);
		sc->gdma_tx_buff = NULL;
		sc->gdma_tx_buff_paddr = 0;
		bus_dma_tag_destroy(sc->gdma_tx_tag);
		sc->gdma_tx_tag = NULL;
	}
	/* Rx buffer. */
	if (sc->gdma_rx_tag) {
		if (sc->gdma_rx_buff_paddr)
			bus_dmamap_unload(sc->gdma_rx_tag,
			    sc->gdma_rx_map);
		if (sc->gdma_rx_buff)
			bus_dmamem_free(sc->gdma_rx_tag,
			    sc->gdma_rx_buff,
			    sc->gdma_rx_map);
		sc->gdma_rx_buff = NULL;
		sc->gdma_rx_buff_paddr = 0;
		bus_dma_tag_destroy(sc->gdma_rx_tag);
		sc->gdma_rx_tag = NULL;
	}

	if (sc->gdma_parent_tag) {
		bus_dma_tag_destroy(sc->gdma_parent_tag);
		sc->gdma_parent_tag = NULL;
	}
}

static void
mtk_gdma_sysctl(device_t dev)
{
	struct mtk_gdma_softc *sc = device_get_softc(dev);

	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(sc->dev);

	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->debug, 0,
	    "enable watchdog debugging");
}

static int
mtk_gdma_attach(device_t dev)
{
	struct mtk_gdma_softc *sc = device_get_softc(dev);
	int rid;

        rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "Could not map memory\n");
		return (ENXIO);
	}

	/* Initialise */
	sc->debug = 0;

	if (device_get_unit(dev) != 0) {
		device_printf(dev, "Only one gdma control allowed\n");
		return (ENXIO);
	}

	sc->dev = dev;

	mtk_gdma_alloc(sc);

	mtk_gdma_sysctl(dev);

	return (0);
}

static device_method_t mtk_gdma_methods[] = {
	DEVMETHOD(device_probe,		mtk_gdma_probe),
	DEVMETHOD(device_attach,	mtk_gdma_attach),

	DEVMETHOD_END
};

static driver_t mtk_gdma_driver = {
	"mtk_gdma",
	mtk_gdma_methods,
	sizeof(struct mtk_gdma_softc),
};
static devclass_t mtk_gdma_devclass;

EARLY_DRIVER_MODULE(mtk_gdma, simplebus, mtk_gdma_driver, mtk_gdma_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_EARLY);
