/*-
 * Copyright (c) 2024 Hiroki Mori
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
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/mutex.h>
#include <sys/ktr.h>
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

#include "miibus_if.h"
#include "mdio_if.h"

#include <dev/mdio/mdio.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miidevs.h"

#include <machine/bus.h>
#include <machine/resource.h>

#include "if_unimacreg.h"

MALLOC_DEFINE(M_BHND_BGMAC, "unimac", "Structures allocated by unimac driver");

struct resource_spec unimac_rspec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, -1, 0 }
};


/****************************************************************************
 * Prototypes
 ****************************************************************************/
static int	unimac_probe(device_t dev);
static int	unimac_attach(device_t dev);
static int	unimac_detach(device_t dev);

/* Interrupt flow */
static void	unimac_intr(void *arg);

/****************************************************************************
 * Implementation
 ****************************************************************************/
void
unimac_print_debug(struct unimac_softc* sc)
{
#if 0
	BGMACDUMP(sc);
	BGMACDUMPMIB(sc);
	BGMACDUMPERRORS(sc);
#endif
}

static int
unimac_probe(device_t dev)
{
/*
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, unimac_match, sizeof(unimac_match[0]));
	if (id == NULL)
		return (ENXIO);

	device_set_desc(dev, id->desc);
*/
	return (BUS_PROBE_DEFAULT);
}

static int
unimac_attach(device_t dev)
{
	struct unimac_softc	*sc;
	struct resource		*res[2];
	int			 error;
	struct sysctl_ctx_list	*ctx;
	struct sysctl_oid	*tree;
	char 			 name[IFNAMSIZ];

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Allocate bus resources */
	error = bus_alloc_resources(dev, unimac_rspec, res);
	if (error){
		device_printf(dev, "can't allocate resources: %d\n", error);
		return (error);
	}

	sc->mem = res[0];
	sc->irq = res[1];

	/* Hook interrupt */
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_NET | INTR_MPSAFE,
			NULL, unimac_intr, sc, &sc->intrhand);
	if (error) {
		device_printf(dev, "can't setup interrupt\n");
		unimac_detach(dev);
		return (error);
	}

	int i, j;
	int off;
	/* BCHP_MBDMA_REG_START */
	off = 0x0;
	for (i = 0; i < 4 ; ++i) {
		printf("%08x ", off + i * 0x10);
		for (j = 0; j < 4 ; ++j) {
			printf("%08x ",
			    bus_read_4(sc->mem, off + i * 0x10 + j * 4));
		}
		printf("\n");
	}
	/* BCHP_UNIMAC_INTERFACE0_REG_START */
	off = 0x600 + 0x2000;
	for (i = 0; i < 4 ; ++i) {
		printf("%08x ", off + i * 0x10);
		for (j = 0; j < 4 ; ++j) {
			printf("%08x ",
			    bus_read_4(sc->mem, off + i * 0x10 + j * 4));
		}
		printf("\n");
	}
	/* BCHP_UNIMAC_CORE0_REG_START */
	off = 0x800 + 0x2000;
	for (i = 0; i < 4 ; ++i) {
		printf("%08x ", off + i * 0x10);
		for (j = 0; j < 4 ; ++j) {
			printf("%08x ",
			    bus_read_4(sc->mem, off + i * 0x10 + j * 4));
		}
		printf("\n");
	}
	/* BCHP_MIB0_REG_START */
	off = 0xc00 + 0x2000;
	for (i = 0; i < 4 ; ++i) {
		printf("%08x ", off + i * 0x10);
		for (j = 0; j < 4 ; ++j) {
			printf("%08x ",
			    bus_read_4(sc->mem, off + i * 0x10 + j * 4));
		}
		printf("\n");
	}

	return 	0;
}

static int
unimac_detach(device_t dev)
{
	struct unimac_softc	*sc;
	int 			 ret;

	sc = device_get_softc(dev);

	/* detach etherswitch connected via MDIO */
	bus_generic_detach(dev);

	if (sc->mem != NULL)
		bus_release_resource(dev, unimac_rspec[0].type, unimac_rspec[0].rid, sc->mem);

	if (sc->irq != NULL)
		bus_release_resource(dev, unimac_rspec[1].type, unimac_rspec[1].rid, sc->irq);

	return 0;
}	

static void
unimac_intr(void *arg)
{
}

static device_method_t unimac_methods[] = {
		DEVMETHOD(device_probe,	 	unimac_probe),
		DEVMETHOD(device_attach, 	unimac_attach),
		DEVMETHOD(device_detach,	unimac_detach),

#if 0
		/** miibus interface **/
		DEVMETHOD(miibus_readreg, 	unimac_readreg),
		DEVMETHOD(miibus_writereg, 	unimac_writereg),

		/** MDIO interface **/
		DEVMETHOD(mdio_readreg,		unimac_readreg),
		DEVMETHOD(mdio_writereg,	unimac_writereg),
#endif

		DEVMETHOD_END
};

devclass_t unimac_devclass;

DEFINE_CLASS_0(unimac, unimac_driver, unimac_methods, sizeof(struct unimac_softc));

DRIVER_MODULE(unimac, obio, unimac_driver, unimac_devclass, 0, 0);
//DRIVER_MODULE(mdio, unimac, mdio_driver, mdio_devclass, 0, 0);

MODULE_VERSION(unimac, 1);
