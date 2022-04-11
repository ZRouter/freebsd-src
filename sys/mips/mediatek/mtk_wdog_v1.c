/*-
 * Copyright (c) 2022 Hiroki Mori
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

/*
 * register space access macros
 */
#define WDOG_WRITE(sc, reg, val)	do {	\
		bus_write_4(sc->sc_mem_res, (reg), (val)); \
	} while (0)

#define WDOG_READ(sc, reg)	 bus_read_4(sc->sc_mem_res, (reg))

#define WDOG_SET_BITS(sc, reg, bits)	\
	WDOG_WRITE(sc, reg, WDOG_READ(sc, (reg)) | (bits))

#define WDOG_CLEAR_BITS(sc, reg, bits)	\
	WDOG_WRITE(sc, reg, WDOG_READ(sc, (reg)) & ~(bits))

#define MAXTIMER	0xffff

struct mtk_wdog_softc {
	device_t dev;
	struct resource *sc_mem_res;
	int armed;
	int reboot_from_watchdog;
	int debug;
};

static const struct ofw_compat_data compat_data[] = {
	{ "ralink,rt2880-wdt",		1 },

	/* Sentinel */
	{ NULL,				0 }
};

static void
mtk_wdog_fn(void *private, u_int cmd, int *error)
{
	struct mtk_wdog_softc *sc = private;
	device_t dev;

	dev = sc->dev;
	cmd &= WD_INTERVAL;

	if (cmd > 0) {
		if (sc->armed) {
			if (sc->debug)
				device_printf(dev, "fn: reset timer\n");
			WDOG_WRITE(sc, 0x0, MAXTIMER);
		} else {
			if (sc->debug)
				device_printf(dev, "fn: start timer\n");
			WDOG_WRITE(sc, 0x0, MAXTIMER);
			WDOG_WRITE(sc, 0x8, 0x0f | 0x30 | 0x80);
			sc->armed = 1;
		}
	} else {
		if (sc->debug)
			device_printf(dev, "mtk_wdog_fn: disarming\n");
		WDOG_WRITE(sc, 0x8, 0x00);
		sc->armed = 0;
	}
}

static int
mtk_wdog_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MTK WatchDog(Timer1) Controller (v1)");

	return (0);
}

static void
mtk_wdog_sysctl(device_t dev)
{
	struct mtk_wdog_softc *sc = device_get_softc(dev);

	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(sc->dev);

	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->debug, 0,
	    "enable watchdog debugging");
}

static int
mtk_wdog_attach(device_t dev)
{
	struct mtk_wdog_softc *sc = device_get_softc(dev);
	int rid;

        rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "Could not map memory\n");
		return (ENXIO);
	}

	/* Initialise */
	sc->reboot_from_watchdog = 0;
	sc->armed = 0;
	sc->debug = 0;

	if (device_get_unit(dev) != 0) {
		device_printf(dev, "Only one wdog control allowed\n");
		return (ENXIO);
	}

	sc->dev = dev;
	EVENTHANDLER_REGISTER(watchdog_list, mtk_wdog_fn, sc, 0);
	mtk_wdog_sysctl(dev);

	return (0);
}

static device_method_t mtk_wdog_methods[] = {
	DEVMETHOD(device_probe,		mtk_wdog_probe),
	DEVMETHOD(device_attach,	mtk_wdog_attach),

	DEVMETHOD_END
};

static driver_t mtk_wdog_driver = {
	"mtk_wdog",
	mtk_wdog_methods,
	sizeof(struct mtk_wdog_softc),
};
static devclass_t mtk_wdog_devclass;

EARLY_DRIVER_MODULE(mtk_wdog_v1, simplebus, mtk_wdog_driver, mtk_wdog_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_EARLY);
