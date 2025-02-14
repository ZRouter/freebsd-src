/*-
 * Copyright 2025 Hiroki Mori
 * All rights reserved.
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
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timetc.h>
#include <sys/timeet.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/hwfunc.h>

#include <dev/extres/clk/clk.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define TMR0LOAD    0x14  /* Timer0 Load Value */
#define TMR0VAL     0x18  /* Timer0 Counter Value */
#define TMR0CTL     0x10  /* Timer0 Control */

struct mtk_timer_softc {
	device_t		dev;
	struct resource	*	res[1];
	void *			ih_cookie;
	struct timecounter	tc;
};

static struct resource_spec mtk_timer_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

/*
 * devclass_get_device / device_get_softc could be used
 * to dynamically locate this, however the timers are a
 * required device which can't be unloaded so there's
 * no need for the overhead.
 */
static struct mtk_timer_softc *mtk_timer_sc = NULL;

#define	CSR_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], reg, (val))
#define	CSR_READ_4(sc, reg)		bus_read_4((sc)->res[0], reg)

static unsigned
mtk_get_timecount(struct timecounter *tc)
{
	struct mtk_timer_softc *sc =
	    (struct mtk_timer_softc *)tc->tc_priv;

	return 0xffff - CSR_READ_4(sc, TMR0VAL);
}

static int
mtk_timer_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "ralink,mt7621-timer"))
		return (ENXIO);

	device_set_desc(dev, "Ralink timer");

	return (BUS_PROBE_DEFAULT);
}

static int
mtk_timer_attach(device_t dev)
{
	struct mtk_timer_softc *sc = device_get_softc(dev);
	pcell_t counter_freq;
	clk_t clk;
	uint32_t reg;

	/* There should be exactly one instance. */
	if (mtk_timer_sc != NULL)
		return (ENXIO);

	sc->dev = dev;

	if (bus_alloc_resources(dev, mtk_timer_spec, sc->res)) {
		device_printf(dev, "can not allocate resources for device\n");
		return (ENXIO);
	}

	counter_freq = 100 * 1000;

	CSR_WRITE_4(sc, TMR0LOAD, 0xffff);
	/* start timer */
	reg = CSR_READ_4(sc, TMR0CTL);
	reg &= 0x0000ffff;
	reg |= (10 << 16);
	reg |= ((1<<7) | (1<<4));
	CSR_WRITE_4(sc, TMR0CTL, reg);

	sc->tc.tc_get_timecount = mtk_get_timecount;
	sc->tc.tc_name = "MTK TIMER";
	sc->tc.tc_frequency = counter_freq;
	sc->tc.tc_counter_mask = 0xffff;
	sc->tc.tc_quality = 1000;
	sc->tc.tc_priv = sc;

	tc_init(&sc->tc);

	/* Now when tc is initialized, allow DELAY to find it */
	mtk_timer_sc = sc;

	return (0);
}

static int
mtk_timer_detach(device_t dev)
{

	return (EBUSY);
}

static device_method_t mtk_timer_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_timer_probe),
	DEVMETHOD(device_attach,	mtk_timer_attach),
	DEVMETHOD(device_detach,	mtk_timer_detach),

	DEVMETHOD_END
};

static driver_t mtk_timer_driver = {
	"timer",
	mtk_timer_methods,
	sizeof(struct mtk_timer_softc),
};

static devclass_t mtk_timer_devclass;

EARLY_DRIVER_MODULE(timer, simplebus, mtk_timer_driver,
    mtk_timer_devclass, 0, 0, BUS_PASS_TIMER);

