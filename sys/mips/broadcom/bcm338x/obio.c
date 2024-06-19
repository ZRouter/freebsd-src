/*-
 * Copyright (c) 2016, Hiroki Mori
 * Copyright (c) 2009, Oleksandr Tymoshenko <gonzo@FreeBSD.org>
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
 */

#include "opt_platform.h"
#include "opt_bcm338x.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/pmc.h>
#include <sys/pmckern.h>

#include <machine/bus.h>
#include <machine/intr_machdep.h>

#include <mips/broadcom/bcm338x/obiovar.h>
#include <mips/broadcom/bcm338x/bcm3383reg.h>

#ifdef BCM338X_OBIO_DEBUG
#define dprintf printf
#else 
#define dprintf(x, arg...)
#endif  /* BCM338X_OBIO_DEBUG */

static int	obio_activate_resource(device_t, device_t, int, int,
		    struct resource *);
static device_t	obio_add_child(device_t, u_int, const char *, int);
static struct resource *
		obio_alloc_resource(device_t, device_t, int, int *, rman_res_t,
		    rman_res_t, rman_res_t, u_int);
static int	obio_attach(device_t);
static int	obio_deactivate_resource(device_t, device_t, int, int,
		    struct resource *);
static struct resource_list *
		obio_get_resource_list(device_t, device_t);
static void	obio_hinted_child(device_t, const char *, int);
static int	obio_filter(void *);
static int	obio_probe(device_t);
static int	obio_release_resource(device_t, device_t, int, int,
		    struct resource *);
static int	obio_setup_intr(device_t, device_t, struct resource *, int,
		    driver_filter_t *, driver_intr_t *, void *, void **);
static int	obio_teardown_intr(device_t, device_t, struct resource *,
		    void *);

static void 
obio_mask_irq(void *source)
{
	unsigned int irq = (unsigned int)source;
	uint32_t reg;

	reg = BCM_READ_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3 + 1));
	BCM_WRITE_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3 + 1),
	    reg & ~(1 << irq));
	reg = BCM_READ_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3));
	BCM_WRITE_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3),
	    reg & ~(1 << irq));
}

static void 
obio_unmask_irq(void *source)
{
	uint32_t reg;
	unsigned int irq = (unsigned int)source;

	reg = BCM_READ_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3));
	BCM_WRITE_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3),
	    reg | (1 << irq));
}

static int
obio_probe(device_t dev)
{
	device_set_desc(dev, "OBIO Bus bridge");

	return (0);
}

static int
obio_attach(device_t dev)
{
	struct obio_softc *sc = device_get_softc(dev);
	int rid = 0;
	int i, j;

	sc->obio_dev = dev;

	sc->obio_mem_rman.rm_type = RMAN_ARRAY;
	sc->obio_mem_rman.rm_descr = "OBIO memory window";

	if (rman_init(&sc->obio_mem_rman) != 0 ||
	    rman_manage_region(&sc->obio_mem_rman, 0x12000000, 0x15ffffff) != 0)
		panic("obio_attach: failed to set up memory rman");

	sc->obio_irq_rman.rm_type = RMAN_ARRAY;
	sc->obio_irq_rman.rm_descr = "OBIO IRQ";

	if (rman_init(&sc->obio_irq_rman) != 0 ||
	    rman_manage_region(&sc->obio_irq_rman, 
			OBIO_IRQ_BASE, OBIO_IRQ_END) != 0)
		panic("obio_attach: failed to set up IRQ rman");

	if ((sc->sc_misc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, 
	    RF_SHAREABLE | RF_ACTIVE)) == NULL) {
		device_printf(dev, "unable to allocate IRQ resource\n");
		return (ENXIO);
	}

	if ((bus_setup_intr(dev, sc->sc_misc_irq, INTR_TYPE_MISC,
	    obio_filter, NULL, sc, &sc->sc_misc_ih))) {
		device_printf(dev,
		    "WARNING: unable to register interrupt handler\n");
		return (ENXIO);
	}

	/* mask all misc interrupt */
	BCM_WRITE_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3), 0);

	obio_unmask_irq(INTERRUPT_ID_USB_EHCI);

	/* USB init refer bcm93383-platform-devs.c brcm_chip_usb_init() */
	BCM_WRITE_REG(BCM3383_INTC_BASE + 0x0c, (1 << 7) |
	    BCM_READ_REG(BCM3383_INTC_BASE + 0x0c));
	BCM_WRITE_REG(BCM3383_INTC_BASE + 0x04, (1 << 7) |
	    BCM_READ_REG(BCM3383_INTC_BASE + 0x04));

	BCM_WRITE_REG(BCM3383_USBCTL_BASE, (1 << 6) |
	    BCM_READ_REG(BCM3383_USBCTL_BASE));
	for (i = 0; i < 1000; ++i) ;
	BCM_WRITE_REG(BCM3383_USBCTL_BASE, ~(1 << 6) &
	    BCM_READ_REG(BCM3383_USBCTL_BASE));

	BCM_WRITE_REG(BCM3383_USBCTL_BASE, (1 << 4) |
	    BCM_READ_REG(BCM3383_USBCTL_BASE));
	BCM_WRITE_REG(BCM3383_USBCTL_BASE + 0xc, 9 |
	    BCM_READ_REG(BCM3383_USBCTL_BASE + 0xc));

	BCM_WRITE_REG(BCM3383_USBCTL_BASE + 0x04, 0x512750c0);
#if 0
	device_printf(dev, "Broadcom CHIP ID: %x\n",
	    BCM_READ_REG(BCM3383_INTC_BASE));

	// Timer intr test
//	int reg = BCM_READ_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3));
//	BCM_WRITE_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3), reg | 1);
//	obio_unmask_irq(INTERRUPT_ID_TIMER);

	/* Stop Timer */
	BCM_WRITE_REG(BCM3383_TIMER_BASE + 0x04, 0);
	/* Interrupt Enable */
	BCM_WRITE_REG(BCM3383_TIMER_BASE + 0x00, (1 << 8));
	/* Start Timer */
	BCM_WRITE_REG(BCM3383_TIMER_BASE + 0x04, (1 << 31) | 0xffffff);

	int off = BCM3383_USBCTL_BASE;
//	int off = BCM3383_INTC_BASE;
	for (i = 0; i < 16 ; ++i) {
		printf("%08x ", i * 0x10);
		for (j = 0; j < 4 ; ++j) {
			printf("%08x ",
			    BCM_READ_REG(off + i * 0x10 + j * 4));
		}
		printf("\n");
	}
#endif

	bus_generic_probe(dev);
	bus_enumerate_hinted_children(dev);
	bus_generic_attach(dev);

	return (0);
}

static struct resource *
obio_alloc_resource(device_t bus, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct obio_softc		*sc = device_get_softc(bus);
	struct obio_ivar		*ivar = device_get_ivars(child);
	struct resource			*rv;
	struct resource_list_entry	*rle;
	struct rman			*rm;
	int				 isdefault, needactivate, passthrough;

	isdefault = (RMAN_IS_DEFAULT_RANGE(start, end));
	needactivate = flags & RF_ACTIVE;
	/*
	 * Pass memory requests to nexus device
	 */
	passthrough = (device_get_parent(child) != bus);
	rle = NULL;

	dprintf("%s: entry (%p, %p, %d, %d, %p, %p, %jd, %d)\n",
	    __func__, bus, child, type, *rid, (void *)(intptr_t)start,
	    (void *)(intptr_t)end, count, flags);

	if (passthrough)
		return (BUS_ALLOC_RESOURCE(device_get_parent(bus), child, type,
		    rid, start, end, count, flags));

	/*
	 * If this is an allocation of the "default" range for a given RID,
	 * and we know what the resources for this device are (ie. they aren't
	 * maintained by a child bus), then work out the start/end values.
	 */

	if (isdefault) {
		rle = resource_list_find(&ivar->resources, type, *rid);
		if (rle == NULL) {
			return (NULL);
		}

		if (rle->res != NULL) {
			panic("%s: resource entry is busy", __func__);
		}
		start = rle->start;
		end = rle->end;
		count = rle->count;

		dprintf("%s: default resource (%p, %p, %jd)\n",
		    __func__, (void *)(intptr_t)start,
		    (void *)(intptr_t)end, count);
	}

	switch (type) {
	case SYS_RES_IRQ:
		rm = &sc->obio_irq_rman;
		break;
	case SYS_RES_MEMORY:
		rm = &sc->obio_mem_rman;
		break;
	default:
		printf("%s: unknown resource type %d\n", __func__, type);
		return (0);
	}

	rv = rman_reserve_resource(rm, start, end, count, flags, child);
	if (rv == NULL) {
		printf("%s: could not reserve resource %d\n", __func__, type);
		return (0);
	}

	rman_set_rid(rv, *rid);

	if (needactivate) {
		if (bus_activate_resource(child, type, *rid, rv)) {
			printf("%s: could not activate resource\n", __func__);
			rman_release_resource(rv);
			return (0);
		}
	}

	return (rv);
}

static int
obio_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{

	/* XXX: should we mask/unmask IRQ here? */
	return (BUS_ACTIVATE_RESOURCE(device_get_parent(bus), child,
		type, rid, r));
}

static int
obio_deactivate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{

	/* XXX: should we mask/unmask IRQ here? */
	return (BUS_DEACTIVATE_RESOURCE(device_get_parent(bus), child,
		type, rid, r));
}

static int
obio_release_resource(device_t dev, device_t child, int type,
    int rid, struct resource *r)
{
	struct resource_list *rl;
	struct resource_list_entry *rle;

	rl = obio_get_resource_list(dev, child);
	if (rl == NULL)
		return (EINVAL);
	rle = resource_list_find(rl, type, rid);
	if (rle == NULL)
		return (EINVAL);
	rman_release_resource(r);
	rle->res = NULL;

	return (0);
}


static int
obio_setup_intr(device_t bus, device_t child, struct resource *ires,
		int flags, driver_filter_t *filt, driver_intr_t *handler,
		void *arg, void **cookiep)
{
	struct obio_softc *sc = device_get_softc(bus);
	int error;
	int irq;
	struct intr_event *event;

	irq = rman_get_start(ires);

	if (irq > OBIO_IRQ_END)
		panic("%s: bad irq %d", __func__, irq);

	event = sc->sc_eventstab[irq];
	if (event == NULL) {
		error = intr_event_create(&event, (void *)irq, 0, irq, 
		    obio_mask_irq, obio_unmask_irq,
		    NULL, NULL,
		    "obio intr%d:", irq);

		if (error == 0) {
			sc->sc_eventstab[irq] = event;
			sc->sc_intr_counter[irq] =
			    mips_intrcnt_create(event->ie_name);
		}
		else
			return (error);
	}

	intr_event_add_handler(event, device_get_nameunit(child), filt,
	    handler, arg, intr_priority(flags), flags, cookiep);
	mips_intrcnt_setname(sc->sc_intr_counter[irq], event->ie_fullname);

	obio_unmask_irq((void*)irq);

	return (0);
}

static int
obio_teardown_intr(device_t dev, device_t child, struct resource *ires,
    void *cookie)
{
	struct obio_softc *sc = device_get_softc(dev);
	int irq, result;

	irq = rman_get_start(ires);
	if (irq > OBIO_IRQ_END)
		panic("%s: bad irq %d", __func__, irq);

	if (sc->sc_eventstab[irq] == NULL)
		panic("Trying to teardown unoccupied IRQ");

	obio_mask_irq((void*)irq);

	result = intr_event_remove_handler(cookie);
	if (!result)
		sc->sc_eventstab[irq] = NULL;

	return (result);
}

static int
obio_filter(void *arg)
{
	struct obio_softc *sc = arg;
	struct intr_event *event;
	uint32_t irq, intr;

	intr = BCM_READ_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3 + 1)) &
	    BCM_READ_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3));

	while ((irq = fls(intr)) != 0) {
		irq--;
		intr &= ~(1u << irq);

		event = sc->sc_eventstab[irq];
		if (!event || CK_SLIST_EMPTY(&event->ie_handlers)) {
			printf("Stray OBIO IRQ %d\n", irq);
			obio_mask_irq(irq);
			continue;
		}

		intr_event_handle(event, PCPU_GET(curthread)->td_intr_frame);
		mips_intrcnt_inc(sc->sc_intr_counter[irq]);
	}

	BCM_WRITE_REG(BCM3383_INTC_BASE + 4 * (12 + 2 * 3 + 1), 0);

	return (FILTER_HANDLED);
}

static void
obio_hinted_child(device_t bus, const char *dname, int dunit)
{
	device_t		child;
	long			maddr;
	int			msize;
	int			irq;
	int			result;
	int			mem_hints_count;

	child = BUS_ADD_CHILD(bus, 0, dname, dunit);

	/*
	 * Set hard-wired resources for hinted child using
	 * specific RIDs.
	 */
	mem_hints_count = 0;
	if (resource_long_value(dname, dunit, "maddr", &maddr) == 0)
		mem_hints_count++;
	if (resource_int_value(dname, dunit, "msize", &msize) == 0)
		mem_hints_count++;

	/* check if all info for mem resource has been provided */
	if ((mem_hints_count > 0) && (mem_hints_count < 2)) {
		printf("Either maddr or msize hint is missing for %s%d\n",
		    dname, dunit);
	} else if (mem_hints_count) {
		result = bus_set_resource(child, SYS_RES_MEMORY, 0,
		    maddr, msize);
		if (result != 0)
			device_printf(bus, 
			    "warning: bus_set_resource() failed\n");
	}

	if (resource_int_value(dname, dunit, "irq", &irq) == 0) {
		result = bus_set_resource(child, SYS_RES_IRQ, 0, irq, 1);
		if (result != 0)
			device_printf(bus,
			    "warning: bus_set_resource() failed\n");
	}
}

static device_t
obio_add_child(device_t bus, u_int order, const char *name, int unit)
{
	device_t		child;
	struct obio_ivar	*ivar;

	ivar = malloc(sizeof(struct obio_ivar), M_DEVBUF, M_WAITOK | M_ZERO);
	if (ivar == NULL) {
		printf("Failed to allocate ivar\n");
		return (0);
	}
	resource_list_init(&ivar->resources);

	child = device_add_child_ordered(bus, order, name, unit);
	if (child == NULL) {
		printf("Can't add child %s%d ordered\n", name, unit);
		return (0);
	}

	device_set_ivars(child, ivar);

	return (child);
}

/*
 * Helper routine for bus_generic_rl_get_resource/bus_generic_rl_set_resource
 * Provides pointer to resource_list for these routines
 */
static struct resource_list *
obio_get_resource_list(device_t dev, device_t child)
{
	struct obio_ivar *ivar;

	ivar = device_get_ivars(child);
	return (&(ivar->resources));
}

static device_method_t obio_methods[] = {
	DEVMETHOD(bus_activate_resource,	obio_activate_resource),
	DEVMETHOD(bus_add_child,		obio_add_child),
	DEVMETHOD(bus_alloc_resource,		obio_alloc_resource),
	DEVMETHOD(bus_deactivate_resource,	obio_deactivate_resource),
	DEVMETHOD(bus_get_resource_list,	obio_get_resource_list),
	DEVMETHOD(bus_hinted_child,		obio_hinted_child),
	DEVMETHOD(bus_release_resource,		obio_release_resource),
	DEVMETHOD(device_attach,		obio_attach),
	DEVMETHOD(device_probe,			obio_probe),
	DEVMETHOD(bus_get_resource,		bus_generic_rl_get_resource),
	DEVMETHOD(bus_set_resource,		bus_generic_rl_set_resource),

	DEVMETHOD(bus_teardown_intr,		obio_teardown_intr),
	DEVMETHOD(bus_setup_intr,		obio_setup_intr),

	DEVMETHOD_END
};

static driver_t obio_driver = {
	"obio",
	obio_methods,
	sizeof(struct obio_softc),
};
static devclass_t obio_devclass;

EARLY_DRIVER_MODULE(obio, nexus, obio_driver, obio_devclass, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
