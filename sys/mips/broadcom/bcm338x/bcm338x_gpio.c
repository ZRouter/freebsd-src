/*-
 * Copyright (c) 2016, Hiroki Mori
 * Copyright (c) 2009, Oleksandr Tymoshenko <gonzo@FreeBSD.org>
 * Copyright (c) 2009, Luiz Otavio O Souza. 
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

/*
 * GPIO driver for BCM338X 
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
/*
#include <mips/atheros/ar531x/bcm338xreg.h>
#include <mips/atheros/ar531x/bcm338x_cpudef.h>
*/
#include <dev/gpio/gpiobusvar.h>
#include <mips/broadcom/bcm338x/bcm338x_gpioreg.h>
#include <mips/broadcom/bcm338x/bcm338x_gpiovar.h>

#include "gpio_if.h"

#define	DEFAULT_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)

/*
 * Helpers
 */
static void bcm338x_gpio_function_enable(struct bcm338x_gpio_softc *sc, 
    uint32_t mask);
static void bcm338x_gpio_function_disable(struct bcm338x_gpio_softc *sc, 
    uint32_t mask);
static void bcm338x_gpio_pin_configure(struct bcm338x_gpio_softc *sc, 
    struct gpio_pin *pin, uint32_t flags);

/*
 * Driver stuff
 */
static int bcm338x_gpio_probe(device_t dev);
static int bcm338x_gpio_attach(device_t dev);
static int bcm338x_gpio_detach(device_t dev);
static int bcm338x_gpio_filter(void *arg);
static void bcm338x_gpio_intr(void *arg);

/*
 * GPIO interface
 */
static device_t bcm338x_gpio_get_bus(device_t);
static int bcm338x_gpio_pin_max(device_t dev, int *maxpin);
static int bcm338x_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps);
static int bcm338x_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t
    *flags);
static int bcm338x_gpio_pin_getname(device_t dev, uint32_t pin, char *name);
static int bcm338x_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags);
static int bcm338x_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value);
static int bcm338x_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val);
static int bcm338x_gpio_pin_toggle(device_t dev, uint32_t pin);

/*
 * Enable/disable the GPIO function control space.
 *
 * This is primarily for the BCM338X, which has SPI CS1/CS2, UART, SLIC, I2S
 * as GPIO pin options.
 */
static void
bcm338x_gpio_function_enable(struct bcm338x_gpio_softc *sc, uint32_t mask)
{
//		GPIO_SET_BITS(sc, BCM338X_GPIO_FUNCTION, mask);
}

static void
bcm338x_gpio_function_disable(struct bcm338x_gpio_softc *sc, uint32_t mask)
{
//		GPIO_CLEAR_BITS(sc, BCM338X_GPIO_FUNCTION, mask);
}

static void
bcm338x_gpio_pin_configure(struct bcm338x_gpio_softc *sc, struct gpio_pin *pin,
    unsigned int flags)
{
	uint32_t mask;

	mask = 1 << pin->gp_pin;

	/*
	 * Manage input/output
	 */
#if 0
	if (flags & (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT)) {
		pin->gp_flags &= ~(GPIO_PIN_INPUT|GPIO_PIN_OUTPUT);
		if (flags & GPIO_PIN_OUTPUT) {
			pin->gp_flags |= GPIO_PIN_OUTPUT;
			GPIO_SET_BITS(sc, ar531x_gpio_cr(), mask);
		}
		else {
			pin->gp_flags |= GPIO_PIN_INPUT;
			GPIO_CLEAR_BITS(sc, ar531x_gpio_cr(), mask);
		}
	}
#endif
}

static device_t
bcm338x_gpio_get_bus(device_t dev)
{
	struct bcm338x_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
bcm338x_gpio_pin_max(device_t dev, int *maxpin)
{

//	*maxpin = ar531x_gpio_pins() - 1;
	*maxpin = 32;
	return (0);
}

static int
bcm338x_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	GPIO_LOCK(sc);
	*caps = sc->gpio_pins[i].gp_caps;
	GPIO_UNLOCK(sc);

	return (0);
}

static int
bcm338x_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);
	int i;
	int dir;

#if 0
	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	dir = GPIO_READ(sc, ar531x_gpio_cr()) & (1 << pin);

	*flags = dir ? GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;

/*
	GPIO_LOCK(sc);
	*flags = sc->gpio_pins[i].gp_flags;
	GPIO_UNLOCK(sc);
*/
#endif

	return (0);
}

static int
bcm338x_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	GPIO_LOCK(sc);
	memcpy(name, sc->gpio_pins[i].gp_name, GPIOMAXNAME);
	GPIO_UNLOCK(sc);

	return (0);
}

static int
bcm338x_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	int i;
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	bcm338x_gpio_pin_configure(sc, &sc->gpio_pins[i], flags);

	return (0);
}

static int
bcm338x_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);
	uint32_t state;

#if 0
	state = GPIO_READ(sc, ar531x_gpio_do());

	if(value == 1) {
		state |= (1 << pin);
	} else {
		state &= ~(1 << pin);
	}

	GPIO_WRITE(sc, ar531x_gpio_do(), state);
#endif

	return (0);
}

static int
bcm338x_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);
	int i;
#if 0

	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	*val = (GPIO_READ(sc, ar531x_gpio_di()) & (1 << pin)) ? 1 : 0;
#endif

	return (0);
}

static int
bcm338x_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	int res, i;
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);

#if 0
	for (i = 0; i < sc->gpio_npins; i++) {
		if (sc->gpio_pins[i].gp_pin == pin)
			break;
	}

	if (i >= sc->gpio_npins)
		return (EINVAL);

	res = (GPIO_READ(sc, ar531x_gpio_do()) & (1 << pin)) ? 1 : 0;
	if (res)
		GPIO_CLEAR_BITS(sc, ar531x_gpio_do(), pin);
	else
		GPIO_SET_BITS(sc, ar531x_gpio_do(), pin);
#endif

	return (0);
}

static int
bcm338x_gpio_filter(void *arg)
{

	/* TODO: something useful */
	return (FILTER_STRAY);
}



static void
bcm338x_gpio_intr(void *arg)
{
	struct bcm338x_gpio_softc *sc = arg;
	GPIO_LOCK(sc);
	/* TODO: something useful */
	GPIO_UNLOCK(sc);
}

static int
bcm338x_gpio_probe(device_t dev)
{

	device_set_desc(dev, "BCM338X GPIO driver");
	return (0);
}

static int
bcm338x_gpio_attach(device_t dev)
{
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);
	int i, j, maxpin;
	int mask, pinon;
	uint32_t oe;

	KASSERT((device_get_unit(dev) == 0),
	    ("bcm338x_gpio: Only one gpio module supported"));

	mtx_init(&sc->gpio_mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Map control/status registers. */
	sc->gpio_mem_rid = 0;
	sc->gpio_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->gpio_mem_rid, RF_ACTIVE);

	if (sc->gpio_mem_res == NULL) {
		device_printf(dev, "couldn't map memory\n");
		bcm338x_gpio_detach(dev);
		return (ENXIO);
	}

	device_printf(dev, "DieRev: %x\n", GPIO_READ(sc, DIEREVID));
	for (i = 0; i < 32 ; ++i) {
		printf("%08x ", i * 0x10);
		for (j = 0; j < 4 ; ++j) {
			printf("%08x ",
			    bus_read_4(sc->gpio_mem_res, i * 0x10 + j * 4));
		}
		printf("\n");
	}

#if 0
	if ((sc->gpio_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, 
	    &sc->gpio_irq_rid, RF_SHAREABLE | RF_ACTIVE)) == NULL) {
		device_printf(dev, "unable to allocate IRQ resource\n");
		bcm338x_gpio_detach(dev);
		return (ENXIO);
	}

	if ((bus_setup_intr(dev, sc->gpio_irq_res, INTR_TYPE_MISC, 
	    bcm338x_gpio_filter, bcm338x_gpio_intr, sc, &sc->gpio_ih))) {
		device_printf(dev,
		    "WARNING: unable to register interrupt handler\n");
		bcm338x_gpio_detach(dev);
		return (ENXIO);
	}

	sc->dev = dev;

	/* Enable function bits that are required */
	if (resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "function_set", &mask) == 0) {
		device_printf(dev, "function_set: 0x%x\n", mask);
		bcm338x_gpio_function_enable(sc, mask);
	}
	/* Disable function bits that are required */
	if (resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "function_clear", &mask) == 0) {
		device_printf(dev, "function_clear: 0x%x\n", mask);
		bcm338x_gpio_function_disable(sc, mask);
	}

	/* Initialise all pins specified in the mask, up to the pin count */
	(void) bcm338x_gpio_pin_max(dev, &maxpin);
	if (resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "pinmask", &mask) != 0)
		mask = 0;
	if (resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "pinon", &pinon) != 0)
		pinon = 0;
	device_printf(dev, "gpio pinmask=0x%x\n", mask);
	for (j = 0; j <= maxpin; j++) {
		if ((mask & (1 << j)) == 0)
			continue;
		sc->gpio_npins++;
	}

	/* Iniatilize the GPIO pins, keep the loader settings. */
	oe = GPIO_READ(sc, ar531x_gpio_cr());
	sc->gpio_pins = malloc(sizeof(struct gpio_pin) * sc->gpio_npins,
	    M_DEVBUF, M_WAITOK | M_ZERO);
	for (i = 0, j = 0; j <= maxpin; j++) {
		if ((mask & (1 << j)) == 0)
			continue;
		snprintf(sc->gpio_pins[i].gp_name, GPIOMAXNAME,
		    "pin %d", j);
		sc->gpio_pins[i].gp_pin = j;
		sc->gpio_pins[i].gp_caps = DEFAULT_CAPS;
		if (oe & (1 << j))
			sc->gpio_pins[i].gp_flags = GPIO_PIN_OUTPUT;
		else
			sc->gpio_pins[i].gp_flags = GPIO_PIN_INPUT;
		i++;
	}S
#endif

#if 0
	/* Turn on the hinted pins. */
	for (i = 0; i < sc->gpio_npins; i++) {
		j = sc->gpio_pins[i].gp_pin;
		if ((pinon & (1 << j)) != 0) {
			bcm338x_gpio_pin_setflags(dev, j, GPIO_PIN_OUTPUT);
			bcm338x_gpio_pin_set(dev, j, 1);
		}
	}

	/*
	 * Search through the function hints, in case there's some
	 * overrides such as LNA control.
	 *
	 * hint.gpio.X.func.<pin>.gpiofunc=<func value>
	 * hint.gpio.X.func.<pin>.gpiomode=1 (for output, default low)
	 */
	for (i = 0; i <= maxpin; i++) {
		char buf[32];
		int gpiofunc, gpiomode;

		snprintf(buf, 32, "func.%d.gpiofunc", i);
		if (resource_int_value(device_get_name(dev),
		    device_get_unit(dev),
		    buf,
		    &gpiofunc) != 0)
			continue;
		/* Get the mode too */
		snprintf(buf, 32, "func.%d.gpiomode", i);
		if (resource_int_value(device_get_name(dev),
		    device_get_unit(dev),
		    buf,
		    &gpiomode) != 0)
			continue;

		/* We only handle mode=1 for now */
		if (gpiomode != 1)
			continue;

		device_printf(dev, "%s: GPIO %d: func=%d, mode=%d\n",
		    __func__,
		    i,
		    gpiofunc,
		    gpiomode);

		/* Set output (bit == 0) */
		oe = GPIO_READ(sc, ar531x_gpio_cr());
		oe &= ~ (1 << i);
		GPIO_WRITE(sc, ar531x_gpio_cr(), oe);

		/* Set pin value = 0, so it stays low by default */
		oe = GPIO_READ(sc, ar531x_gpio_do());
		oe &= ~ (1 << i);
		GPIO_WRITE(sc, ar531x_gpio_do(), oe);

		/* Finally: Set the output config */
//		bcm338x_gpio_ouput_configure(i, gpiofunc);
	}
#endif

	sc->busdev = gpiobus_attach_bus(dev);
	if (sc->busdev == NULL) {
		bcm338x_gpio_detach(dev);
		return (ENXIO);
	}

	return (0);
}

static int
bcm338x_gpio_detach(device_t dev)
{
	struct bcm338x_gpio_softc *sc = device_get_softc(dev);

	KASSERT(mtx_initialized(&sc->gpio_mtx), ("gpio mutex not initialized"));

	gpiobus_detach_bus(dev);
	if (sc->gpio_ih)
		bus_teardown_intr(dev, sc->gpio_irq_res, sc->gpio_ih);
	if (sc->gpio_irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, sc->gpio_irq_rid,
		    sc->gpio_irq_res);
	if (sc->gpio_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->gpio_mem_rid,
		    sc->gpio_mem_res);
	if (sc->gpio_pins)
		free(sc->gpio_pins, M_DEVBUF);
	mtx_destroy(&sc->gpio_mtx);

	return(0);
}

static device_method_t bcm338x_gpio_methods[] = {
	DEVMETHOD(device_probe, bcm338x_gpio_probe),
	DEVMETHOD(device_attach, bcm338x_gpio_attach),
	DEVMETHOD(device_detach, bcm338x_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus, bcm338x_gpio_get_bus),
	DEVMETHOD(gpio_pin_max, bcm338x_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname, bcm338x_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags, bcm338x_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps, bcm338x_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags, bcm338x_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get, bcm338x_gpio_pin_get),
	DEVMETHOD(gpio_pin_set, bcm338x_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle, bcm338x_gpio_pin_toggle),
	{0, 0},
};

static driver_t bcm338x_gpio_driver = {
	"gpio",
	bcm338x_gpio_methods,
	sizeof(struct bcm338x_gpio_softc),
};
static devclass_t bcm338x_gpio_devclass;

DRIVER_MODULE(bcm338x_gpio, obio, bcm338x_gpio_driver, bcm338x_gpio_devclass, 0, 0);
