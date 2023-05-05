/*-
 * Copyright (c) 2011, Aleksandr Rybalko <ray@dlink.ua>
 * Copyright (c) 2023. Hiroki Mori
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_gpio.h"

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <sys/gpio.h>
#include "gpiobus_if.h"

#include <dev/gpio/gpiobusvar.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>
#include "spibus_if.h"

#ifdef FDT
#include "opt_platform.h"

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#ifdef	GPIO_SPI_DEBUG
#define	dprintf printf
#else
#define	dprintf(x, arg...)
#endif	/* GPIO_SPI_DEBUG */

//#undef FDT

struct gpio_spi_softc {
	device_t	sc_dev;
	device_t	sc_busdev;
	int		sc_freq;
	gpio_pin_t	sc_sclk;
	gpio_pin_t	sc_miso;
	gpio_pin_t	sc_mosi;
	gpio_pin_t	sc_cs0;
	gpio_pin_t	sc_cs1;
	gpio_pin_t	sc_cs2;
	gpio_pin_t	sc_cs3;
};

static void gpio_spi_chip_activate(struct gpio_spi_softc *, int);
static void gpio_spi_chip_deactivate(struct gpio_spi_softc *, int);

#ifdef FDT

#include <dev/ofw/ofw_bus.h>

static struct ofw_compat_data compat_data[] = {
	{"spi-gpio",  true}, /* Standard devicetree compat string */
	{NULL,        false}
};
/*
OFWBUS_PNP_INFO(compat_data);
SIMPLEBUS_PNP_INFO(compat_data);
*/

static phandle_t
gpiospi_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

static int
gpiospi_setup_fdt_pins(struct gpio_spi_softc *sc)
{
	phandle_t node;
	int err;

	node = ofw_bus_get_node(sc->sc_dev);

	/*
	 * Historically, we used the first two array elements of the gpios
	 * property.  The modern bindings specify separate scl-gpios and
	 * sda-gpios properties.  We cope with whichever is present.
	 */
	if ((err = gpio_pin_get_by_ofw_property(sc->sc_dev, node,
	    "gpio-sck", &sc->sc_sclk)) != 0) {
		device_printf(sc->sc_dev, "missing gpio-sck property\n");
		return (err);
	}
	if ((err = gpio_pin_get_by_ofw_property(sc->sc_dev, node,
	    "gpio-mosi", &sc->sc_mosi)) != 0) {
		device_printf(sc->sc_dev, "missing gpio-mosi property\n");
		return (err);
	}
	/* Handle no miso; we just never read back from the device */
	if ((err = gpio_pin_get_by_ofw_property(sc->sc_dev, node,
	    "gpio-miso", &sc->sc_miso)) != 0) {
		sc->sc_miso = NULL;
	}
	if ((err = gpio_pin_get_by_ofw_property(sc->sc_dev, node,
	    "cs-gpios", &sc->sc_cs0)) != 0) {
		device_printf(sc->sc_dev, "missing cs-gpio property\n");
		return (err);
	}
	sc->sc_cs1 = NULL;
	sc->sc_cs2 = NULL;
	sc->sc_cs3 = NULL;
	sc->sc_freq = 100000;

	return (0);
}
#endif /* FDT */

static int
gpio_spi_probe(device_t dev)
{

#ifdef FDT
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0) 
		return (ENXIO);
#endif

	device_set_desc(dev, "GPIO SPI bit-banging driver");
	return (0);
}

static void
gpio_delay(struct gpio_spi_softc *sc)
{
	int d;

	d = sc->sc_freq / 1000000;

	if (d != 0)
		DELAY(d);
}

static int
gpio_spi_attach(device_t dev)
{
	uint32_t value;
	struct gpio_spi_softc *sc;
	int err;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_busdev = device_get_parent(dev);

#ifdef FDT
	if (gpiospi_setup_fdt_pins(sc) != 0)
		return (ENXIO);
#else
	/* Required variables */
	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "sclk", &value))
		 return (ENXIO);
	if ((err = gpio_pin_get_by_child_index(dev, value,
	    &sc->sc_sclk)) != 0)
		return (err);

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "mosi", &value))
		 return (ENXIO);
	if ((err = gpio_pin_get_by_child_index(dev, value,
	    &sc->sc_mosi)) != 0)
		return (err);

	/* Handle no miso; we just never read back from the device */
	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "miso", &value))
		sc->sc_miso = NULL;
	else
		if ((err = gpio_pin_get_by_child_index(dev, value,
		    &sc->sc_miso)) != 0)
			return (err);

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "cs0", &value))
		 return (ENXIO);
	if ((err = gpio_pin_get_by_child_index(dev, value,
	    &sc->sc_cs0)) != 0)
		return (err);

	/* Optional variables */
	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "cs1", &value))
		sc->sc_cs1 = NULL;
	else
		if ((err = gpio_pin_get_by_child_index(dev, value,
		    &sc->sc_cs1)) != 0)
			return (err);

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "cs2", &value))
		sc->sc_cs2 = NULL;
	else
		if ((err = gpio_pin_get_by_child_index(dev, value,
		    &sc->sc_cs2)) != 0)
			return (err);

	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "cs3", &value))
		sc->sc_cs3 = NULL;
	else
		if ((err = gpio_pin_get_by_child_index(dev, value,
		    &sc->sc_cs3)) != 0)
			return (err);

	/* Default to 100KHz */
	if (resource_int_value(device_get_name(dev),
	    device_get_unit(dev), "freq", &value)) {
		value = 100000;
	}
	sc->sc_freq = value;
#endif
	if (bootverbose) {
		device_printf(dev, "frequency: %d Hz\n",
		    sc->sc_freq);
#ifdef FDT
		device_printf(dev,
		    "Use GPIO pins: sclk=%s:%d, mosi=%s:%d, miso=%s:%d, "
//		    "cs0=%s:%d, cs1=%s:%d, cs2=%s:%d, cs3=%s:%d\n",
		    "cs0=%s:%d\n",
		    device_get_nameunit(GPIO_GET_BUS(sc->sc_sclk->dev)),
		    sc->sc_sclk->pin,
		    device_get_nameunit(GPIO_GET_BUS(sc->sc_mosi->dev)),
		    sc->sc_mosi->pin,
		    device_get_nameunit(GPIO_GET_BUS(sc->sc_miso->dev)),
		    sc->sc_miso->pin,
		    device_get_nameunit(GPIO_GET_BUS(sc->sc_cs0->dev)),
		    sc->sc_cs0->pin);
#else
		device_printf(dev,
		    "Use GPIO pins: sclk=%d, mosi=%d, miso=%d, "
		    "cs0=%d, cs1=%d, cs2=%d, cs3=%d\n",
		    sc->sc_sclk->pin,
		    sc->sc_mosi->pin,
		    sc->sc_miso != NULL ? sc->sc_miso->pin : 0xff,
		    sc->sc_cs0->pin,
		    sc->sc_cs1 != NULL ? sc->sc_cs1->pin : 0xff,
		    sc->sc_cs2 != NULL ? sc->sc_cs2->pin : 0xff,
		    sc->sc_cs3 != NULL ? sc->sc_cs2->pin : 0xff);
#endif
	}

	/* Set directions */
	gpio_pin_setflags(sc->sc_sclk, GPIO_PIN_OUTPUT|GPIO_PIN_PULLDOWN);
	gpio_pin_setflags(sc->sc_mosi, GPIO_PIN_OUTPUT|GPIO_PIN_PULLDOWN);
	if (sc->sc_miso != NULL) {
		gpio_pin_setflags(sc->sc_miso,
		    GPIO_PIN_INPUT|GPIO_PIN_PULLDOWN);
	}

	gpio_pin_setflags(sc->sc_cs0, GPIO_PIN_OUTPUT|GPIO_PIN_PULLUP);

	if (sc->sc_cs1 != NULL)
		gpio_pin_setflags(sc->sc_cs1,
		    GPIO_PIN_OUTPUT|GPIO_PIN_PULLUP);
	if (sc->sc_cs2 != NULL)
		gpio_pin_setflags(sc->sc_cs2,
		    GPIO_PIN_OUTPUT|GPIO_PIN_PULLUP);
	if (sc->sc_cs3 != NULL)
		gpio_pin_setflags(sc->sc_cs3,
		    GPIO_PIN_OUTPUT|GPIO_PIN_PULLUP);

	gpio_spi_chip_deactivate(sc, -1);

	device_add_child(dev, "spibus", -1);
	return (bus_generic_attach(dev));
}

static int
gpio_spi_detach(device_t dev)
{

	return (0);
}

static void
gpio_spi_chip_activate(struct gpio_spi_softc *sc, int cs)
{

	/* called with locked gpiobus */
	switch (cs) {
	case 0:
		gpio_pin_set_active(sc->sc_cs0, 0);
		break;
	case 1:
		if (sc->sc_cs1 != NULL)
			gpio_pin_set_active(sc->sc_cs1, 0);
		break;
	case 2:
		if (sc->sc_cs2 != NULL)
			gpio_pin_set_active(sc->sc_cs2, 0);
		break;
	case 3:
		if (sc->sc_cs3 != NULL)
			gpio_pin_set_active(sc->sc_cs3, 0);
		break;
	default:
		device_printf(sc->sc_dev, "don't have CS%d\n", cs);
	}

	gpio_delay(sc);
}

static void
gpio_spi_chip_deactivate(struct gpio_spi_softc *sc, int cs)
{

	/* called wth locked gpiobus */
	/*
	 * Put CSx to high
	 */
	switch (cs) {
	case -1:
		/* All CS */
		gpio_pin_set_active(sc->sc_cs0, 1);
		if (sc->sc_cs1 == NULL) break;
		gpio_pin_set_active(sc->sc_cs1, 1);
		if (sc->sc_cs2 == NULL) break;
		gpio_pin_set_active(sc->sc_cs2, 1);
		if (sc->sc_cs3 == NULL) break;
		gpio_pin_set_active(sc->sc_cs3, 1);
		break;
	case 0:
		gpio_pin_set_active(sc->sc_cs0, 1);
		break;
	case 1:
		if (sc->sc_cs1 != NULL)
			gpio_pin_set_active(sc->sc_cs1, 1);
		break;
	case 2:
		if (sc->sc_cs2 != NULL)
			gpio_pin_set_active(sc->sc_cs2, 1);
		break;
	case 3:
		if (sc->sc_cs3 != NULL)
			gpio_pin_set_active(sc->sc_cs3, 1);
		break;
	default:
		device_printf(sc->sc_dev, "don't have CS%d\n", cs);
	}
}

static uint8_t
gpio_spi_txrx(struct gpio_spi_softc *sc, int cs, int mode, uint8_t data)
{
	uint32_t mask, out = 0;
	unsigned int bit;

	/* called with locked gpiobus */

	for (mask = 0x80; mask > 0; mask >>= 1) {
		if ((mode == SPIBUS_MODE_CPOL) ||
		    (mode == SPIBUS_MODE_CPHA)) {
			/* If mode 1 or 2 */

			/* first step */
			gpio_pin_set_active(
			    sc->sc_mosi, (data & mask)?1:0);
			gpio_pin_set_active(
			    sc->sc_sclk, 0);
			gpio_delay(sc);
			/* second step */
			if (sc->sc_miso != NULL) {
				gpio_pin_is_active(
				    sc->sc_miso, &bit);
				out |= bit?mask:0;
			}
			/* Data captured */
			gpio_delay(sc);
			gpio_pin_set_active(
			    sc->sc_sclk, 1);
			gpio_delay(sc);
		} else {
			/* If mode 0 or 3 */

			/* first step */
			gpio_pin_set_active(
			    sc->sc_mosi, (data & mask)?1:0);
			gpio_pin_set_active(
			    sc->sc_sclk, 1);
			gpio_delay(sc);
			/* second step */
			if (sc->sc_miso != NULL) {
				gpio_pin_is_active(
				    sc->sc_miso, &bit);
				out |= bit?mask:0;
			}
			 /* Data captured */
			gpio_delay(sc);
			gpio_pin_set_active(
			    sc->sc_sclk, 0);
			gpio_delay(sc);
		}
	}

	return (out & 0xff);
}

static int
gpio_spi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	struct gpio_spi_softc *sc;
	uint8_t *buf_in, *buf_out;
	struct spibus_ivar *devi = SPIBUS_IVAR(child);
	int i;

	sc = device_get_softc(dev);

	KASSERT(cmd->tx_cmd_sz == cmd->rx_cmd_sz, 
	    ("TX/RX command sizes should be equal"));
	KASSERT(cmd->tx_data_sz == cmd->rx_data_sz, 
	    ("TX/RX data sizes should be equal"));

	gpio_spi_chip_activate(sc, devi->cs);

	/* Preset pins */
	if ((devi->mode == SPIBUS_MODE_CPOL) ||
	    (devi->mode == SPIBUS_MODE_CPHA)) {
		gpio_pin_set_active(
		    sc->sc_sclk, 1);
	} else {
		gpio_pin_set_active(
		    sc->sc_sclk, 0);
	}

	/*
	 * Transfer command
	 */
	buf_out = (uint8_t *)cmd->tx_cmd;
	buf_in = (uint8_t *)cmd->rx_cmd;

	for (i = 0; i < cmd->tx_cmd_sz; i++)
		buf_in[i] = gpio_spi_txrx(sc, devi->cs, devi->mode, buf_out[i]);

	/*
	 * Receive/transmit data (depends on command)
	 */
	buf_out = (uint8_t *)cmd->tx_data;
	buf_in = (uint8_t *)cmd->rx_data;
	for (i = 0; i < cmd->tx_data_sz; i++)
		buf_in[i] = gpio_spi_txrx(sc, devi->cs, devi->mode, buf_out[i]);

	/* Return pins to mode default */
	if ((devi->mode == SPIBUS_MODE_CPOL) ||
	    (devi->mode == SPIBUS_MODE_CPHA)) {
		gpio_pin_set_active(
		    sc->sc_sclk, 1);
	} else {
		gpio_pin_set_active(
		    sc->sc_sclk, 0);
	}

	gpio_spi_chip_deactivate(sc, devi->cs);

	return (0);
}

static device_method_t gpio_spi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		gpio_spi_probe),
	DEVMETHOD(device_attach,	gpio_spi_attach),
	DEVMETHOD(device_detach,	gpio_spi_detach),

	DEVMETHOD(spibus_transfer,	gpio_spi_transfer),

#ifdef FDT
	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_node,	gpiospi_get_node),
#endif

	DEVMETHOD_END
};

static driver_t gpio_spi_driver = {
#ifdef FDT
	"spi",
#else
	"gpiospi",
#endif
	gpio_spi_methods,
	sizeof(struct gpio_spi_softc),
};

static devclass_t gpio_spi_devclass;

#ifdef FDT
DRIVER_MODULE(spi, simplebus, gpio_spi_driver, gpio_spi_devclass, 0, 0);
#else
DRIVER_MODULE(gpiospi, gpiobus, gpio_spi_driver, gpio_spi_devclass, 0, 0);
DRIVER_MODULE(spibus, gpiospi, spibus_driver, spibus_devclass, 0, 0);
MODULE_DEPEND(spi, gpiospi, 1, 1, 1);
MODULE_DEPEND(gpiobus, gpiospi, 1, 1, 1);
#endif
