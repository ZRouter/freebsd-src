/*-
 * Copyright (c) 2017 Hiroki Mori
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
 * 3. Neither the name of RMI Corporation, nor the names of its contributors,
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include "opt_platform.h"

/*
 * This code do not test 
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include "iicbus_if.h"

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mediatek/mtk_iic.h>

/*
 * register space access macros
 */

#define	MTK_IIC_LOCK(sc)		mtx_lock_spin(&(sc)->mtx)
#define	MTK_IIC_UNLOCK(sc)		mtx_unlock_spin(&(sc)->mtx)
#define	MTK_IIC_LOCK_INIT(sc)		\
    mtx_init(&(sc)->mtx, "mtk_iic", "mtk_iic", MTX_DEF)
#define	MTK_IIC_LOCK_DESTROY(sc)	mtx_destroy(&(sc)->mtx)

#define	I2CREG_WRITE(sc, reg, val) do {	\
		bus_write_4(sc->sc_mem_res, (reg), (val));	\
	} while (0)

#define	I2CREG_READ(sc, reg)	bus_read_4(sc->sc_mem_res, (reg))

/*
 * Device methods
 */
static int mtk_iic_probe(device_t);
static int mtk_iic_attach(device_t);
static int mtk_iic_detach(device_t);

static int mtk_iic_start(device_t dev, u_char slave, int timeout);
static int mtk_iic_stop(device_t dev);
static int mtk_iic_read(device_t dev, char *buf, int len, int *read, int last, int delay);
static int mtk_iic_write(device_t dev, const char *buf, int len, int *sent, int timeout);
static int mtk_iic_transfer(device_t bus, struct iic_msg *msgs, uint32_t nmsgs);
#ifdef notyet
static int mtk_iic_repeated_start(device_t dev, u_char slave, int timeout);
#endif

static struct ofw_compat_data compat_data[] = {
	{ "ralink,rt2880-i2c",  1 },
	{ "ralink,rt3050-i2c",  1 },
	{ NULL,                 0 }
};

struct mtk_iic_softc {
	device_t dev;
	struct resource *sc_mem_res;
	device_t iicbus;
	unsigned int clkdiv;
	int sc_started;
	uint8_t i2cdev_addr;
	struct mtx mtx;
	int last_slave;
};

static int
mtk_iic_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return(ENXIO);

	device_set_desc(dev, "MTK I2C Controller");

	return (0);
}

static int
mtk_iic_attach(device_t dev)
{
	int rid;
	struct mtk_iic_softc *sc;
	phandle_t node;

	sc = device_get_softc(dev);

	MTK_IIC_LOCK_INIT(sc);

	node = ofw_bus_get_node(dev);
	sc->dev = dev;
	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "Could not map memory\n");
		return (ENXIO);
	}

	if (OF_getencprop(node, "clock-div", &sc->clkdiv, 
	    sizeof(sc->clkdiv)) <= 0) {
		sc->clkdiv = RT2880_CLKDIV_VALUE;
	}

	I2CREG_WRITE(sc, RA_I2C_CLKDIV, sc->clkdiv);

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "cannot add iicbus child device\n");
		return (ENXIO);
	}

	sc->sc_started = 0;
	sc->last_slave = -1;

	return (bus_generic_attach(dev));
}

static int
mtk_iic_detach(device_t dev)
{
	struct mtk_iic_softc *sc;
	int error;

	sc = device_get_softc(dev);

	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);

	MTK_IIC_LOCK_DESTROY(sc);
	if (sc->iicbus &&
	    (error = device_delete_child(dev, sc->iicbus)) != 0)
		return error;

	return (0);
}

static int 
mtk_iic_start(device_t dev, u_char slave, int timeout)
{
	int error;
	struct mtk_iic_softc *sc;
	uint32_t r;
	int i;

	sc = device_get_softc(dev);

	if (sc->sc_started == 1)
		return (IIC_EBUSERR);

	MTK_IIC_LOCK(sc);
	error = IIC_NOERR;
	sc->i2cdev_addr = (slave >> 1);

	/* This is work around for Ralink I2C device.
	 * Ralink I2C device can't send one byte. Most smoll is two byte.
	 * Then We send only first time dummy write
	 */

	if (sc->last_slave != sc->i2cdev_addr) {
		r = I2C_CONFIG_ADDRLEN(I2C_CONFIG_ADDRLEN_8) |
		    I2C_CONFIG_DEVADLEN(I2C_CONFIG_DEVADLEN_7) |
		    I2C_CONFIG_ADDRDIS;
		I2CREG_WRITE(sc, RA_I2C_CONFIG, r);

		/* dmmy write */
		I2CREG_WRITE(sc, RA_I2C_DEVADDR, sc->i2cdev_addr);
		I2CREG_WRITE(sc, RA_I2C_BYTECNT, 0);
		I2CREG_WRITE(sc, RA_I2C_DATAOUT, slave);
		I2CREG_WRITE(sc, RA_I2C_STARTXFR, I2C_OP_WRITE);

		for (i = 0; i < max_ee_busy_loop; i++) {
			r = I2CREG_READ(sc, RA_I2C_STATUS);
			if (r & I2C_STATUS_ACKERR) {
				error = IIC_ENOACK;
			}
			if ((r & I2C_STATUS_BUSY) == 0) {
				break;
			}
		}
		if (i == max_ee_busy_loop) {
			error = IIC_EBUSERR;
		}
		if (error == IIC_NOERR) {
			sc->last_slave = sc->i2cdev_addr;
		}
	}

	if (error == IIC_NOERR) {
		sc->sc_started = 1;
	} else {
		MTK_IIC_UNLOCK(sc);
	}


	return (error);
}

static int 
mtk_iic_stop(device_t dev)
{
	int error;
	struct mtk_iic_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_started) {
		MTK_IIC_UNLOCK(sc);
		error = IIC_NOERR;
		sc->sc_started = 0;
	} else {
		error = IIC_EBUSERR;
	}
	return error;

}

static inline void
mtk_iic_busy_wait(struct mtk_iic_softc *sc)
{
	int i;
	uint32_t r;

	for (i = 0; i < i2c_busy_loop; ++i) {
		r = I2CREG_READ(sc, RA_I2C_STATUS);
		if ((r & I2C_STATUS_BUSY) == 0)
			break;
	}
}

static int 
mtk_iic_read(device_t dev, char *buf, int len, int *read, int last,
    int delay)
{
	struct mtk_iic_softc *sc;
	int i, j;
	uint32_t r;

	sc = device_get_softc(dev);

	mtk_iic_busy_wait(sc);

	I2CREG_WRITE(sc, RA_I2C_DEVADDR, sc->i2cdev_addr);
	I2CREG_WRITE(sc, RA_I2C_BYTECNT, len - 1);
	I2CREG_WRITE(sc, RA_I2C_STARTXFR, I2C_OP_READ);

	for (i = 0; i < len; i++) {
		for (j=0; j < max_ee_busy_loop; j++) {
			r = I2CREG_READ(sc, RA_I2C_STATUS);
			if((r & I2C_STATUS_DATARDY) != 0) {
				buf[i] = I2CREG_READ(sc, RA_I2C_DATAIN);
				break;
			}
		}
		if (j == max_ee_busy_loop) {
			return (IIC_ENOACK);
		}
	}

	*read =  i;

	mtk_iic_busy_wait(sc);
	return (IIC_NOERR);

}

static int 
mtk_iic_write(device_t dev, const char *buf, int len, int *sent,
    int timeout /* us */ )
{
	struct mtk_iic_softc *sc;
	int i, j;
	uint32_t r;
	int loopcount;

	sc = device_get_softc(dev);

	mtk_iic_busy_wait(sc);

	I2CREG_WRITE(sc, RA_I2C_DEVADDR, sc->i2cdev_addr);
	I2CREG_WRITE(sc, RA_I2C_BYTECNT, len - 1);
	I2CREG_WRITE(sc, RA_I2C_DATAOUT, buf[0]);
	I2CREG_WRITE(sc, RA_I2C_STARTXFR, I2C_OP_WRITE);

	for (i = 1; i < len; i++) {
		if (i == 1)   /* first loop send 2 byte */
			loopcount = max_ee_busy_loop * 2;
		else
			loopcount = max_ee_busy_loop;
		for (j=0; j < loopcount; j++) {
			r = I2CREG_READ(sc, RA_I2C_STATUS);
			if ((r & I2C_STATUS_SDOEMPTY) != 0) {
				I2CREG_WRITE(sc, RA_I2C_DATAOUT, buf[i]);
				break;
			}
		}
		if (j == max_ee_busy_loop) {
			return (IIC_ENOACK);
		}
	}

	*sent = i;

	mtk_iic_busy_wait(sc);
	return (IIC_NOERR);
}

#ifdef notyet
static int
mtk_iic_repeated_start(device_t dev, u_char slave, int timeout)
{
	return (IIC_NOERR);
}
#endif

int
mtk_iic_transfer(device_t bus, struct iic_msg *msgs, uint32_t nmsgs)
{       
        int i, error, lenread, lenwrote;
        u_char addr;
 
	addr = msgs[0].slave | LSB;
	error = mtk_iic_start(bus, addr, 0);
        for (i = 0, error = IIC_NOERR; i < nmsgs && error == IIC_NOERR; i++) {
                if (msgs[i].flags & IIC_M_RD) {
		        error = mtk_iic_read((bus), msgs[i].buf, msgs[i].len,
			    &lenread, IIC_LAST_READ, 0);
		}
                else {    
		        error = mtk_iic_write((bus), msgs[i].buf, msgs[i].len,
			    &lenwrote, 0);
		}
        }
	mtk_iic_stop(bus);
        return (error);
}

static int
mtk_iic_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	return (IIC_NOERR);
}

static phandle_t
mtk_iic_get_node(device_t bus, device_t dev)
{

	/* We only have one child, the I2C bus, which needs our own node. */
	return (ofw_bus_get_node(bus));
}

static device_method_t mtk_iic_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe, mtk_iic_probe),
	DEVMETHOD(device_attach, mtk_iic_attach),
	DEVMETHOD(device_detach, mtk_iic_detach),

	/* iicbus interface */
	DEVMETHOD(iicbus_start, mtk_iic_start),
	DEVMETHOD(iicbus_stop, mtk_iic_stop),
	DEVMETHOD(iicbus_write, mtk_iic_write),
	DEVMETHOD(iicbus_read, mtk_iic_read),
	DEVMETHOD(iicbus_callback, iicbus_null_callback),
	DEVMETHOD(iicbus_reset, mtk_iic_reset),
	DEVMETHOD(iicbus_transfer, mtk_iic_transfer),
#ifdef notyet
	DEVMETHOD(iicbus_repeated_start, mtk_iic_repeated_start),
#endif

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node, mtk_iic_get_node),

	DEVMETHOD_END
};

static driver_t mtk_iic_driver = {
	"iichb",
	mtk_iic_methods,
	sizeof(struct mtk_iic_softc),
};

static devclass_t mtk_iic_devclass;

DRIVER_MODULE(mtk_iic, simplebus, mtk_iic_driver, mtk_iic_devclass, 0, 0);
DRIVER_MODULE(iicbus, mtk_iic, iicbus_driver, iicbus_devclass, 0, 0);

MODULE_DEPEND(mtk_iic, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);

MODULE_VERSION(mtk_iic, 1);
