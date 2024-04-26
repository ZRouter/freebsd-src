/*-
 * Copyright (c) 2024, Hiroki Mori
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>

#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/pmap.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>
#include "spibus_if.h"

#include <mips/broadcom/bcm338x/bcm338x_spireg.h>

#undef BCM338X_SPI_DEBUG
#ifdef BCM338X_SPI_DEBUG
#define dprintf printf
#else
#define dprintf(x, arg...)
#endif

/*
 * register space access macros
 */
#define SPI_WRITE32(sc, reg, val) \
	bus_write_4(sc->sc_mem_res, (reg), (val))
#define SPI_WRITE16(sc, reg, val) \
	bus_write_2(sc->sc_mem_res, (reg), (val))
#define SPI_WRITE8(sc, reg, val) \
	bus_write_1(sc->sc_mem_res, (reg), (val))

#define SPI_READ32(sc, reg)	 bus_read_4(sc->sc_mem_res, (reg))
#define SPI_READ16(sc, reg)	 bus_read_2(sc->sc_mem_res, (reg))
#define SPI_READ8(sc, reg)	 bus_read_1(sc->sc_mem_res, (reg))

struct bcm338x_spi_softc {
	device_t		sc_dev;
	struct resource		*sc_mem_res;
	uint32_t		sc_reg_ctrl;
	uint32_t		sc_debug;
};

static void
bcm338x_spi_attach_sysctl(device_t dev)
{
	struct bcm338x_spi_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;

	sc = device_get_softc(dev);
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		"debug", CTLFLAG_RW, &sc->sc_debug, 0,
		"bcm338x_spi debugging flags");
}

static int
bcm338x_spi_probe(device_t dev)
{
	device_set_desc(dev, "BCM338X SPI");
	return (0);
}

static int
bcm338x_spi_attach(device_t dev)
{
	struct bcm338x_spi_softc *sc = device_get_softc(dev);
	int rid;

	sc->sc_dev = dev;
        rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, 
	    RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "Could not map memory\n");
		return (ENXIO);
	}

	device_add_child(dev, "spibus", -1);
	bcm338x_spi_attach_sysctl(dev);

	return (bus_generic_attach(dev));
}

static void
bcm338x_spi_chip_activate(struct bcm338x_spi_softc *sc, int cs)
{
}

static void
bcm338x_spi_chip_deactivate(struct bcm338x_spi_softc *sc, int cs)
{
}

static void
bcm338x_spi_read(struct bcm338x_spi_softc *sc, unsigned char *pRxBuf,
    int prependcnt, int nbytes, int devId)
{
	int i;
	uint16_t msgCtrl;

	SPI_WRITE32(sc, PROFILEMODECTRL, prependcnt << HS_SPI_PREPENDBYTE_CNT |
	    0 << HS_SPI_MODE_ONE_WIRE | 0<<HS_SPI_MULTIDATA_WR_SIZE |
	    0 << HS_SPI_MULTIDATA_RD_SIZE | 2 << HS_SPI_MULTIDATA_WR_STRT |
	    2 << HS_SPI_MULTIDATA_RD_STRT | 0xff << HS_SPI_FILLBYTE);

	msgCtrl = (HS_SPI_OP_READ << HS_SPI_OP_CODE) | nbytes;
	SPI_WRITE16(sc, PINGPONGFIFOREGS, msgCtrl);

	for (i = 0; i < prependcnt; ++i) {
		SPI_WRITE8(sc, PINGPONGFIFOREGS + 2 + i, pRxBuf[i]);
	}

	SPI_WRITE32(sc, PINGPONGCOMMAND,  devId << HS_SPI_SS_NUM |
	    0 << HS_SPI_PROFILE_NUM | 0 << HS_SPI_TRIGGER_NUM |
	    HS_SPI_COMMAND_START_NOW << HS_SPI_COMMAND_VALUE);
}

static void
bcm338x_spi_write(struct bcm338x_spi_softc *sc, unsigned char *pTxBuf,
    int nbytes, int devId, int opcode)
{
	int i;
	uint16_t msgCtrl;
	
	SPI_WRITE32(sc, PROFILEMODECTRL, 0 << HS_SPI_PREPENDBYTE_CNT |
	    0 << HS_SPI_MODE_ONE_WIRE | 0 << HS_SPI_MULTIDATA_WR_SIZE |
	    0 << HS_SPI_MULTIDATA_RD_SIZE | 2 << HS_SPI_MULTIDATA_WR_STRT |
	    2 << HS_SPI_MULTIDATA_RD_STRT | 0xff << HS_SPI_FILLBYTE);

	if (opcode == BCM_SPI_FULL)
		msgCtrl  = (HS_SPI_OP_READ_WRITE << HS_SPI_OP_CODE) | nbytes;
	else
		msgCtrl  = (HS_SPI_OP_WRITE << HS_SPI_OP_CODE) | nbytes;

	SPI_WRITE16(sc, PINGPONGFIFOREGS, msgCtrl);

	for (i = 0; i < nbytes; ++i) {
		SPI_WRITE8(sc, PINGPONGFIFOREGS + 2 + i, pTxBuf[i]);
	}

	SPI_WRITE32(sc, PINGPONGCOMMAND, devId << HS_SPI_SS_NUM |
	    0 << HS_SPI_PROFILE_NUM | 0 << HS_SPI_TRIGGER_NUM |
	    HS_SPI_COMMAND_START_NOW << HS_SPI_COMMAND_VALUE);
}

static void
bcm338x_spi_trans_end(struct bcm338x_spi_softc *sc, unsigned char *rxBuf,
    int nbytes)
{
	int i;

	for (i = 0; i < nbytes; ++i)
		rxBuf[i] = SPI_READ8(sc, PINGPONGFIFOREGS + i);
}

static int
bcm338x_spi_trans_poll(struct bcm338x_spi_softc *sc)
{
	int wait;

	for (wait = 0; wait < 100 * 1000; ++wait) {
		if ((SPI_READ32(sc, PINGPONGSTATUS) & (1 << HS_SPI_SOURCE_BUSY))
		     == 0)
			return 1;
		DELAY(1);
	}

	return 1;
}
static void
bcm338x_spi_set_clock(struct bcm338x_spi_softc *sc, int clockHz)
{
	int clock;

	clock = HS_SPI_PLL_FREQ / clockHz;
	if (HS_SPI_PLL_FREQ % HS_SPI_CLOCK_DEF)
		clock++;

	clock = 2048 / clock;
	if (2048 % clock)
		clock++;

	SPI_WRITE32(sc, PROFILECLKCTRL, 1 << HS_SPI_ACCUM_RST_ON_LOOP |
	    0 << HS_SPI_SPI_CLK_2X_SEL | clock << HS_SPI_FREQ_CTRL_WORD);
}

static int
bcm338x_spi_txrx(struct ar71xx_spi_softc *sc, int cs, int size,
    uint8_t *txdata, uint8_t *rxdata)
{
	bcm338x_spi_set_clock(sc, 1000 * 1000);
	bcm338x_spi_write(sc, txdata, size, cs, BCM_SPI_FULL);
	if(bcm338x_spi_trans_poll(sc)) {
		bcm338x_spi_trans_end(sc, rxdata, size);
	}
	return (0);
}

static int
bcm338x_spi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	struct bcm338x_spi_softc *sc;
	uint8_t *buf_in, *buf_out;
	uint32_t cs;

	sc = device_get_softc(dev);

	spibus_get_cs(child, &cs);

	/*
	 * Transfer command
	 */
	buf_out = (uint8_t *)cmd->tx_cmd;
	buf_in = (uint8_t *)cmd->rx_cmd;
	if (cmd->tx_cmd_sz)
		bcm338x_spi_txrx(sc, cs, cmd->tx_cmd_sz,
		    buf_out, buf_in);

	/*
	 * Receive/transmit data (depends on  command)
	 */
	buf_out = (uint8_t *)cmd->tx_data;
	buf_in = (uint8_t *)cmd->rx_data;
	if (cmd->tx_data_sz)
		bcm338x_spi_txrx(sc, cs, cmd->tx_data_sz,
		    buf_out, buf_in);
	/*
	 * Close SPI controller interface, restore flash memory mapped access.
	 */

	return (0);
}

static int
bcm338x_spi_detach(device_t dev)
{
	struct bcm338x_spi_softc *sc = device_get_softc(dev);

	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);

	return (0);
}

static device_method_t bcm338x_spi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcm338x_spi_probe),
	DEVMETHOD(device_attach,	bcm338x_spi_attach),
	DEVMETHOD(device_detach,	bcm338x_spi_detach),

	DEVMETHOD(spibus_transfer,	bcm338x_spi_transfer),

	DEVMETHOD_END
};

static driver_t bcm338x_spi_driver = {
	"spi",
	bcm338x_spi_methods,
	sizeof(struct bcm338x_spi_softc),
};

static devclass_t bcm338x_spi_devclass;

DRIVER_MODULE(bcm338x_spi, obio, bcm338x_spi_driver, bcm338x_spi_devclass, 0, 0);
