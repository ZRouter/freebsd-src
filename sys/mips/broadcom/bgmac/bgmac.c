/*-
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
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
 * Broadcom Gigabit MAC
 *
 * On Asus RT-N16 GMAC core attaches BCM53115 chip to SoC via GMII. So this driver is
 * MII-based. Information about registers are taken from:
 *      http://bcm-v4.sipsolutions.net/mac-gbit/Registers
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

#include "bgmac.h"

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_ids.h>

#include "bcm_dma.h"

MALLOC_DEFINE(M_BHND_BGMAC, "bgmac", "Structures allocated by bgmac driver");

struct resource_spec bgmac_rspec[BGMAC_MAX_RSPEC] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, -1, 0 }
};

struct bhnd_device bgmac_match[] = {
	BHND_DEVICE(BCM, GMAC, "BHND Gigabit MAC", NULL),
	BHND_DEVICE_END,
};

/****************************************************************************
 * Prototypes
 ****************************************************************************/
static int	bgmac_probe(device_t dev);
static int	bgmac_attach(device_t dev);
static int	bgmac_detach(device_t dev);

/* MII interface */
static int	bgmac_readreg(device_t dev, int phy, int reg);
static int	bgmac_writereg(device_t dev, int phy, int reg, int val);
static int	bgmac_phyreg_poll(device_t dev, uint32_t reg, uint32_t mask);
static int	bgmac_phyreg_op(device_t dev, phymode op, int phy, int reg,
		    int* val);

/* Interrupt flow */
static void	bgmac_intr(void *arg);

/* NVRAM variables */
static int	bgmac_get_config(struct bgmac_softc *sc);

/* chip manipulations */
static int	bgmac_chip_force_init(struct bgmac_softc *sc);
static void	bgmac_chip_start_txrx(struct bgmac_softc * sc);
static void	bgmac_chip_stop_txrx(struct bgmac_softc * sc);
static void	bgmac_chip_set_macaddr(struct bgmac_softc * sc);
static void	bgmac_chip_set_cmdcfg(struct bgmac_softc * sc, uint32_t val);
static void	bgmac_chip_set_intr_mask(struct bgmac_softc *sc,
		    enum bgmac_intr_status st);

/* ifnet(9) interface */
static void	bgmac_if_setup(device_t dev);
static void	bgmac_if_init(void* arg);
static void	bgmac_if_init_locked(struct bgmac_softc *sc);
static int	bgmac_if_ioctl(if_t ifp, u_long command, caddr_t data);
static int	bgmac_if_mediachange(struct ifnet *ifp);
static void	bgmac_if_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr);

/****************************************************************************
 * Implementation
 ****************************************************************************/
void
bgmac_print_debug(struct bgmac_softc* sc)
{

	BGMACDUMP(sc);
	BGMACDUMPMIB(sc);
	BGMACDUMPERRORS(sc);
}

static int
bgmac_probe(device_t dev)
{
	const struct bhnd_device *id;

	id = bhnd_device_lookup(dev, bgmac_match, sizeof(bgmac_match[0]));
	if (id == NULL)
		return (ENXIO);

	device_set_desc(dev, id->desc);
	return (BUS_PROBE_DEFAULT);
}

static int
bgmac_attach(device_t dev)
{
	struct bgmac_softc	*sc;
	struct resource		*res[BGMAC_MAX_RSPEC];
	int			 error;
	struct sysctl_ctx_list	*ctx;
	struct sysctl_oid	*tree;
	char 			 name[IFNAMSIZ];

	sc = device_get_softc(dev);
	sc->dev = dev;

	CTR1(KTR_BGMAC, "%s: ----- START ATTACH -----", device_get_nameunit(dev));

	/* Allocate bus resources */
	error = bus_alloc_resources(dev, bgmac_rspec, res);
	if (error){
		BHND_ERROR_DEV(dev, "can't allocate resources: %d", error);
		return (error);
	}

	sc->mem = res[0];
	sc->irq = res[1];

	error = bgmac_get_config(sc);
	if (error) {
		BHND_ERROR_DEV(dev, "can't get bgmac config from NVRAM: %d",
		    error);
		return (ENXIO);
	}

	bgmac_print_debug(sc);

	/* Reset breaks BCM5356 due to power down of internal ethernet switch */
#if defined(notyet)
	bhnd_reset_hw(dev, 0, 0);
#endif /* notyet */

	/* Hook interrupt */
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_NET | INTR_MPSAFE,
			NULL, bgmac_intr, sc, &sc->intrhand);
	if (error) {
		BHND_ERROR_DEV(dev, "can't setup interrupt");
		bgmac_detach(dev);
		return (error);
	}


#if defined(notyet)
	/* Get power */
	error = bhnd_alloc_pmu(dev);
	if(error) {
		BHND_ERROR_DEV(dev, "can't alloc pmu: %d", error);
		return (error);
	}

	error = bhnd_release_ext_rsrc(dev, 1);
	if(error) {
		BHND_ERROR_DEV(dev, "can't release ext: %d", error);
		return (error);
	}
#endif


	error = bgmac_chip_force_init(sc);
	if (error) {
		BHND_ERROR_DEV(dev, "can't init GMAC chip %x", error);
		bgmac_detach(dev);
		return (error);
	}

	bgmac_if_setup(dev);
	BGMAC_LOCK_INIT(sc);

	/* Attach MDIO bus to discover ethernet switch */
	sc->mdio = device_add_child(dev, "mdio", -1);
	bus_generic_attach(dev);

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	snprintf(name, IFNAMSIZ, "cpu_rx");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    name, CTLFLAG_RD | CTLTYPE_U16, sc,
	    0,
	    bgmac_sysctl_mib, "I", "number of packets received by port");

	snprintf(name, IFNAMSIZ, "cpu_tx");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    name, CTLFLAG_RD | CTLTYPE_U16, sc,
	    1,
	    bgmac_sysctl_mib, "I", "number of packets sent by port");

	snprintf(name, IFNAMSIZ, "dump");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    name, CTLFLAG_RD | CTLTYPE_STRING, sc,
	    0,
	    bgmac_sysctl_dump, "A", "dump of registers");


	CTR1(KTR_BGMAC, "%s: ----- AFTER ATTACH -----", device_get_nameunit(dev));
	bgmac_print_debug(sc);

	return 	0;
}

static int
bgmac_detach(device_t dev)
{
	struct bgmac_softc	*sc;
	int 			 ret;

	sc = device_get_softc(dev);

	/* detach etherswitch connected via MDIO */
	bus_generic_detach(dev);

	if (sc->mdio != NULL)
		device_delete_child(dev, sc->mdio);

	if (sc->mem != NULL)
		bus_release_resource(dev, bgmac_rspec[0].type, bgmac_rspec[0].rid, sc->mem);

	if (sc->irq != NULL)
		bus_release_resource(dev, bgmac_rspec[1].type, bgmac_rspec[1].rid, sc->irq);

	ret = bhnd_release_pmu(dev);
	if (ret != 0)
		CTR2(KTR_BGMAC, "%s: error on release PMU 0x%x",
		    device_get_nameunit(dev), ret);

	/* stop chip */
#if defined(notyet)
	bgmac_chip_stop();
	/* unmap dma and reset chip configuration */
	bgmac_chip_deinit();
	/* free resources */
#endif

	if (sc->dma != NULL) {
		bcm_dma_detach(sc->dma);
		free(sc->dma, M_BHND_BGMAC);
	}
	return 0;
}	

/***** Chip operations *****/

/*
 * This function resets chip and makes preparations like clocks, DMA, access to
 * external switch if present
 */
static int
bgmac_chip_force_init(struct bgmac_softc *sc)
{
	int			 error;
	uint32_t		 tmp;
	device_t		 dev;

	dev = sc->dev;

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp | BGMAC_REG_CMD_CFG_RESET);
	/* hope it's enough for reset */
	DELAY(5);

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);

	/* disable tx, rx, loopback and enable full duplex, support of pauses */
	tmp &= ~(	BGMAC_REG_CMD_CFG_TX | BGMAC_REG_CMD_CFG_RX |
			BGMAC_REG_CMD_CFG_PAUSEIGNORE | BGMAC_REG_CMD_CFG_TAI |
			BGMAC_REG_CMD_CFG_HALFDUPLEX | BGMAC_REG_CMD_CFG_LOOPBACK |
			BGMAC_REG_CMD_CFG_RL | BGMAC_REG_CMD_CFG_PAD_ENA |
			BGMAC_REG_CMD_CFG_RED | BGMAC_REG_CMD_CFG_PE |
			BGMAC_REG_CMD_CFG_PF);
	/* enable promiscuous support */
	tmp |= (	BGMAC_REG_CMD_CFG_PROM | BGMAC_REG_CMD_CFG_NLC |
			BGMAC_REG_CMD_CFG_CFE | BGMAC_REG_CMD_CFG_TPI |
			BGMAC_REG_CMD_CFG_AT);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp);

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp & ~BGMAC_REG_CMD_CFG_RESET);
	DELAY(5);

	/* enable external access to switch */
	tmp = bus_read_4(sc->mem, BGMAC_REG_PHY_CONTROL);
	bus_write_4(sc->mem, BGMAC_REG_PHY_CONTROL, tmp | BGMAC_REG_PHY_CONTROL_MTE);

	/* set MAC address */
	bgmac_chip_set_macaddr(sc);

	if (sc->dma != NULL) {
		bcm_dma_detach(sc->dma);
		sc->dma = NULL;
	}

	sc->dma = malloc(sizeof(struct bcm_dma), M_BHND_BGMAC, M_WAITOK);
	if (sc->dma == NULL) {
		BHND_ERROR_DEV(sc->dev, "can't allocate memory for bcm_dma");
		return (ENOMEM);
	}

	/* clear interrupt status */
	uint32_t intr_status = bus_read_4(sc->mem, BGMAC_REG_INTR_STATUS);
	bus_write_4(sc->mem, BGMAC_REG_INTR_STATUS, intr_status);

	CTR2(KTR_BGMAC, "%s: clear bgmac_intr_status: 0x%x",
	    device_get_nameunit(dev), intr_status);

	/* set number of interrupts per frame */
	bus_write_4(sc->mem, BGMAC_REG_INTR_RECV_LAZY,
	    1 << BGMAC_REG_INTR_RECV_LAZY_FC_SHIFT);

	/* enable high throughput clocking */
	error = bhnd_alloc_pmu(dev);
	CTR2(KTR_BGMAC, "%s: allocate power unit: err = 0x%x",
		device_get_nameunit(dev), error);
	if (error != 0)
		return (error);

	error = bhnd_request_clock(dev, BHND_CLOCK_HT);
	CTR2(KTR_BGMAC, "%s: request clock gating HT: err = 0x%x",
		    device_get_nameunit(dev), error);
	if (error != 0)
		return (error);

	error = bhnd_request_ext_rsrc(dev, 1);
	CTR2(KTR_BGMAC, "%s: request external PMU resource: err = 0x%x",
		    device_get_nameunit(dev), error);
	if(error != 0)
		return (error);

	/* Flow control - on/off and pause */
	/*
	bus_write_4(sc->mem, BGMAC_FLOW_CTL_THRESH, 0x03cb04cb);
	bus_write_4(sc->mem, BGMAC_PAUSE_CTL, 0x27fff);
	*/

	error = bcm_dma_attach(sc->dev, sc->mem, sc->dma);
	if (error) {
		BHND_ERROR_DEV(sc->dev, "error occurred during bcm_dma_attach: %d",
		    error);
		return (error);
	}

	return (0);
}

static void
bgmac_chip_set_intr_mask(struct bgmac_softc *sc, enum bgmac_intr_status st)
{
	uint32_t	mask;
	uint32_t	feed;

	if (st & I_OR) {
		mask = bus_read_4(sc->mem, BGMAC_REG_INTERRUPT_MASK);
	} else {
		mask = 0;
	}

	if (st & I_ERR)
		mask |= BGMAC_REG_INTR_STATUS_ERR;
	if (st & I_RX)
		mask |= BGMAC_REG_INTR_STATUS_RX;
	if (st & I_TX)
		mask |= BGMAC_REG_INTR_STATUS_TX;

	mask |= BGMAC_REG_INTR_STATUS_MDIO;

	bus_write_4(sc->mem, BGMAC_REG_INTERRUPT_MASK, mask);
	feed = bus_read_4(sc->mem, BGMAC_REG_INTERRUPT_MASK);
	CTR2(KTR_BGMAC, "%s: bgmac_intr = 0x%x", device_get_nameunit(sc->dev), feed);
}

static void
bgmac_chip_set_cmdcfg(struct bgmac_softc *sc, uint32_t val)
{
	uint32_t	tmp;

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp | BGMAC_REG_CMD_CFG_RESET);
	DELAY(2);
	bus_barrier(sc->mem, BGMAC_REG_CMD_CFG, 0, BUS_SPACE_BARRIER_WRITE | BUS_SPACE_BARRIER_READ);

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp | val);
	DELAY(100);
	bus_barrier(sc->mem, BGMAC_REG_CMD_CFG, 0, BUS_SPACE_BARRIER_WRITE | BUS_SPACE_BARRIER_READ);

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp & ~BGMAC_REG_CMD_CFG_RESET);
	DELAY(2);
	bus_barrier(sc->mem, BGMAC_REG_CMD_CFG, 0, BUS_SPACE_BARRIER_WRITE | BUS_SPACE_BARRIER_READ);
}

static void
bgmac_chip_unset_cmdcfg(struct bgmac_softc *sc, uint32_t val)
{
	uint32_t	tmp;

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp | BGMAC_REG_CMD_CFG_RESET);
	DELAY(2);
	bus_barrier(sc->mem, BGMAC_REG_CMD_CFG, 0, BUS_SPACE_BARRIER_WRITE | BUS_SPACE_BARRIER_READ);

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp & ~val);
	DELAY(100);
	bus_barrier(sc->mem, BGMAC_REG_CMD_CFG, 0, BUS_SPACE_BARRIER_WRITE | BUS_SPACE_BARRIER_READ);

	tmp = bus_read_4(sc->mem, BGMAC_REG_CMD_CFG);
	bus_write_4(sc->mem, BGMAC_REG_CMD_CFG, tmp & ~BGMAC_REG_CMD_CFG_RESET);
	DELAY(2);
	bus_barrier(sc->mem, BGMAC_REG_CMD_CFG, 0, BUS_SPACE_BARRIER_WRITE | BUS_SPACE_BARRIER_READ);
}


static void
bgmac_chip_set_macaddr(struct bgmac_softc * sc)
{

	/*
	 * Write MAC address
	 */
	bus_write_4(sc->mem, 0x80c, *((uint32_t*)sc->addr));
	bus_write_4(sc->mem, 0x810, *(((uint16_t*)sc->addr)+2));
}

static void
bgmac_chip_start_txrx(struct bgmac_softc * sc)
{

	BGMAC_ASSERT_LOCKED(sc);
	CTR1(KTR_BGMAC, "%s: start TX / RX", device_get_nameunit(sc->dev));
	bgmac_chip_set_cmdcfg(sc, BGMAC_REG_CMD_CFG_RX | BGMAC_REG_CMD_CFG_TX);
	bgmac_chip_set_intr_mask(sc, I_ERR | I_RX | I_TX);
	return;
}

static void
bgmac_chip_stop_txrx(struct bgmac_softc * sc)
{

	BGMAC_ASSERT_LOCKED(sc);
	CTR1(KTR_BGMAC, "%s: stop TX / RX", device_get_nameunit(sc->dev));
	bgmac_chip_unset_cmdcfg(sc, BGMAC_REG_CMD_CFG_RX | BGMAC_REG_CMD_CFG_TX);
	bgmac_chip_set_intr_mask(sc, I_ERR);
	return;
}

/*
 * Setup interface
 */
static void
bgmac_if_setup(device_t dev)
{
	struct bgmac_softc	*sc;
	struct ifnet		*ifp;

	sc = device_get_softc(dev);
	ifp = sc->ifp = if_alloc(IFT_ETHER);
	ifp->if_softc = sc;

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(ifp, bgmac_if_init);
	if_setioctlfn(ifp, bgmac_if_ioctl);
	if_setstartfn(ifp, bgmac_if_start);

	ifmedia_init(&sc->ifmedia, 0, bgmac_if_mediachange, bgmac_if_mediastatus);
	ifmedia_add(&sc->ifmedia, IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
	ifmedia_set(&sc->ifmedia, IFM_ETHER | IFM_1000_T | IFM_FDX);

	ether_ifattach(ifp, sc->addr);
	ifp->if_capabilities = ifp->if_capenable = IFCAP_RXCSUM | IFCAP_TXCSUM;

	//IFCAP_VLAN_MTU | IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_HWCSUM;
	//IFCAP_VLAN_HWFILTER | ;

	return;
}

static int
bgmac_if_mediachange(struct ifnet *ifp)
{
	struct bgmac_softc	*sc;
	struct ifmedia		*ifm;
	struct ifmedia_entry	*ife;

	sc = ifp->if_softc;
	ifm = &sc->ifmedia;
	ife = ifm->ifm_cur;

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return (EINVAL);

	if (IFM_SUBTYPE(ife->ifm_media) == IFM_AUTO) {
		device_printf(sc->dev, "AUTO is not supported for multiphy MAC");
		return (EINVAL);
	}

	/*
	 * Ignore everything
	 */
	return (0);
}

static void
bgmac_if_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)
{

	ifmr->ifm_status = IFM_AVALID | IFM_ACTIVE;
	/* TODO: retrieve mode & duplex info from softc */
	ifmr->ifm_active = IFM_ETHER | IFM_1000_T | IFM_FDX;
}

static void
bgmac_if_init(void* arg)
{
	struct bgmac_softc	*sc;

	sc = (struct bgmac_softc*) arg;

	BGMAC_LOCK(sc);
	bgmac_if_init_locked(sc);
	BGMAC_UNLOCK(sc);
}

static void
bgmac_if_init_locked(struct bgmac_softc *sc)
{

	/* TODO: promiscios mode => ioctl */
	BHND_DEBUG_DEV(sc->dev, "init driver");
	/* set number of interrupts per frame */
	bus_write_4(sc->mem, BGMAC_REG_INTR_RECV_LAZY,
	    1 << BGMAC_REG_INTR_RECV_LAZY_FC_SHIFT);

	bgmac_chip_set_intr_mask(sc, I_ERR | I_OR);

	sc->ifp->if_drv_flags |= IFF_DRV_RUNNING;
	sc->ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
}

static int
bgmac_if_ioctl(if_t ifp, u_long command, caddr_t data)
{
	struct bgmac_softc	*sc;
	struct ifreq		*ifr;
	int			 error;

	sc = if_getsoftc(ifp);
	ifr = (struct ifreq *) data;
	error = 0;

	switch (command) {
	case SIOCSIFFLAGS:
		BGMAC_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				bgmac_chip_start_txrx(sc);
			else
			{
				/* TODO: add detach */
				bgmac_if_init_locked(sc);
				bgmac_chip_start_txrx(sc);
			}
		} else if (ifp->if_drv_flags & IFF_DRV_RUNNING)
			bgmac_chip_stop_txrx(sc);
		BGMAC_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		/* XXX: implement SIOCDELMULTI */
		error = 0;
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->ifmedia, command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}

	return (error);
}

/**************************** MII BUS Functions *****************************/

int bgmac_phyreg_poll(device_t dev,uint32_t reg, uint32_t mask){
	struct bgmac_softc	*sc;
	int			 i;
	uint32_t		 val;

	sc = device_get_softc(dev);
	i = BGMAC_TIMEOUT;

	/* Poll for the register to complete. */
	for (; i > 0; i--) {
		DELAY(10);
		val = bus_read_4(sc->mem, reg);
		if ((val & mask) != 0)
			continue;
		/* Success */
		DELAY(5);
		return (0);
	}
	return (-1);
}

static int
bgmac_phyreg_op(device_t dev, phymode op, int phy, int reg, int* val)
{
	struct bgmac_softc	*sc;
	uint32_t		 tmp;

	sc = device_get_softc(dev);

	/* Set address on PHY control register */
	tmp = bus_read_4(sc->mem, BGMAC_REG_PHY_CONTROL);
	tmp = (tmp & (~BGMAC_REG_PHY_CONTROL_ADDR)) | phy;

	bus_write_4(sc->mem, BGMAC_REG_PHY_CONTROL, tmp);

	/* Send header (first 16 bytes) over MII */
	tmp = BGMAC_REG_PHY_ACCESS_START;
	if (op == PHY_WRITE) {
		tmp |= BGMAC_REG_PHY_ACCESS_WRITE;
		tmp |= ((*val) & BGMAC_REG_PHY_ACCESS_DATA);
	}

	tmp |= (phy << BGMAC_REG_PHY_ACCESS_ADDR_SHIFT);
	tmp |= (reg << BGMAC_REG_PHY_ACCESS_REG_SHIFT);

	bus_write_4(sc->mem, BGMAC_REG_PHY_ACCESS, tmp);

	/* Wait while operation is finished */
	if (bgmac_phyreg_poll(dev, BGMAC_REG_PHY_ACCESS,
	    BGMAC_REG_PHY_ACCESS_START)) {
		return (-1);
	}

	if (op == PHY_READ) {
		/* Read rest of 16 bytes back */
		tmp = bus_read_4(sc->mem, BGMAC_REG_PHY_ACCESS);
		tmp &= BGMAC_REG_PHY_ACCESS_DATA;
		*val = tmp;
	}

	return (0);
}

static int
bgmac_readreg(device_t dev, int phy, int reg)
{
	int	tmp;

	if (bgmac_phyreg_op(dev, PHY_READ, phy, reg, &tmp)) {
		BHND_ERROR_DEV(dev, "phy_readreg: phy=%x reg=%x", phy, reg);
		return (-1);
	}

	return (tmp);
}

static int
bgmac_writereg(device_t dev, int phy, int reg, int val)
{
	int	tmp;

	tmp = val;
	if (bgmac_phyreg_op(dev, PHY_WRITE, phy, reg, &tmp)) {
		BHND_ERROR_DEV(dev, "phy_writereg: phy=%x reg=%x", phy, reg);
		return (-1);
	}

	return (0);
}

/* Interrupt handler */
static void
bgmac_intr(void *arg)
{
	struct bgmac_softc	*sc;
	uint32_t		 intr_status;

	sc = (struct bgmac_softc*)arg;

	BGMAC_LOCK(sc);

	intr_status = bus_read_4(sc->mem, BGMAC_REG_INTR_STATUS);
	bus_write_4(sc->mem, BGMAC_REG_INTR_STATUS, intr_status);

	CTR2(KTR_BGMAC, "%s: bgmac_intr_status: 0x%x",
	    device_get_nameunit(sc->dev), intr_status);

	/* disable interrupt for a while */
	bgmac_chip_set_intr_mask(sc, 0);

	if (intr_status & BGMAC_REG_INTR_STATUS_TX) {
		CTR1(KTR_BGMAC, "TX!", device_get_nameunit(sc->dev));
		bcm_dma_tx(sc->dma->wme[0]);
		intr_status &= ~BGMAC_REG_INTR_STATUS_TX;
	}

	if (intr_status & BGMAC_REG_INTR_STATUS_RX) {
		CTR1(KTR_BGMAC, "RX!", device_get_nameunit(sc->dev));
		bcm_dma_rx(sc->dma->rx);
		intr_status &= ~BGMAC_REG_INTR_STATUS_RX;
	}

	if (intr_status & BGMAC_REG_INTR_STATUS_MDIO) {
		CTR1(KTR_BGMAC, "MDIO!", device_get_nameunit(sc->dev));
		intr_status &= ~BGMAC_REG_INTR_STATUS_MDIO;
	}

	if (intr_status & BGMAC_REG_INTR_STATUS_ERR_OVER) {
		BHND_ERROR_DEV(sc->dev, "Overflow!");
		bgmac_print_debug(sc);
		bcm_dma_rx(sc->dma->rx);
		bgmac_print_debug(sc);
		intr_status &= ~BGMAC_REG_INTR_STATUS_ERR_OVER;
	}

	if (intr_status & BGMAC_REG_INTR_STATUS_ERR_DESC){
		BHND_ERROR_DEV(sc->dev, "ERROR!");
		bgmac_print_debug(sc);
		kdb_enter("bgmac_error", "unknown error: descriptor?");
		intr_status &= ~BGMAC_REG_INTR_STATUS_ERR_OVER;
	}

	/* enable interrupt */
	bgmac_chip_set_intr_mask(sc, I_ERR | I_RX | I_TX);
	BGMAC_UNLOCK(sc);
	return;
}

/* Get configuration from NVRAM */
static int
bgmac_get_config(struct bgmac_softc *sc)
{
	char	*tmp;
	char	 macaddr[18];

	tmp = kern_getenv("sb/1/macaddr");
	/* FIXME: RT-N12 - why missing? */
	if (tmp == NULL)
		tmp = "AC:9E:17:BA:FC:D8";
		//return (-1);

	if (strlen(tmp) != strlen("00:00:00:00:00:00"))
		return (-2);

	strcpy(macaddr, tmp);

	for (int i = 2; i < strlen(macaddr); i+=3) {
		if (macaddr[i] != ':')
			return (-3);
		macaddr[i] = '\0';
	}

	tmp = macaddr;
	for (int i = 0; i < 6; i++) {
		sc->addr[i] = (u_char)strtoul(tmp, NULL, 16);
		tmp += 3;
	}

	return (0);
}

void
bgmac_rxeof(struct device *dev, struct mbuf *m, struct bcm_rx_header *rxhdr)
{
	struct bgmac_softc	*sc;
	struct ifnet		*ifp;

	/* TODO: early draft - need more attention */
	sc = device_get_softc(dev);
	ifp = sc->ifp;

	BGMAC_ASSERT_LOCKED(sc);

	m->m_pkthdr.rcvif = ifp;
	BGMAC_UNLOCK(sc);
	(*ifp->if_input)(ifp, m);
	BGMAC_LOCK(sc);
}

/**
 * Driver metadata
 */
static device_method_t bgmac_methods[] = {
		DEVMETHOD(device_probe,	 	bgmac_probe),
		DEVMETHOD(device_attach, 	bgmac_attach),
		DEVMETHOD(device_detach,	bgmac_detach),

		/** miibus interface **/
		DEVMETHOD(miibus_readreg, 	bgmac_readreg),
		DEVMETHOD(miibus_writereg, 	bgmac_writereg),

		/** MDIO interface **/
		DEVMETHOD(mdio_readreg,		bgmac_readreg),
		DEVMETHOD(mdio_writereg,	bgmac_writereg),

		DEVMETHOD_END
};

devclass_t bgmac_devclass;

DEFINE_CLASS_0(bgmac, bgmac_driver, bgmac_methods, sizeof(struct bgmac_softc));

DRIVER_MODULE(bgmac, bhnd, bgmac_driver, bgmac_devclass, 0, 0);
DRIVER_MODULE(mdio, bgmac, mdio_driver, mdio_devclass, 0, 0);

MODULE_VERSION(bgmac, 1);
