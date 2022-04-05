/*-
 * Copyright (c) 2011 Aleksandr Rybalko.
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
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
 * $FreeBSD$
 */

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/errno.h>

#include "robosw_var.h"
#include "robosw_reg.h"
#include "robosw_hal.h"

static int	bcm53115_get_vlan_group(struct robosw_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id);
static int	bcm53115_set_vlan_group(struct robosw_softc *sc, int vlan_group,
		    int vlan_id, int members, int untagged, int forward_id);
static int 	bcm53115_reset(struct robosw_softc *sc);
static void	bcm53115_init_context(struct robosw_softc *sc);
static int 	bcm53115_vlan_enable_1q(struct robosw_softc *sc, int on);

static uint32_t	bcm53115_mib_get(struct robosw_softc *sc, int port, int metric);

struct robosw_functions bcm53115_f = {
	.api.reset = bcm53115_reset,
	.api.init_context = bcm53115_init_context,
	.api.mib_get = bcm53115_mib_get,
	.api.vlan_enable_1q = bcm53115_vlan_enable_1q,
	.api.vlan_get_vlan_group = bcm53115_get_vlan_group,
	.api.vlan_set_vlan_group = bcm53115_set_vlan_group
};

struct robosw_hal bcm53115_hal = {
	.chipname = "BCM53115",
	.parent = &bcm5325_hal,
	.self = &bcm53115_f
};

static void
bcm53115_init_context(struct robosw_softc *sc)
{

	sc->info.es_nports = 7;
}

static uint32_t
bcm53115_mib_get(struct robosw_softc *sc, int port, int metric)
{
	switch (metric) {
	case ROBOSW_MIB_GOODRXPKTS:
		return
		   (robosw_read4(sc, MIB_53115_RX_UnicastPkts(port)) +
		    robosw_read4(sc, MIB_53115_RX_BroadcastPkts(port)) +
		    robosw_read4(sc, MIB_53115_RX_MulticastPkts(port)) +
		    robosw_read4(sc, MIB_53115_RX_DropPkts(port)));
	case ROBOSW_MIB_GOODTXPKTS:
		return
		   (robosw_read4(sc, MIB_53115_TX_UnicastPkts(port)) +
		    robosw_read4(sc, MIB_53115_TX_BroadcastPkts(port)) +
		    robosw_read4(sc, MIB_53115_TX_MulticastPkts(port)) +
		    robosw_read4(sc, MIB_53115_TX_DropPkts(port)));
	default:
		return (0);
	}
}

static int
bcm53115_reset(struct robosw_softc *sc)
{
	uint32_t	reg;
	int		err;

	err = ROBOSW_PARENT_API(bcm53115_hal)->reset(sc);
	if (err != 0)
		return (err);

	reg = GLOBAL_MGMT_CTL_MGMT_PORT_MII | GLOBAL_MGMT_CTL_RST_MIB | 0x40;
	ROBOSW_WR(GLOBAL_MGMT_CTL, reg, sc);


	reg = GLOBAL_MGMT_CTL_MGMT_PORT_MII | 0x40;
	ROBOSW_WR(GLOBAL_MGMT_CTL, reg, sc);

	return (0);
}

static int
bcm53115_vlan_enable_1q(struct robosw_softc *sc, int on)
{
	uint32_t	ctl0, ctl1, drop, ctl4, ctl5, sw;

	drop = 0;

	ROBOSW_RD(VLAN_GLOBAL_CTL0, ctl0, sc);
	ROBOSW_RD(VLAN_GLOBAL_CTL1, ctl1, sc);
	/* b53115-specific */
	ROBOSW_RD(VLAN_GLOBAL_CTL4_531xx, ctl4, sc);
	ctl4 &= ~VLAN_GLOBAL_CTL4_VID_MASK;
	ROBOSW_RD(VLAN_GLOBAL_CTL5_531xx, ctl5, sc);

	device_printf(sc->sc_dev, "ctl <=: %x/%x/%x/%x\n", ctl0, ctl1, ctl4,
	    ctl5);

	if (on) {
		ctl0 |= VLAN_GLOBAL_CTL0_1Q_ENABLE |
			    VLAN_GLOBAL_CTL0_MATCH_VIDMAC |
			    VLAN_GLOBAL_CTL0_HASH_VIDADDR;
		ctl1 |= VLAN_GLOBAL_CTL1_MCAST_UNTAGMAP_CHECK |
			    VLAN_GLOBAL_CTL1_MCAST_FWDMAP_CHECK;
		//ctl4 |= VLAN_GLOBAL_CTL4_DROP_VID_VIOLATION;
		//ctl5 |= VLAN_GLOBAL_CTL5_DROP_VTAB_MISS;

		/* b53115-specific: TODO: add support of 4095 vlans */
		//ctl5 &= ~VLAN_GLOBAL_CTL5_VID_4095_ENABLE;
	} else {
		ctl0 &= ~(VLAN_GLOBAL_CTL0_1Q_ENABLE |
			    VLAN_GLOBAL_CTL0_MATCH_VIDMAC |
			    VLAN_GLOBAL_CTL0_HASH_VIDADDR);
		ctl1 &= ~(VLAN_GLOBAL_CTL1_MCAST_UNTAGMAP_CHECK |
			    VLAN_GLOBAL_CTL1_MCAST_FWDMAP_CHECK);
		ctl5 &= ~VLAN_GLOBAL_CTL5_DROP_VTAB_MISS;
		ctl4 &= ~VLAN_GLOBAL_CTL4_DROP_VID_VIOLATION;
		/* b53115-specific */
		ctl4 |= VLAN_GLOBAL_CTL4_FWD_TO_MII;
		ctl5 &= ~VLAN_GLOBAL_CTL5_VID_4095_ENABLE;
	}

	device_printf(sc->sc_dev, "ctl =>: %x/%x/%x/%x\n", ctl0, ctl1, ctl4,
	    ctl5);

	ROBOSW_WR(VLAN_GLOBAL_CTL0, ctl0, sc);
	ROBOSW_WR(VLAN_GLOBAL_CTL1, ctl1, sc);
	//ROBOSW_WR(VLAN_DROP_UNTAGGED, drop, sc);
	ROBOSW_WR(VLAN_GLOBAL_CTL4_531xx, ctl4, sc);
	ROBOSW_WR(VLAN_GLOBAL_CTL5_531xx, ctl5, sc);

	ROBOSW_RD(SWITCH_MODE, sw, sc);
	sw &= ~SWITCH_MODE_MANAGED;
	ROBOSW_WR(SWITCH_MODE, sw, sc);
	return (0);
}

static int
bcm53115_get_vlan_group(struct robosw_softc *sc, int vlan_group,
    int *vlan_id, int *members, int *untagged, int *forward_id)
{
	uint32_t		 reg;

	/* Set VLAN_GROUP as input */
	ROBOSW_WR(VLAN_TABLE_INDX_5395, vlan_group, sc);
	/* Execute READ vlan information */
	reg = VLAN_TABLE_ACCESS_5395_RUN | VLAN_TABLE_ACCESS_5395_READ;
	ROBOSW_WR(VLAN_TABLE_ACCESS_5395, reg, sc);
	/* Read result */
	ROBOSW_RD(VLAN_TABLE_ENTRY_5395, reg, sc);

#if 0
	printf("reg[%d] = 0x%x\n",vlan_group, reg);
#endif

	*vlan_id = ETHERSWITCH_VID_VALID | vlan_group; /* XXX: ??? */
	*members = ROBOSW_UNSHIFT(reg, VLAN_RW_MEMBER_5395); /* TODO: shift MUST be 9 */
	*untagged = ROBOSW_UNSHIFT(reg, VLAN_RW_UNTAGGED_5395);
	/* TODO: forwarding */
	*forward_id = 0; // RTL8366RB_VMCR_FID(vmcr);

	return (0);
}

static int
bcm53115_set_vlan_group(struct robosw_softc *sc, int vlan_group,
    int vlan_id, int members, int untagged, int forward_id)
{
	uint32_t		 reg;

	/* TODO: check range of vlan group */
	/* Set new entry */
	reg = ROBOSW_SHIFT(untagged, VLAN_RW_UNTAGGED_5395);
	reg |= ROBOSW_SHIFT(members, VLAN_RW_MEMBER_5395);
	ROBOSW_WR(VLAN_TABLE_ENTRY_5395, reg, sc);
	/* Set VLAN_GROUP as input */
	ROBOSW_WR(VLAN_TABLE_INDX_5395, vlan_group, sc);
	/* Execute READ vlan information */
	reg = VLAN_TABLE_ACCESS_5395_RUN;
	ROBOSW_WR(VLAN_TABLE_ACCESS_5395, reg, sc);

	return (0);
}
