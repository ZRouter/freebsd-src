/*-
 * Copyright (c) 2011 Aleksandr Rybalko.
 * Copyright (c) 2016 Michael Zhilin.
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

#ifndef _ROBOSW_VAR_H_
#define _ROBOSW_VAR_H_

#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/callout.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>

#include <dev/etherswitch/etherswitch.h>
#include <dev/mii/mii.h>

#define	ROBOSW_NUM_PHYS		9

/* VLAN 0,1 and 4095 are reserved, so default VLAN ID is 2 */
#define	ROBOSW_DEF_VLANID	2
#define ROBOSW_DEF_MASK		0x11e

MALLOC_DECLARE(M_BCMSWITCH);

#define	ROBOSWHALSIZE		10

typedef void (*voidfunctype) (void);

struct robosw_softc;
struct robosw_hal;

struct robosw_api {
	/* Initialization functions */
	int (* reset) (struct robosw_softc *sc);
	void (* init_context) (struct robosw_softc *sc);

	/* MIB data */
	uint32_t (* mib_get) (struct robosw_softc *sc, int port, int metric_id);

	/* Port-based VLAN functions */
	int (* vlan_get_pvlan_group) (struct robosw_softc *sc, int vlan_group,
	    int *vlan_id, int *members, int *untagged, int *forward_id);
	int (* vlan_set_pvlan_group) (struct robosw_softc *sc, int vlan_group,
	    int vlan_id, int members, int untagged, int forward_id);

	/* .1q VLAN functions */
	int (* vlan_enable_1q) (struct robosw_softc *sc, int on);
	int (* vlan_get_pvid) (struct robosw_softc *sc, int port, int *pvid);
	int (* vlan_set_pvid) (struct robosw_softc *sc, int port, int pvid);
	int (* vlan_get_vlan_group) (struct robosw_softc *sc, int vlan_group,
	    int *vlan_id, int *members, int *untagged, int *forward_id);
	int (* vlan_set_vlan_group) (struct robosw_softc *sc, int vlan_group,
	    int vlan_id, int members, int untagged, int forward_id);
};

struct robosw_functions {
	union{
		struct robosw_api	api;
		voidfunctype		func[ROBOSWHALSIZE];
	};
};

struct robosw_hal {
	char			*chipname;
	struct robosw_hal	*parent;
	struct robosw_functions	*self;
};

#define	ROBOSW_PARENT_API(hal)	(&((hal).parent->self->api))

struct robosw_softc {
	struct mtx	 sc_mtx;		/* serialize access to softc */
	device_t	 sc_dev;
	device_t	 sc_parent;
	int		 sc_full_reset;	/* see possible values below */
	int		 sc_debug; 	/* debug */
	char		*chipname;	/* chip id */
	int		 media;		/* cpu port media */
	int		 cpuport;	/* which PHY is connected to the CPU */
	int		 phymask;	/* PHYs we manage */
	int		 numports;	/* number of ports */
	uint32_t	 vlan_mode;	/* port-based or 1q */
	int		 ifpport[MII_NPHY];
	int		*portphy;
	char		*ifname[ROBOSW_NUM_PHYS];
	device_t	 miibus[ROBOSW_NUM_PHYS];
	struct ifnet	*ifp[ROBOSW_NUM_PHYS];
	struct callout	 callout_tick;
	struct robosw_functions	hal;
	struct etherswitch_info	info;
};

/* full reset possible values */
#define	ROBOSW_NORESET		0	/* 0b00 - not required/supported */
#define	ROBOSW_TRYRESET		1	/* 0b01 - try reset */
#define	ROBOSW_RESETDONE	2	/* 0b10 - reset done */
#define	ROBOSW_RESETFAIL	3	/* 0b11 - reset failed */

/* MIB metrics */
#define	ROBOSW_MIB_GOODRXPKTS	0
#define	ROBOSW_MIB_GOODTXPKTS	1

#define ROBOSW_LOCK(_sc)			mtx_lock(&(_sc)->sc_mtx)
#define ROBOSW_UNLOCK(_sc)			mtx_unlock(&(_sc)->sc_mtx)
#define ROBOSW_LOCK_ASSERT(_sc, _what)	mtx_assert(&(_sc)->sc_mtx, (_what))
#define ROBOSW_TRYLOCK(_sc)		mtx_trylock(&(_sc)->sc_mtx)

/* Read/write access to SWITCH registers via PSEUDO PHY */
uint32_t	robosw_read4(struct robosw_softc *sc, uint32_t reg);
int		robosw_write4(struct robosw_softc *sc, uint32_t reg, uint32_t val);
int		robosw_op(struct robosw_softc *sc, uint32_t reg, uint32_t *res,
		    int is_write);

/* Etherswitch interface */
void		robosw_lock(device_t dev);
void		robosw_unlock(device_t dev);
int		robosw_getvgroup(device_t dev, etherswitch_vlangroup_t *vg);
int		robosw_setvgroup(device_t dev, etherswitch_vlangroup_t *vg);
int		robosw_getport(device_t dev, etherswitch_port_t *p);
int		robosw_setport(device_t dev, etherswitch_port_t *p);

#define ROBOSW_RD(_reg, _val, _sc)						\
	do { 								\
		int	robosw_err; 					\
		robosw_err = robosw_op(_sc, _reg, &_val, 0);		\
		if (robosw_err) {						\
			device_printf(_sc->sc_dev, "can't read"		\
			    " " #_reg ", err: %d\n", robosw_err);		\
			return (robosw_err);				\
		}							\
	} while (0);

#define ROBOSW_WR(_reg, _val, _sc)						\
	do { 								\
		int	robosw_err; 					\
		robosw_err = robosw_op(_sc, _reg, &_val, 1);		\
		if (robosw_err) {						\
			device_printf(_sc->sc_dev, "can't write"	\
			    " " #_reg ", err: %d\n", robosw_err);		\
			return (robosw_err);				\
		}							\
	} while (0);

/* Chip operations */
int	robosw_reset(device_t dev);

/* Common chip actions: */
/*	- enable/disable forwarding */
int	robosw_enable_fw(device_t dev, uint32_t forward);

#endif /* _ROBOSW_VAR_H_ */
