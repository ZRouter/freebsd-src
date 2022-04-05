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

static int	bcm5358_get_vlan_group(struct robosw_softc *sc, int vlan_group,
		    int *vlan_id, int *members, int *untagged, int *forward_id);
static int	bcm5358_set_vlan_group(struct robosw_softc *sc, int vlan_group,
		    int vlan_id, int members, int untagged, int forward_id);
static int	bcm5358_noreset(struct robosw_softc *sc);

struct robosw_functions bcm5358_f = {
	.api.reset = bcm5358_noreset,
	.api.vlan_get_vlan_group = bcm5358_get_vlan_group,
	.api.vlan_set_vlan_group = bcm5358_set_vlan_group
};

struct robosw_hal bcm5358_hal = {
	.parent = &bcm5325_hal,
	.self = &bcm5358_f
};

static int
bcm5358_noreset(struct robosw_softc *sc)
{

	/* Reset is unsupported */
	sc->sc_full_reset = ROBOSW_NORESET;
	return (0);
}

static int
bcm5358_get_vlan_group(struct robosw_softc *sc, int vlan_group,
    int *vlan_id, int *members, int *untagged, int *forward_id)
{
	uint32_t	reg;
	int		error;

	/* Set VLAN_GROUP as input */
	reg = VLAN_TABLE_ACCESS_RW_ENABLE;
	reg|= vlan_group & VLAN_TABLE_ACCESS_VID_MASK;
	error = robosw_write4(sc, VLAN_TABLE_ACCESS, reg);
	if (error) {
		device_printf(sc->sc_dev, "can't write to VLAN_TABLE_ACCESS"
		    "[%d]: %d\n", vlan_group, error);
		return (error);
	}

	/* Read VLAN information */
	error = robosw_op(sc, VLAN_READ, &reg, 0);
	if (error) {
		device_printf(sc->sc_dev, "can't read from VLAN_READ[%d]: %d\n",
		    vlan_group, error);
		return (error);
	}

	/* Check if it valid */
	if (!(reg & VLAN_RW_VALID_5358)) {
		device_printf(sc->sc_dev, "not a valid VLAN[%d] info = 0x%x\n",
		    vlan_group, reg);
		return (ENOENT);
	}

#if 0
	printf("reg[%d] = 0x%x\n", vlan_group, reg);
#endif
	*vlan_id = ETHERSWITCH_VID_VALID | vlan_group;
	*members = ROBOSW_UNSHIFT(reg, VLAN_RW_MEMBER);
	*untagged = ROBOSW_UNSHIFT(reg, VLAN_RW_UNTAGGED);
	/* TODO: forwarding */
	*forward_id = 0;

	return (0);
}

static int
bcm5358_set_vlan_group(struct robosw_softc *sc, int vlan_group,
    int vlan_id, int members, int untagged, int forward_id)
{
	uint32_t		 reg;
	int			 error;

	/* TODO: check range of vlan group */

	reg = VLAN_RW_VALID_5358;
	reg|= vlan_group << 12;
	reg|= ROBOSW_SHIFT(members, VLAN_RW_MEMBER);
	reg|= ROBOSW_SHIFT(untagged, VLAN_RW_UNTAGGED);
	error = robosw_write4(sc, VLAN_WRITE, reg);
	if (error) {
		device_printf(sc->sc_dev, "can't write to VLAN_WRITE"
		    "[%d]: %d\n", vlan_group, error);
		return (error);
	}
	/* Set VLAN_GROUP as input */
	reg = VLAN_TABLE_ACCESS_RW_ENABLE;
	reg|= VLAN_TABLE_ACCESS_WRITE;
	reg|= vlan_group & VLAN_TABLE_ACCESS_VID_MASK;
	error = robosw_write4(sc, VLAN_TABLE_ACCESS, reg);
	if (error) {
		device_printf(sc->sc_dev, "can't write to VLAN_TABLE_ACCESS"
		    "[%d]: %d\n", vlan_group, error);
		return (error);
	}

	return (0);
}
