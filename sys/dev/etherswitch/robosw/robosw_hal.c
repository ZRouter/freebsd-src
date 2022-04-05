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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * HAL routines for Broadcom RoboSwitch / 53xx ethernet switch
 */
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/bus.h>

#ifdef WITH_BHND
#include <dev/bhnd/bhnd.h>
#include "bhnd_bus_if.h"
#endif

#include "robosw_reg.h"
#include "robosw_hal.h"

struct robosw_hal_mapping {
	uint32_t		 id;
	struct robosw_hal	*hal;
};

static struct robosw_hal_mapping integrated_ids[] = {
	{0x5358, 	&bcm5358_hal},
	{0x5357, 	&bcm5358_hal},
	{53572,		&bcm5358_hal},
	{0, 		NULL}
};

static struct robosw_hal_mapping outband_ids[] = {
	{0x53115,	&bcm53115_hal},
	{0x53125,	&bcm53115_hal},	/* BananaPiR1 SWITCH_DEVICEID=00053125*/
	{0, NULL}
};

static struct robosw_hal *	robosw_hal_lookup(struct robosw_softc *sc);

void
robosw_hal_init(struct robosw_softc *sc)
{
	struct robosw_functions		*scfunc, *initfunc;
	struct robosw_hal		*inithal;

	inithal = robosw_hal_lookup(sc);
	sc->chipname = inithal->chipname;

	scfunc = &sc->hal;
	for (;;) {
		initfunc = inithal->self;

		/* inherit functions from initial HAL */
		for (int i = 0; i < ROBOSWHALSIZE; i++)
			if (scfunc->func[i] == NULL)
				scfunc->func[i] = initfunc->func[i];

		if (inithal->parent == NULL)
			break;

		/* has parent, next cycle */
		inithal = inithal->parent;
	}
}

static struct robosw_hal *
robosw_hal_lookup(struct robosw_softc *sc)
{
#ifdef WITH_BHND
	const struct bhnd_chipid	*chip;
#endif
	struct robosw_hal_mapping	*ptr;
	char				*map_type;
	uint32_t 		 	 switchid;
	int				 err;
	device_t			 dev;

	dev = sc->sc_dev;
	err = robosw_op(sc, SWITCH_DEVICEID, &switchid, 0);
	if (err || switchid == 0) {
		/* assume integrated etherswitch */
#ifdef WITH_BHND
		chip = BHND_BUS_GET_CHIPID(device_get_parent(dev), dev);
		if (chip != NULL)
			switchid = chip->chip_id;
#endif
		ptr = integrated_ids;
		map_type = "integrated";
	} else {
		ptr = outband_ids;
		map_type = "external";
	}

	for(; ptr->id != 0; ptr++)
		if (ptr->id == switchid) {
			return ptr->hal;
		}

	/* HAL isn't found in mapping, use default */
	return (&bcm5325_hal);
}
