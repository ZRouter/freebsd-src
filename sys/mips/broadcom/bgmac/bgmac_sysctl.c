/*-
 * Copyright (c) 2018 Michael Zhilin <mizhka@freebsd.org>
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
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/rman.h>
#include <sys/sbuf.h>

#include "bgmac.h"

int
bgmac_sysctl_mib(SYSCTL_HANDLER_ARGS)
{
	struct bgmac_softc	*sc;
	uint32_t		 value;
	int			 metric;

	sc = (struct bgmac_softc *) arg1;
	metric = (arg2 & 0xF);

	switch (metric) {
	case 0:
		value = (
			bus_read_4(sc->mem, BGMAC_MIB_RX_PCKTS) +
			bus_read_4(sc->mem, BGMAC_MIB_RX_BROADCAST_PCKTS) +
			bus_read_4(sc->mem, BGMAC_MIB_RX_MULTICAST_PCKTS)
			);
		break;
	case 1:
		value = bus_read_4(sc->mem, BGMAC_MIB_TX_PCKTS);
		break;
	default:
		value = 0;
	}

	return (sysctl_handle_16(oidp, NULL, value, req));
}

int
bgmac_sysctl_dump(SYSCTL_HANDLER_ARGS)
{
	struct bgmac_softc	*sc;
	struct sbuf 		 sbuf;
	int			 error;

	sc = (struct bgmac_softc *) arg1;
	sbuf_new_for_sysctl(&sbuf, NULL, 128, req);
	sbuf_printf(&sbuf, "\n");

#undef	BGMACDUMPREG
#define BGMACDUMPREG(sc, _reg)						\
	sbuf_printf(&sbuf, #_reg "\t=%08x\n", bus_read_4(sc->mem, _reg));

#undef	BGMACDUMPREGB
#define BGMACDUMPREGB(sc, _reg, _bits)					\
	sbuf_printf(&sbuf, #_reg "\t=%b\n", bus_read_4(sc->mem, _reg), _bits);

#undef	BGMACDUMPREGF
#define BGMACDUMPREGF(sc, _reg, _func)					\
	do {								\
		uint32_t val = bus_read_4(sc->mem, _reg);		\
		char* desc = _func(val);				\
		sbuf_printf(&sbuf,  #_reg "\t=%08x (%s)\n", val, desc);	\
		free(desc, M_TEMP);					\
	} while (0);

	BGMACDUMP(sc);
	BGMACDUMPMIB(sc);
	BGMACDUMPERRORS(sc);

	error = sbuf_finish(&sbuf);
	sbuf_delete(&sbuf);
	return (error);
}
