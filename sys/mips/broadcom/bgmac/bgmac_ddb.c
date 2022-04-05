/*-
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

/*
 * DDB command for debugging of Broadcom GMAC like:
 *  - TBD
 */

#include "opt_ddb.h"

#ifdef DDB
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/queue.h>

#include <machine/bus.h>

#include <ddb/ddb.h>
#include <ddb/db_lex.h>

#define BHND_LOGGING	BHND_TRACE_LEVEL

#include "bgmac.h"
#include "bgmacreg.h"

#ifdef B53PRINT
#undef B53PRINT
#endif
#define	B53PRINT(dev)		db_printf(

static struct command_table db_bgmac_table = LIST_HEAD_INITIALIZER(db_t4_table);
_DB_SET(_show, bgmac, NULL, db_show_table, 0, &db_bgmac_table);

DB_FUNC(read, db_show_bgmacread, db_bgmac_table, CS_OWN, NULL)
{
	struct bgmac_softc	*sc;
	device_t		 dev;
	uint32_t		 reg;
	int			 t;
	int			 init;

	init = 0;
	t = db_read_token();
	if (t == tIDENT) {
		dev = device_lookup_by_name(db_tok_string);
		t = db_read_token();
		if (t == tNUMBER) {
			reg = db_tok_number;
			init = 1;
		}
	}

	db_skip_to_eol();

	if (init) {
		sc = device_get_softc(dev);
		db_printf("0x%x: 0x%x\n", reg, bus_read_4(sc->mem, reg));
	}
	else
		db_printf("usage: show bgmac read <bgmac_device> <reg>\n");

	return;
}

DB_FUNC(write, db_show_bgmacwrite, db_bgmac_table, CS_OWN, NULL)
{
	struct bgmac_softc	*sc;
	device_t		 dev;
	uint32_t		 reg;
	uint32_t		 val;
	int			 t;
	int			 init;

	init = 0;

	t = db_read_token();
	if (t == tIDENT) {
		dev = device_lookup_by_name(db_tok_string);
		t = db_read_token();
		if (t == tNUMBER) {
			reg = db_tok_number;
			t = db_read_token();
			if (t == tNUMBER) {
				val = db_tok_number;
				init = 1;
			}
		}
	}

	db_skip_to_eol();

	if (init) {
		sc = device_get_softc(dev);
		bus_write_4(sc->mem, reg, val);
	}
	else
		db_printf("usage: show bgmac write <bgmac_device> <register> "
		    "<value>\n");
	return;
}

DB_FUNC(dump, db_show_bgmacdump, db_bgmac_table, CS_OWN, NULL)
{
	struct bgmac_softc	*sc;
	device_t		 dev;
	int			 t;

	dev = NULL;
	t = db_read_token();
	if (t == tIDENT) {
		dev = device_lookup_by_name(db_tok_string);
	}

	db_skip_to_eol();

	if (dev != NULL) {
		sc = device_get_softc(dev);
		bgmac_print_debug(sc);
	} else
		db_printf("usage: show bgmac dump <bgmac>\n");

	return;
}

#endif
