/*
 * ath79-i2s-pll.c -- ALSA DAI PLL management for QCA AR71xx/AR9xxx designs
 *
 * Copyright (c) 2012 Qualcomm-Atheros Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <mips/atheros/ar71xxreg.h>
#include <mips/atheros/ar934xreg.h>

#include <mips/atheros/ar71xx_cpudef.h>
#include <mips/atheros/ar71xx_setup.h>

#include <mips/atheros/ar71xx_chip.h>

#include <mips/atheros/ar71xx_pcmvar.h>

#include "ar71xx_pcm_pll.h"

static const struct ath79_pll_config pll_cfg_25MHz[] = {
	/* Freq		divint	divfrac		ppllpwd	bypass	extdiv	refdiv	PS	ki	kd	shift */
	/* 		-----------------------PLL----------------------------	STEREO	--------DPLL--------- */
	{ 22050,	0x15,	0x2B442,	0x3,	0,	0x6,	0x1,	3,	0x4,	0x3d,	0x6 },
	{ 32000,	0x17,	0x24F76,	0x3,	0,	0x6,	0x1,	3,	0x4,	0x3d,	0x6 },
	{ 44100,	0x15,	0x2B442,	0x3,	0,	0x6,	0x1,	2,	0x4,	0x3d,	0x6 },
	{ 48000,	0x17,	0x24F76,	0x3,	0,	0x6,	0x1,	2,	0x4,	0x3d,	0x6 },
	{ 88200,	0x15,	0x2B442,	0x3,	0,	0x6,	0x1,	1,	0x4,	0x3d,	0x6 },
	{ 96000,	0x17,	0x24F76,	0x3,	0,	0x6,	0x1,	1,	0x4,	0x3d,	0x6 },
	{ 0,		0,	0,		0,	0,	0,	0,	0,	0,	0,	0   },
};

static const struct ath79_pll_config pll_cfg_40MHz[] = {
	{ 22050,	0x1b,	0x6152,		0x3,	0,	0x6,	0x2,	3,	0x4,	0x32,	0x6 },
	{ 32000,	0x1d,	0x1F6FD,	0x3,	0,	0x6,	0x2,	3,	0x4,	0x32,	0x6 },
	{ 44100,	0x1b,	0x6152,		0x3,	0,	0x6,	0x2,	2,	0x4,	0x32,	0x6 },
	{ 48000,	0x1d,	0x1F6FD,	0x3,	0,	0x6,	0x2,	2,	0x4,	0x32,	0x6 },
	{ 88200,	0x1b,	0x6152,		0x3,	0,	0x6,	0x2,	1,	0x4,	0x32,	0x6 },
	{ 96000,	0x1d,	0x1F6FD,	0x3,	0,	0x6,	0x2,	1,	0x4,	0x32,	0x6 },
	{ 0,		0,	0,		0,	0,	0,	0,	0,	0,	0,	0   },
};

static void ath79_pll_powerup(void)
{
int reg;

	reg = ATH_READ_REG(AR71XX_PLL_AUDIO_CONFIG);
	reg = reg & ~(1 << 5);
	ATH_WRITE_REG(AR71XX_PLL_AUDIO_CONFIG, reg);
}

static void ath79_pll_powerdown(void)
{
int reg;

	reg = ATH_READ_REG(AR71XX_PLL_AUDIO_CONFIG);
	reg = reg | (1 << 5);
	ATH_WRITE_REG(AR71XX_PLL_AUDIO_CONFIG, reg);
}


static void ath79_audiodpll_do_meas_clear(void)
{
int reg;

	reg = ATH_READ_REG(AR934X_SRIF_AUD_DPLL3_REG);
	reg = reg & ~AR934X_DPLL_3_DO_MEAS;
	ATH_WRITE_REG(AR934X_SRIF_AUD_DPLL3_REG, reg);
}

static void ath79_audiodpll_do_meas_set(void)
{
int reg;

	reg = ATH_READ_REG(AR934X_SRIF_AUD_DPLL3_REG);
	reg = reg | AR934X_DPLL_3_DO_MEAS;
	ATH_WRITE_REG(AR934X_SRIF_AUD_DPLL3_REG, reg);
}


static void ath79_audiodpll_range_set(void)
{
int reg;

	reg = ATH_READ_REG(AR934X_SRIF_AUD_DPLL2_REG);
	reg = reg & ~AR934X_DPLL_2_RANGE;
	ATH_WRITE_REG(AR934X_SRIF_AUD_DPLL2_REG, reg);
	reg = ATH_READ_REG(AR934X_SRIF_AUD_DPLL2_REG);
	reg = reg  | AR934X_DPLL_2_RANGE;
	ATH_WRITE_REG(AR934X_SRIF_AUD_DPLL2_REG, reg);
}


static bool ath79_audiodpll_meas_done_is_set(void)
{
int status;

	status = ATH_READ_REG(AR934X_SRIF_AUD_DPLL4_REG) &
	    AR934X_DPLL_4_MEAS_DONE;
	return(status ? 1 : 0);
}

static int ath79_audiodpll_sqsum_dvc_get(void)
{
int reg;

	reg = ATH_READ_REG(AR934X_SRIF_AUD_DPLL3_REG);
	reg = (reg & 0x007ffff8) >> 3;
	return reg;
}

static void ath79_load_pll_regs(const struct ath79_pll_config *cfg)
{
int reg;

	/* Set PLL regs. But still power down */
	ATH_WRITE_REG(AR71XX_PLL_AUDIO_CONFIG, (cfg->extdiv << 12) |
	    (cfg->postpllpwd << 7) | (cfg->bypass << 4) | cfg->refdiv |
	    (1 << 7) | (1 << 5));
	reg = (cfg->divfrac << 11) | (cfg->divint << 1);
	ATH_WRITE_REG(AR71XX_PLL_AUDIO_MOD, reg);

	ath79_audiodpll_range_set();

	/* Set DPLL regs */

	/* set phase shift */
	reg = ATH_READ_REG(AR934X_SRIF_AUD_DPLL3_REG);
	reg = reg & ~(AR934X_DPLL_3_PHASESH_MASK <<
	    AR934X_DPLL_3_PHASESH_SHIFT);
	reg = reg | (cfg->shift << AR934X_DPLL_3_PHASESH_SHIFT);
	ATH_WRITE_REG(AR934X_SRIF_AUD_DPLL3_REG, reg);

	/* get gains */
	reg = ATH_READ_REG(AR934X_SRIF_AUD_DPLL2_REG);
	reg = reg & ~(AR934X_DPLL_2_KI_MASK << AR934X_DPLL_2_KI_SHIFT);
	reg = reg | (cfg->ki << AR934X_DPLL_2_KI_SHIFT);
	reg = reg & ~(AR934X_DPLL_2_KD_MASK << AR934X_DPLL_2_KD_SHIFT);
	reg = reg | (cfg->kd << AR934X_DPLL_2_KD_SHIFT);
	ATH_WRITE_REG(AR934X_SRIF_AUD_DPLL2_REG, reg);

}

void ar934x_pcm_setpll(struct ar71xx_pcm_softc *sc, int freq)
{
device_t dev;
const struct ath79_pll_config *cfg;
int mhz, reg;
 
	dev = sc->dev;

	mhz = u_ar71xx_refclk / (1000 * 1000);
	if (mhz == 25)
		cfg = &pll_cfg_25MHz[0];
	else if (mhz == 40)
		cfg = &pll_cfg_40MHz[0];
	else {
		device_printf(dev, "Unsupported CPU clock\n");
		return;
	}

	while (cfg->rate != 0) {
		if (cfg->rate == freq)
			break;
		++cfg;
	}

	if (cfg->rate == 0) {
		device_printf(dev, "Unsupported frequency %d\n", freq);
		return;
	}

	do {
		ath79_audiodpll_do_meas_clear();
		ath79_pll_powerdown();
		DELAY(100);

		ath79_load_pll_regs(cfg);

		ath79_pll_powerup();
		ath79_audiodpll_do_meas_clear();
		ath79_audiodpll_do_meas_set();

		while ( ! ath79_audiodpll_meas_done_is_set()) {
			DELAY(10);
		}
	} while (ath79_audiodpll_sqsum_dvc_get() >= 0x40000);

}
