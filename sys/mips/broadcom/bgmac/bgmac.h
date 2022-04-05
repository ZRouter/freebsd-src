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
 */

#ifndef _MIPS_BROADCOM_BGMAC_BGMAC_H_
#define _MIPS_BROADCOM_BGMAC_BGMAC_H_

#include <sys/bus.h>
#include <sys/sysctl.h>
#include <machine/bus.h>

#if 1
#define	BHND_LOGGING	BHND_TRACE_LEVEL
#define KTR_BGMAC	KTR_DEV
#else
#define	BHND_LOGGING	BHND_INFO_LEVEL
#define KTR_BGMAC	0
#endif

#include <dev/bhnd/bhnd_debug.h>

#include "bgmacreg.h"
#include "bgmacvar.h"

#define BGMACDUMPREG(sc, _reg)						\
	BHND_INFO_DEV(sc->dev, #_reg "\t=%08x", bus_read_4(sc->mem, _reg))

#define BGMACDUMPREGB(sc, _reg, _bits)					\
	BHND_INFO_DEV(sc->dev, #_reg "\t=%b", bus_read_4(sc->mem, _reg), _bits)

#define BGMACDUMPREGF(sc, _reg, _func)					\
	do {								\
		uint32_t val = bus_read_4(sc->mem, _reg);		\
		char* desc = _func(val);				\
		BHND_INFO_DEV(sc->dev,  #_reg "\t=%08x (%s)", val, desc)	\
		free(desc, M_TEMP);						\
	} while (0);

#define	BUFSIZE		80

static inline char*
bgmac_decode_ctl(uint32_t val)
{
	int		 len;
	char		*buf;

	len = 0;
	buf = malloc(BUFSIZE * sizeof(char), M_TEMP, M_WAITOK);
	if (buf == NULL)
		return "";

#define PRINT(...)							\
	if (BUFSIZE > len)						\
		len += snprintf(buf + len, BUFSIZE - len, ## __VA_ARGS__);

#define PRBOOL(_mask, _prf, _false, _true)				\
	PRINT(_prf, (((val & _mask) == 0) ? _false : _true))

#define PRRAW(_mask, _prf)						\
	PRINT(_prf, (val & _mask))

	PRBOOL(BGMAC_REG_DMA_RX_CTRL_ENABLE, "%s,", "OFF", "ON");
	PRBOOL(BGMAC_REG_DMA_RX_CTRL_PIOMODE, "%s,", "DMA", "PIO");
	PRBOOL(BGMAC_REG_DMA_RX_CTRL_SRXHDENABLE, "%s,", "", "SRXHD");
	PRBOOL(BGMAC_REG_DMA_RX_CTRL_OVERFLOW_CONT, "%s,", "", "OVERFLOWCNT");
	PRBOOL(BGMAC_REG_DMA_RX_CTRL_DISABLE_PARITYCHK, "%s,", "PARITYCHK", "");
	PRRAW(BGMAC_REG_DMA_RX_CTRL_FRAMEOFFSET, "off:%08x,");
	PRRAW(BGMAC_REG_DMA_RX_CTRL_ADDR_EXTENSION, "ext:%08x");

#undef PRINT
#undef PRBOOL
#undef PRRAW
	return (buf);
}

#define BGMACDUMP(sc)							\
	BGMACDUMPREGF(sc,BGMAC_REG_DMA_TX_CTRL, bgmac_decode_ctl)	\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_TX_INDEX)				\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_TX_RINGLOW)			\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_TX_RINGHIGH)			\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_TX_STATE)				\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_TX_ERROR)				\
	BGMACDUMPREGF(sc,BGMAC_REG_DMA_RX_CTRL, bgmac_decode_ctl)	\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_RX_INDEX);			\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_RX_RINGLOW);			\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_RX_RINGHIGH);			\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_RX_STATE);			\
	BGMACDUMPREG(sc,BGMAC_REG_DMA_RX_ERROR);			\
	BGMACDUMPREGB(sc,BGMAC_REG_CMD_CFG,BGMAC_REG_CMD_CFG_BITS);	\
	BGMACDUMPREG(sc,BGMAC_REG_RX_MAX_LEN);				\
	BGMACDUMPREG(sc,BGMAC_REG_DEVICE_CONTROL);			\
	BGMACDUMPREG(sc,BGMAC_REG_DEVICE_STATUS);			\
	BGMACDUMPREG(sc,BGMAC_REG_BIST_STATUS);				\
	BGMACDUMPREGB(sc,BGMAC_REG_INTR_STATUS,BGMAC_REG_INTR_STATUS_BITS);	\
	BGMACDUMPREG(sc,BGMAC_REG_INTERRUPT_MASK);			\
	BGMACDUMPREG(sc,BGMAC_REG_GP_TIMER);				\
	BGMACDUMPREG(sc,BGMAC_REG_INTR_RECV_LAZY);			\
	BGMACDUMPREG(sc,BGMAC_PWR_CTL);					\
	BGMACDUMPREG(sc,BGMAC_FLOW_CTL_THRESH);				\
	BGMACDUMPREG(sc,BGMAC_PAUSE_CTL);				\
	BGMACDUMPREG(sc,BGMAC_REG_TXQ_CTL);				\
	BGMACDUMPREG(sc,BGMAC_REG_RXQ_CTL);				\
	BGMACDUMPREG(sc,BGMAC_CLOCK_CONTROL_ST);

#define BGMACDUMPMIB(sc)						\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_BYTES);				\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_PCKTS);				\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_DEFERED);				\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_Q0_PKTS);				\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_Q0_OCTETS);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_GOOD_BYTES);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_GOOD_PCKTS);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_BYTES);				\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_PCKTS);

#define BGMACDUMPERRORS(sc)						\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_ERR_JABBER_PKTS);			\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_ERR_OVERSIZE_PKTS);		\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_ERR_UNDERRUNS);			\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_ERR_EXCESSIVE_COLS);		\
	BGMACDUMPREG(sc,BGMAC_MIB_TX_ERR_LATE_COLS);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_ERR_JABBER_PCKTS);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_ERR_OVERSIZE_PCKTS);		\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_ERR_MISSED_PCKTS);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_ERR_CRC_ALIGN);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_ERR_UNDERSIZE);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_ERR_CRC);				\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_ERR_ALIGN);			\
	BGMACDUMPREG(sc,BGMAC_MIB_RX_ERR_SYMBOL);

void	bgmac_if_start(struct ifnet *ifp);
void	bgmac_print_debug(struct bgmac_softc* sc);
int	bgmac_sysctl_dump(SYSCTL_HANDLER_ARGS);
int	bgmac_sysctl_mib(SYSCTL_HANDLER_ARGS);

#endif /* _MIPS_BROADCOM_BGMAC_BGMAC_H_ */
