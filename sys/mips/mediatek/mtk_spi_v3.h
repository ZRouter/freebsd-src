/*-
 * Copyright (c) 2009, Oleksandr Tymoshenko <gonzo@FreeBSD.org>
 * Copyright (c) 2011, Aleksandr Rybalko <ray@FreeBSD.org>
 * Copyright (c) 2013, Alexander A. Mityaev <sansan@adm.ua>
 * Copyright (c) 2016, Stanislav Galabov <sgalabov@gmail.com>
 * Copyright (c) 2023, Hiroki Mori
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
 *
 * $FreeBSD$
 */

#ifndef _MTK_SPIVAR_H_
#define _MTK_SPIVAR_H_

/* SPI controller interface */

#define MTK_SPISTAT0		0x00
/* SPIBUSY is alias for SPIBUSY, because SPISTAT have only BUSY bit*/
#define MTK_SPIBUSY0		MTK_SPISTAT0

#define MTK_SPICFG0		0x10
#define		ADDRMODE		(1<<12)
#define		RXENVDIS		(1<<11)
#define		RXCAP			(1<<10)
#define		SPIENMODE		(1<<9)
#define		MSBFIRST		(1<<8)
#define		SPICLKPOL		(1<<6)
#define		CAPT_ON_CLK_FALL	(1<<5)
#define		TX_ON_CLK_FALL		(1<<4)
#define		HIZSPI			(1<<3)	/* Set SPI pins to Tri-state */
#define		SPI_CLK_SHIFT		0	/* SPI clock divide control */
#define		SPI_CLK_MASK		0x00000007
#define		SPI_CLK_DIV2		0
#define		SPI_CLK_DIV4		1
#define		SPI_CLK_DIV8		2
#define		SPI_CLK_DIV16		3
#define		SPI_CLK_DIV32		4
#define		SPI_CLK_DIV64		5
#define		SPI_CLK_DIV128		6
#define		SPI_CLK_DISABLED	7

#define MTK_SPICTL0		0x14
#define		STARTSPI		(1<<4)
#define		HIZSMOSI		(1<<3)
#define		START_WRITE		(1<<2)
#define		START_READ		(1<<1)
#define		CS_HIGH			(1<<0)

#define MTK_SPIDATA0		0x20
#define		SPIDATA_MASK		0x000000ff

#define MTK_SPIADDR0		0x24
#define MTK_SPIBS0		0x28
#define MTK_SPIUSER0		0x2c
#define		USER_MODE		(1<<21)
#define		INSER_PHASE_SHIFT	20
#define 	USER_NO_INSTR		0x0
#define 	USER_ONE_INSTR		0x1

#define		ADDR_PHASE_SHIFT	17
#define 	USER_NO_ADDR		0x0
#define 	USER_ONE_BYTE_ADDR	0x1
#define 	USER_TWO_BYTE_ADDR	0x2
#define 	USER_THREE_BYTE_ADDR	0x3
#define 	USER_FOUR_BYTE_ADDR	0x4

#define		MODE_PHASE_SHIFT	16
#define 	USER_NO_MODE		0x0
#define 	USER_ONE_MODE		0x1

#define		DUMMY_PHASE_SHIFT	14
#define 	USER_NO_DUMMY		0x0
#define 	USER_ONE_DUMMY		0x1
#define 	USER_TWO_DUMMY		0x2
#define 	USER_THREE_DUMMY	0x3

#define		DATA_PHASE_SHIFT	12
#define 	USER_NO_DATA		0x0
#define 	USER_READ_DATA		0x1
#define 	USER_WRITE_DATA		0x2

#define 	ADDR_TYPE_SHIFT		9
#define 	MODE_TYPE_SHIFT		6
#define 	DUMMY_TYPE_SHIFT	3
#define 	DATA_TYPE_SHIFT		0
#define 	USER_SINGLE		0x1
#define 	USER_DUAL		0x2
#define 	USER_QUAD		0x4

#define MTK_SPITXFIFO0		0x30
#define MTK_SPIRXFIFO0		0x34
#define MTK_SPIFIFOSTAT0	0x38
#define MTK_SPIMD0		0x3c

#define MTK_SPISTAT1		0x40
#define MTK_SPIBUSY1		MTK_SPISTAT1
#define MTK_SPICFG1		0x50
#define MTK_SPICTL1		0x54
#define MTK_SPIDATA1		0x60

#define MTK_SPIDMA		0x80
#define MTK_SPIDMASTAT		0x84
#define MTK_SPIARB		0xf0

#define MTK_SPI_WRITE		1
#define MTK_SPI_READ		0

#define MTK_SPI_FIFOSIZE	16

#endif /* _MTK_SPIVAR_H_ */
