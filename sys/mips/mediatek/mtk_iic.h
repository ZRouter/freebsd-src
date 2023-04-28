/*-
 * Copyright (c) 2017 Hiroki Mori
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

#ifndef _MTK_I2C_NEWVAR_H_
#define	_MTK_I2C_NEWVAR_H_

/* Helpers from NetBSD */
/* __BIT(n): nth bit, where __BIT(0) == 0x1. */
#define	__BIT(__n)      \
	(((__n) >= NBBY * sizeof(uintmax_t)) ? 0 : ((uintmax_t)1 << (__n)))

/*
 * I2C controller interface
 * Copy from netbsd code
 */

#define	RA_I2C_CONFIG			0x00
#define	RA_I2C_CLKDIV			0x04
#define	RA_I2C_DEVADDR			0x08
#define	RA_I2C_ADDR			0x0C
#define	RA_I2C_DATAOUT			0x10
#define	RA_I2C_DATAIN			0x14
#define	RA_I2C_STATUS			0x18
#define	RA_I2C_STARTXFR			0x1C
#define	RA_I2C_BYTECNT			0x20
#define	 I2C_CONFIG_ADDRLEN(x)		(((x) & 0x7) << 5)
#define	  I2C_CONFIG_ADDRLEN_7		6
#define	  I2C_CONFIG_ADDRLEN_8		7
#define	 I2C_CONFIG_DEVADLEN(x)		(((x) & 0x7) << 2)
#define	  I2C_CONFIG_DEVADLEN_6		5
#define	  I2C_CONFIG_DEVADLEN_7		6
#define	 I2C_CONFIG_ADDRDIS		__BIT(1)
#define	 I2C_CONFIG_DEVDIS		__BIT(0)
#define	 I2C_STATUS_STARTERR		__BIT(4)
#define	 I2C_STATUS_ACKERR		__BIT(3)
#define	 I2C_STATUS_DATARDY		__BIT(2)
#define	 I2C_STATUS_SDOEMPTY		__BIT(1)
#define	 I2C_STATUS_BUSY		__BIT(0)

#define	I2C_OP_READ			1
#define	I2C_OP_WRITE			0

/*
 * This is RT2880 default value. Please set clock-div property in dts file 
 * other socs.
 */
#define	RT2880_CLKDIV_VALUE		333

#define	i2c_busy_loop			(sc->clkdiv*30)
#define	max_ee_busy_loop		(sc->clkdiv*25)

#endif /* _MTK_I2C_NEWVAR_H_ */
