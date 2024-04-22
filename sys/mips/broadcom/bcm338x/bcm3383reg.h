/*
 * Copyright (c) 2024 Hiroki Mori
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. All advertising materials mentioning features or use of this
 *    software must display the following acknowledgements:
 *      This product includes software developed by the Urbana-Champaign
 *      Independent Media Center.
 *	This product includes software developed by Garrett D'Amore.
 * 4. Urbana-Champaign Independent Media Center's name and Garrett
 *    D'Amore's name may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE URBANA-CHAMPAIGN INDEPENDENT
 * MEDIA CENTER AND GARRETT D'AMORE ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE URBANA-CHAMPAIGN INDEPENDENT
 * MEDIA CENTER OR GARRETT D'AMORE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	_MIPS_BROADCOM_BCM3383REG_H_
#define	_MIPS_BROADCOM_BCM3383REG_H_

#define	BCM3383_INTC_BASE	0xb4e00000
#define	BCM3383_TIMER_BASE	0xb4e000c0
#define	BCM3383_GPIO_BASE	0xb4e00100
#define	BCM3383_UART0_BASE	0xb4e00500
#define	BCM3383_UART1_BASE	0xb4e00520
#define	BCM3383_I2C_BASE	0xb4e00e00
#define BCM3383_LED_BASE	0xb4e00f00
#define BCM3383_SPIM_BASE	0xb4e01000

#define	BCM3383_EHCI_BASE	0x12e00000
#define	BCM3383_OHCI_BASE	0x12e00100
#define	BCM3383_USBCTL_BASE	0xb2e00200

#define	BCM3383_UNIMAC_BASE	0xb2c00000
#define	BCM3383_GMAC0_BASE	0xb2c00600
#define	BCM3383_GMAC1_BASE	0xb2c02600

#define BCM_READ_REG(reg) \
    *((volatile uint32_t *)MIPS_PHYS_TO_KSEG1((reg)))
 
#define BCM_WRITE_REG(reg, val) \
    *((volatile uint32_t *)MIPS_PHYS_TO_KSEG1((reg))) = (val)

#endif
