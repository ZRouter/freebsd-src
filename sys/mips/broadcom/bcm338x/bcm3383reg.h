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

enum bcm338x_soc_type {
	BCM338X_SOC_UNKNOWN,
	BCM338X_SOC_BCM3380,
	BCM338X_SOC_BCM3381,
	BCM338X_SOC_BCM3382,
	BCM338X_SOC_BCM3383,
	BCM338X_SOC_BCM3384,
};

extern enum bcm338x_soc_type bcm338x_soc;

#define	BCM3383_UNIMAC_BASE	0x12c00000
#define	BCM3383_GMAC0_BASE	0x12c00600
#define	BCM3383_GMAC1_BASE	0x12c02600

#define	BCM3383_EHCI_BASE	0x12e00000
#define	BCM3383_OHCI_BASE	0x12e00100
#define	BCM3383_USBCTL_BASE	0x12e00200

#define	BCM3383_PMC_BASE	0x13e00000

#define	BCM3383_PERIPH_BASE	0x14e00000
#define	BCM3383_INTC_BASE	0x14e00000
#define	BCM3383_TIMER_BASE	0x14e000c0
#define	BCM3383_GPIO_BASE	0x14e00100
#define	BCM3383_UART0_BASE	0x14e00500
#define	BCM3383_UART1_BASE	0x14e00520
#define	BCM3383_I2C_BASE	0x14e00e00
#define	BCM3383_LED_BASE	0x14e00f00
#define	BCM3383_SPIM_BASE	0x14e01000

#define	BCM3380_UART0_BASE	0x14e00200
#define	BCM3380_UART1_BASE	0x14e00220

#define INTERRUPT_ID_TIMER		0
#define INTERRUPT_ID_SPI		1
#define INTERRUPT_ID_UART0		2
#define INTERRUPT_ID_UART1		3
#define INTERRUPT_ID_SIMCARD0		4
#define INTERRUPT_ID_SIMCARD1		5
#define INTERRUPT_ID_I2C		6
#define INTERRUPT_ID_HS_SPI		7
#define INTERRUPT_ID_RING_OSC		8
#define INTERRUPT_ID_PERIPH_ERR		9
#define INTERRUPT_ID_RESERVED_10	10
#define INTERRUPT_ID_RESERVED_11	11
#define INTERRUPT_ID_RESERVED_12	12
#define INTERRUPT_ID_RESERVED_13	13
#define INTERRUPT_ID_RESERVED_14	14
#define INTERRUPT_ID_PCIE_RC		15
#define INTERRUPT_ID_PCIE_EP_LNK_RST	16
#define INTERRUPT_ID_BRG_UBUS0		17
#define INTERRUPT_ID_BRG_UBUS1		18
#define INTERRUPT_ID_FPM		19
#define INTERRUPT_ID_USB0		20
#define INTERRUPT_ID_USB1		21
#define INTERRUPT_ID_APM		22
#define INTERRUPT_ID_USB_OHCI		23
#define INTERRUPT_ID_USB_EHCI		24
#define INTERRUPT_ID_UNI_IRQ		25
#define INTERRUPT_ID_GPHY_IRQB		26
#define INTERRUPT_ID_DAVIC		27
#define INTERRUPT_ID_OB			28
#define INTERRUPT_ID_RESERVED_29	29
#define INTERRUPT_ID_RESERVED_30	30
#define INTERRUPT_ID_EXT_IRQ		31

#define UART_INT_TXCHARDONE		0x8000
#define UART_INT_RXBRK			0x4000
#define UART_INT_RXPARERR		0x2000
#define UART_INT_RXFRAMERR		0x1000
#define UART_INT_RXFIFONE		0x0800
#define UART_INT_RXFIFOTHOLD		0x0400
#define UART_INT_RXFIFOFULL		0x0200
#define UART_INT_RXTIMEOUT		0x0100
#define UART_INT_RXOVFERR		0x0080
#define UART_INT_RXUNDERR		0x0040
#define UART_INT_TXFIFOEMT		0x0020
#define UART_INT_TXREADLATCH		0x0010
#define UART_INT_TXFIFOTHOLD		0x0008
#define UART_INT_TXOVFERR		0x0004
#define UART_INT_TXUNDERR		0x0002
#define UART_INT_DELTAIP		0x0001

#define BCM_READ_REG(reg) \
    *((volatile uint32_t *)MIPS_PHYS_TO_KSEG1((reg)))
 
#define BCM_WRITE_REG(reg, val) \
    *((volatile uint32_t *)MIPS_PHYS_TO_KSEG1((reg))) = (val)

#endif
