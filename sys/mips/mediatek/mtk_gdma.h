/*-
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

#ifndef _MTK_GDMA_H_
#define _MTK_GDMA_H_

#define CONFIG_RALINK_MT7620

/* GDMA controller interface */

#if defined (CONFIG_RALINK_MT7621) || defined (CONFIG_ARCH_MT7623)
#define MAX_GDMA_CHANNEL		16
#elif defined (CONFIG_RALINK_RT3052)
#define MAX_GDMA_CHANNEL		8
#elif defined (CONFIG_RALINK_RT3883) || defined (CONFIG_RALINK_RT3352) || defined (CONFIG_RALINK_RT5350) || defined (CONFIG_RALINK_RT6855) || defined (CONFIG_RALINK_RT6855A) || defined (CONFIG_RALINK_MT7620) || defined (CONFIG_RALINK_MT7628)
#define MAX_GDMA_CHANNEL		16
#endif

/*
#define RALINK_GDMA_CTRL_BASE		(RALINK_GDMA_BASE)
*/
#define RALINK_GDMA_CTRL_BASE		0

#if defined (CONFIG_RALINK_RT3052)
#define RALINK_GDMAISTS			(RALINK_GDMA_BASE + 0x80)
#define RALINK_GDMAGCT			(RALINK_GDMA_BASE + 0x88)
#elif defined (CONFIG_RALINK_RT3883) || defined (CONFIG_RALINK_RT3352) || defined (CONFIG_RALINK_RT5350) || defined (CONFIG_RALINK_RT6855) || defined (CONFIG_RALINK_RT6855A) || defined (CONFIG_RALINK_MT7620)  ||  defined (CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628) || defined (CONFIG_ARCH_MT7623)
#define RALINK_GDMA_UNMASKINT		(RALINK_GDMA_BASE + 0x200)
#define RALINK_GDMA_DONEINT		(RALINK_GDMA_BASE + 0x204)
#define RALINK_GDMA_GCT			(RALINK_GDMA_BASE + 0x220)
#endif

/*
#define GET_GDMA_IP_VER			(GDMA_READ_REG(RALINK_GDMA_GCT) & 0x6) >> 1 //GDMA_GCT[2:1]
*/

#define RALINK_IRQ_ADDR                 RALINK_INTCL_BASE
#if defined (CONFIG_RALINK_MT7621) || defined (CONFIG_ARCH_MT7628)
#define RALINK_REG_INTENA               (RALINK_IRQ_ADDR + 0x80)
#define RALINK_REG_INTDIS               (RALINK_IRQ_ADDR + 0x78)
#else
#define RALINK_REG_INTENA               (RALINK_IRQ_ADDR + 0x34)
#define RALINK_REG_INTDIS               (RALINK_IRQ_ADDR + 0x38)
#endif

/* 
 * 12bytes=GDMA Channel n Source Address(4) +
 *         GDMA Channel n Destination Address(4) +
 *         GDMA Channel n Control Register(4)
 *
 */
#define GDMA_SRC_REG(ch)		(RALINK_GDMA_BASE + ch*16)
#define GDMA_DST_REG(ch)		(GDMA_SRC_REG(ch) + 4)
#define GDMA_CTRL_REG(ch)		(GDMA_DST_REG(ch) + 4)
#define GDMA_CTRL_REG1(ch)		(GDMA_CTRL_REG(ch) + 4)

/* GDMA Interrupt Status Register */
#if defined (CONFIG_RALINK_RT3052)
#define UNMASK_INT_STATUS(ch)           (ch+16)
#elif defined (CONFIG_RALINK_RT3883) || defined (CONFIG_RALINK_RT3352) || defined (CONFIG_RALINK_RT5350) || defined (CONFIG_RALINK_RT6855) || defined (CONFIG_RALINK_RT6855A) || defined (CONFIG_RALINK_MT7620)  ||  defined (CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628) || defined (CONFIG_ARCH_MT7623)
#define UNMASK_INT_STATUS(ch)           (ch)
#endif
#define TXDONE_INT_STATUS(ch)           (ch)

/* Control Reg0 */
#define MODE_SEL_OFFSET			0
#define CH_EBL_OFFSET			1
#define CH_DONEINT_EBL_OFFSET		2
#define BRST_SIZE_OFFSET		3
#define DST_BRST_MODE_OFFSET		6
#define SRC_BRST_MODE_OFFSET		7
#define TRANS_CNT_OFFSET		16

/* Control Reg1 */
#if defined (CONFIG_RALINK_RT3052)
#define CH_UNMASKINT_EBL_OFFSET		4
#define NEXT_UNMASK_CH_OFFSET		1
#elif defined (CONFIG_RALINK_RT3883) || defined (CONFIG_RALINK_RT3352) || defined (CONFIG_RALINK_RT5350) || defined (CONFIG_RALINK_RT6855) || defined (CONFIG_RALINK_RT6855A) || defined (CONFIG_RALINK_MT7620)  ||  defined (CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628) || defined (CONFIG_ARCH_MT7623)
#define CH_UNMASKINT_EBL_OFFSET		1
#define NEXT_UNMASK_CH_OFFSET		3
#endif
#define COHERENT_INT_EBL_OFFSET		2
#define CH_MASK_OFFSET			0

#if defined (CONFIG_RALINK_RT3052)
/* Control Reg0 */
#define DST_DMA_REQ_OFFSET		8
#define SRC_DMA_REQ_OFFSET		12
#elif defined (CONFIG_RALINK_RT3883) || defined (CONFIG_RALINK_RT3352) || defined (CONFIG_RALINK_RT5350) || defined (CONFIG_RALINK_RT6855) || defined (CONFIG_RALINK_RT6855A) || defined (CONFIG_RALINK_MT7620)  ||  defined (CONFIG_RALINK_MT7621) || defined (CONFIG_RALINK_MT7628) || defined (CONFIG_ARCH_MT7623)
/* Control Reg1 */
#define DST_DMA_REQ_OFFSET		8
#define SRC_DMA_REQ_OFFSET		16
#endif

#define GDMA_PCM0_RX0			0
#define GDMA_PCM0_RX1			1
#define GDMA_PCM0_TX0			2
#define GDMA_PCM0_TX1			3

#define GDMA_PCM1_RX0			4
#define GDMA_PCM1_RX1			5
#define GDMA_PCM1_TX0			6
#define GDMA_PCM1_TX1			7

#define GDMA_PCM_RX(i,j)		(0+((i)<<2)+j)
#define GDMA_PCM_TX(i,j)		(2+((i)<<2)+j)

#define GDMA_I2S_TX0			4
#define GDMA_I2S_TX1			5
#define GDMA_I2S_RX0			6
#define GDMA_I2S_RX1			7

#define GDMA_SPI_TX			13
#define GDMA_SPI_RX			12

#endif /* _MTK_GDMA_H_ */
