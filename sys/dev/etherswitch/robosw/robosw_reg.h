/*-
 * Copyright (c) 2011 Aleksandr Rybalko.
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

#ifndef _ROBOSWSWITCHREG_H_
#define _ROBOSWSWITCHREG_H_

#define	ROBOSW_READOP	0
#define	ROBOSW_WRITEOP	1

#define	ROBOSW_SHIFT(val, SHF)		((val << SHF ## _SHIFT) & SHF ## _MASK)
#define	ROBOSW_UNSHIFT(val, SHF)	((val & SHF ## _MASK) >> SHF ## _SHIFT)

/*
 * Macroses for READ / WRITE over parent MDIO / MII
 */
#define ROBOSW_WRITEREG(r, v)						\
	if (MDIO_WRITEREG(sc->sc_parent, ROBOSW_PSEUDOPHY_ADDR, r, v)) {\
		device_printf(sc->sc_dev, "WRITEREG failed: %x/%x\n", r,\
		    v); 						\
	}

#define ROBOSW_READREG(reg)						\
	MDIO_READREG(sc->sc_parent, ROBOSW_PSEUDOPHY_ADDR, reg)

#define	ROBOSWPRINT(_dev)		device_printf(_dev,

#define ROBOSWDUMPREG(_reg)						\
	ROBOSWPRINT(dev) #_reg "=%08x\n", robosw_read4(sc, _reg))

#define	ROBOSWPORTBITS	"\020\001p0\002p1\003p2\004p3\005p4\006p5\007p6"
#define	ROBOSWDUMPVLAN(_dev, _start, _end, _prefix)			\
	do {								\
		struct etherswitch_vlangroup 	vg;			\
		int				robosw_err;		\
		for(int i = _start; i < _end; i++) {			\
			vg.es_vlangroup = i;				\
			robosw_err = robosw_getvgroup(_dev, &vg);	\
			ROBOSWPRINT(_dev) _prefix "[%d] err=%d;\t"	\
					"members=%b\tuntagged=%b\n", i,	\
					robosw_err,			\
					vg.es_member_ports,		\
					ROBOSWPORTBITS,			\
					vg.es_untagged_ports,		\
					ROBOSWPORTBITS);		\
		}							\
	} while (0);

#define ROBOSWDUMP(sc, dev)					\
	if (sc->sc_debug) {					\
		ROBOSWDUMPREG(PORT_CTL(PORT0));			\
		ROBOSWDUMPREG(PORT_CTL(PORT1));			\
		ROBOSWDUMPREG(PORT_CTL(PORT2));			\
		ROBOSWDUMPREG(PORT_CTL(PORT3));			\
		ROBOSWDUMPREG(PORT_CTL(PORT4));			\
		ROBOSWDUMPREG(PORT_CTL(PORT5));			\
		ROBOSWDUMPREG(PORT_CTL(PORT6));			\
		ROBOSWDUMPREG(PORT_CTL(PORT7));			\
		ROBOSWDUMPREG(PORT_CTL(PORTMII));		\
		ROBOSWDUMPREG(PORTMII_STATUS_OVERRIDE);		\
		ROBOSWDUMPREG(SWITCH_DEVICEID);			\
		ROBOSWDUMPREG(SWITCH_MODE);			\
		ROBOSWDUMPREG(POWER_DOWN_MODE);			\
		ROBOSWDUMPREG(LINK_STATUS_SUMMARY);		\
		ROBOSWDUMPREG(PORT_SPEED_100M);			\
		ROBOSWDUMPREG(BIST_STATUS_RC);			\
		ROBOSWDUMPREG(VLAN_GLOBAL_CTL0);		\
		ROBOSWDUMPREG(VLAN_GLOBAL_CTL1);		\
		ROBOSWDUMPREG(VLAN_GLOBAL_CTL2);		\
		ROBOSWDUMPREG(VLAN_DROP_UNTAGGED);		\
		ROBOSWDUMPREG(VLAN_GLOBAL_CTL4);		\
		ROBOSWDUMPREG(VLAN_GLOBAL_CTL5);		\
		ROBOSWDUMPREG(VLAN_GLOBAL_CTL5_531xx);		\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(0));	\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(1));	\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(2));	\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(3));	\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(4));	\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(5));	\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(6));	\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(7));	\
		ROBOSWDUMPREG(VLAN_DEFAULT_PORT_TAG(8));	\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(0));		\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(1));		\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(2));		\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(3));		\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(4));		\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(5));		\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(6));		\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(7));		\
		ROBOSWDUMPREG(PBVLAN_ALLOWED_PORTS(8));		\
		ROBOSWDUMPREG(MIB_TX_OCTETS(0));		\
		ROBOSWDUMPREG(MIB_TX_OCTETS(1));		\
		ROBOSWDUMPREG(MIB_RX_OCTETS_531xx(0));		\
		ROBOSWDUMPREG(MIB_RX_OCTETS_531xx(1));		\
		ROBOSWDUMPREG(MIB_TX_OCTETS(8));		\
		ROBOSWDUMPREG(MIB_RX_OCTETS_531xx(8));		\
		ROBOSWDUMPVLAN(dev, 0, 4, "VLAN");		\
	}

#define	ROBOSW_PSEUDOPHY_ADDR	0x1e
#define	ROBOSW_PHYMASK		(1 << PSEUDOPHY_ADDR)
#define	PAGE_REG		0xff

/*
 * Software-used shifts (not related to HW)
 */
#define ROBOSW_LEN_MASK			0x00ff0000
#define ROBOSW_LEN_SHIFT		16
#define ROBOSW_PAGE_MASK		0x0000ff00
#define ROBOSW_PAGE_SHIFT		8
#define ROBOSW_REG_MASK			0x000000ff
#define ROBOSW_REG_SHIFT		0

#define	ROBOSW_OP_RETRY			100

/*
 * Hardware registers, masks and etc
 */

#define	ROBOSW_ACCESS_CONTROL_REG		0x10
#define		ACCESS_CONTROL_PAGE_MASK	0xff00
#define		ACCESS_CONTROL_PAGE_SHIFT	8
#define		ACCESS_CONTROL_RW		0x0001

#define	ROBOSW_RW_CONTROL_REG			0x11
#define		RW_CONTROL_ADDR_MASK		0xff00
#define		RW_CONTROL_ADDR_SHIFT		8
#define		RW_CONTROL_NOP			0x0000
#define		RW_CONTROL_WRITE		0x0001
#define		RW_CONTROL_READ			0x0002
#define		RW_CONTROL_OP_MASK		0x0003

#define	ROBOSW_RW_STATUS_REG			0x12
#define		RW_STATUS_ERROR			0x0002
#define		RW_STATUS_PROHIBIT		0x0001

#define	ROBOSW_DATA_REG_BASE		0x18
#define	ROBOSW_DATA_31_16_REG		0x19
#define	ROBOSW_DATA_47_32_REG		0x1a
#define	ROBOSW_DATA_63_48_REG		0x1b

#define	PORT0			0
#define	PORT1			1
#define	PORT2			2
#define	PORT3			3
#define	PORT4			4
#define	PORT5			5
#define	PORT6			6
#define	PORT7			7
#define	PORTMII			8
#define	PORTGLB			9
#define	PORT_SERIAL_MGMT	10

#define	PAGE(_page)	((_page) << 8)
/* Size */
#define	S1		(1 << 16)
#define	S2		(2 << 16)
#define	S3		(3 << 16)
#define	S4		(4 << 16)
#define	S6		(6 << 16)
#define	S8		(8 << 16)

#define PORT_CTL(_port)			(S1 | PAGE(0x00) | ((_port) & 0x0f))
#define		PORT_CTL_STP_STATE_MASK		0xe0
#define		PORT_CTL_STP_STATE_NOSTP	0x00
#define		PORT_CTL_STP_STATE_DISABLED	0x20
#define		PORT_CTL_STP_STATE_BLOCKING	0x40
#define		PORT_CTL_STP_STATE_LISTENING	0x60
#define		PORT_CTL_STP_STATE_LEARNING	0x80
#define		PORT_CTL_STP_STATE_FORWARDING	0xa0
#define		PORTMII_CTL_UCAST_ENABLED	0x10 /* MII port only */
#define		PORTMII_CTL_MCAST_ENABLED	0x08 /* MII port only */
#define		PORTMII_CTL_BCAST_ENABLED	0x04 /* MII port only */
#define		PORT_CTL_TX_DISABLED		0x02
#define		PORT_CTL_RX_DISABLED		0x01

#define SWITCH_MODE			(S1 | PAGE(0x00) | 0x0b)
#define 	SWITCH_MODE_HDX_NORETRY_LIMIT	0x04
#define 	SWITCH_MODE_FORWARDING_ENABLED	0x02
#define 	SWITCH_MODE_MANAGED		0x01

#define PORTMII_STATUS_OVERRIDE		(S1 | PAGE(0x00) | 0x0e)
#define 	PORTMII_STATUS_FORCE_1000M	0x80	/* it is force? */
#define 	PORTMII_STATUS_REVERSE_MII	0x10
#define 	PORTMII_STATUS_PAUSE_CAPABLE	0x08
#define 	PORTMII_STATUS_FORCE_100M	0x04
#define 	PORTMII_STATUS_FORCE_FDX	0x02
#define 	PORTMII_STATUS_FORCE_LINK	0x01

#define POWER_DOWN_MODE			(S1 | PAGE(0x00) | 0x0f)
#define		PORT7_POWER_DOWN		0x80
#define		PORT6_POWER_DOWN		0x40
#define		PORT5_POWER_DOWN		0x20
#define		PORT4_POWER_DOWN		0x10
#define		PORT3_POWER_DOWN		0x08
#define		PORT2_POWER_DOWN		0x04
#define		PORT1_POWER_DOWN		0x02
#define		PORT0_POWER_DOWN		0x01 /* Do not set bit 0 to 1.
						      * Doing so will disable
						      * the PLL power and the
						      * switch function. */

#define	IP_MULTICAST_REG		(S1 | PAGE(0x00) | 0x21)
#define		IP_MULTICAST_ENABLE		0x01

#define	IO_MUX_CTL			(S2 | PAGE(0x00) | 0x22)
#define	SWITCH_RESET			(S1 | PAGE(0x00) | 0x79)
#define		SWITCH_RESET_ON			0x80
#define		SWITCH_RESET_ENABLE		0x10

#define	LINK_STATUS_SUMMARY		(S2 | PAGE(0x01) | 0x00)
#define	LINK_STATUS_CHANGE_RC		(S2 | PAGE(0x01) | 0x02)
#define	PORT_SPEED_100M			(S4 | PAGE(0x01) | 0x04)
#define	PORT_DUPLEX_FDX(_x)		(S2 | PAGE(0x01) | 0x06)
#define	PAUSE_SUMMARY(_x)		(S2 | PAGE(0x01) | 0x08)
#define	SOURCE_ADDRESS_CHANGE(_x)	(S2 | PAGE(0x01) | 0x0c)

#define	LAST_PORT_SOURCE_ADDRESS(_p, _x)	\
		(S6 | PAGE(0x01) | (0x10 + (((_p) & 0x07) * 6)))

#define	BIST_STATUS_RC			(S1 | PAGE(0x01) | 0x46)
#define		BIST_STATUS_VLAN_RAM_ERR	0x10
#define		BIST_STATUS_VID_RAM_ERR		0x08
#define		BIST_STATUS_MIB_RAM_ERR		0x04
#define		BIST_STATUS_MEM_ERR		0x02
#define		BIST_STATUS_BUF_CTL_RAM_ERR	0x01

#define	GLOBAL_MGMT_CTL			(S1 | PAGE(0x02) | 0x00)
#define		GLOBAL_MGMT_CTL_MGMT_PORT_MII	0x80
#define		GLOBAL_MGMT_CTL_MGMT_PORT_PHY	0x00
#define		GLOBAL_MGMT_CTL_MIB_AC_EN	0x20
#define		GLOBAL_MGMT_CTL_MIB_AC_HDR	0x10
#define		GLOBAL_MGMT_CTL_IGMP_IP		0x08
#define		GLOBAL_MGMT_CTL_IGMP_MAC	0x04
#define		GLOBAL_MGMT_CTL_RX_BPDU		0x02
#define		GLOBAL_MGMT_CTL_RST_MIB		0x01

#define	MGMT_PORT			(S1 | PAGE(0x02) | 0x02)
#define	RMON_MIB_STEERING		(S2 | PAGE(0x02) | 0x04)
#define	AGE_TIME			(S4 | PAGE(0x02) | 0x06)
#define	PORT_MIRROR			(S2 | PAGE(0x02) | 0x10)
#define		PORT_MIRROR_ENABLE		0x8000
#define		PORT_MIRROR_PORT(_p)		(1 << (_p))

#define	PORT_IN_MIRROR_CTL		(S2 | PAGE(0x02) | 0x12)
#define		PORT_IN_MIRROR_ALL		0x0000
#define		PORT_IN_MIRROR_DA_MATCH		0x4000
#define		PORT_IN_MIRROR_SA_MATCH		0x8000
#define		PORT_IN_MIRROR_DIV_EN		0x2000
#define		PORT_IN_MIRROR_PORT(_p)		(1 << (_p))

#define	PORT_IN_MIRROR_DIV		(S2 | PAGE(0x02) | 0x14)
#define	PORT_IN_MIRROR_MAC		(S6 | PAGE(0x02) | 0x16)
#define	PORT_OUT_MIRROR_CTL		(S2 | PAGE(0x02) | 0x1c)
#define		PORT_OUT_MIRROR_ALL		0x0000
#define		PORT_OUT_MIRROR_DA_MATCH	0x4000
#define		PORT_OUT_MIRROR_SA_MATCH	0x8000
#define		PORT_OUT_MIRROR_DIV_EN		0x2000
#define		PORT_OUT_MIRROR_PORT(_p)	(1 << (_p))

#define	PORT_OUT_MIRROR_DIV		(S2 | PAGE(0x02) | 0x1e)
#define	PORT_OUT_MIRROR_MAC		(S6 | PAGE(0x02) | 0x20)

#define	IGMP_CPU_FWD_CTL		(S1 | PAGE(0x02) | 0x26)
#define	IGMP_REPLACE_DA			(S6 | PAGE(0x02) | 0x27)

#define	SWITCH_DEVICEID			(S4 | PAGE(0x02) | 0x30)

#define	MIB_AUTOCAST_PORT	(S2 | PAGE(0x03) | 0x00)
#define	MIB_AUTOCAST_HDR_PNTR	(S2 | PAGE(0x03) | 0x02)
#define		MIB_AC_HDR_VALID		0x8000
#define		MIB_AC_HDR_PTR_MASK		0x03ff
#define	MIB_AUTOCAST_HDR_LEN	(S2 | PAGE(0x03) | 0x04)
#define		MIB_AC_HDR_LEN_MASK		0x00ff
#define	MIB_AUTOCAST_DA		(S6 | PAGE(0x03) | 0x06)
#define	MIB_AUTOCAST_SA		(S6 | PAGE(0x03) | 0x0C)
#define	MIB_AUTOCAST_TYPE	(S2 | PAGE(0x03) | 0x12)
#define	MIB_AUTOCAST_RATE	(S2 | PAGE(0x03) | 0x14)

#define	GLOBAL_ARL_CONFIG	(S1 | PAGE(0x04) | 0x00)
#define		GLOBAL_ARL_MPORT_ADDR_EN	0x10
#define		GLOBAL_ARL_HASH_DISABLE		0x01

#define	BPDU_MULTICAST_ADDRESS	(S6 | PAGE(0x04) | 0x04)
#define	MULTIPORT_ADDRESS_1	(S6 | PAGE(0x04) | 0x10)
#define	MULTIPORT_VECTOR_1	(S2 | PAGE(0x04) | 0x16)
#define		MULTIPORT_VECTOR_1_PORT(_p)	(1 << (_p))
#define	MULTIPORT_ADDRESS_2	(S6 | PAGE(0x04) | 0x20)
#define	MULTIPORT_VECTOR_2	(S2 | PAGE(0x04) | 0x26)
#define		MULTIPORT_VECTOR_2_PORT(_p)	(1 << (_p))
#define	SECURE_SRC_PORT_MASK	(S2 | PAGE(0x04) | 0x30)
#define		SECURE_SRC_PORT(_p)		(1 << (_p))
#define	SECURE_DST_PORT_MASK	(S2 | PAGE(0x04) | 0x32)
#define		SECURE_DST_PORT(_p)		(1 << (_p))

#define	ARL_RW_CTL		(S1 | PAGE(0x05) | 0x00)
#define		ARL_RW_CTL_START		0x80
#define		ARL_RW_CTL_DONE			0x80
#define		ARL_RW_CTL_READ			0x01
#define	MAC_ADDRESS_INDEX	(S6 | PAGE(0x05) | 0x02)
#define	VID_TABLE_INDEX		(S1 | PAGE(0x05) | 0x08)
#define	ARL_ENTRY_0		(S8 | PAGE(0x05) | 0x10)
#define		ARL_ENTRY_H32_VALID		0x80000000
#define		ARL_ENTRY_H32_STATIC		0x40000000
#define		ARL_ENTRY_H32_AGE		0x20000000
/* For Unicast port number */
#define		ARL_ENTRY_H32_UCAST_PORTID_NUM_MASK	0x000f0000
#define		ARL_ENTRY_H32_UCAST_PORTID_NUM_SHIFT	16
/* For Multicast ports map */
#define		ARL_ENTRY_H32_MCAST_PORTID_MAP_MASK	0x03ff0000
#define		ARL_ENTRY_H32_MCAST_PORTID_MAP_SHIFT	16
#define		ARL_ENTRY_H32_MAC_HIGH_MASK	0x0000ffff
#define		ARL_ENTRY_L32_MAC_LOW_MASK	0xffffffff
#define	ARL_ENTRY_1		(S8 | PAGE(0x05) | 0x18)
#define	ARL_SEARCH_CTL		(S1 | PAGE(0x05) | 0x20)
#define		ARL_SEARCH_CTL_START		0x80
#define		ARL_SEARCH_CTL_DONE		0x80
#define		ARL_SEARCH_CTL_VALID		0x01
#define	ARL_SEARCH_ADDRESS	(S2 | PAGE(0x05) | 0x22)
#define		ARL_SEARCH_ADDRESS_VALID	0x8000
#define		ARL_SEARCH_ADDRESS_MASK		0x7fff
#define	ARL_SEARCH_RESULT	(S8 | PAGE(0x05) | 0x24) /* ARL Entry format */
#define	ARL_SEARCH_RESULT_EXT	(S1 | PAGE(0x05) | 0x2c) /* 5350 */
#define	VID_ENTRY_0		(S1 | PAGE(0x05) | 0x30)
#define	VID_ENTRY_1		(S1 | PAGE(0x05) | 0x32)
#define ARL_SEARCH_CTL_53115		(S1 | PAGE(0x05) | 0x50)
#define ARL_SEARCH_ADDRESS_53115	(S2 | PAGE(0x05) | 0x51)
#define ARL_SEARCH_RESULT_53115		(S8 | PAGE(0x05) | 0x60)
#define ARL_SEARCH_RESULT_EXT_53115	(S2 | PAGE(0x05) | 0x68)

#define	MEMORY_RW_CTL		(S4 | PAGE(0x08) | 0x00)
#define		MEMORY_RW_CTL_START		0x00080000
#define		MEMORY_RW_CTL_DONE		0x00080000
#define		MEMORY_RW_CTL_READ		0x00040000
#define		MEMORY_RW_CTL_ADDR_MASK		0x00003fff
#define	MEMORY_RW_DATA		(S8 | PAGE(0x08) | 0x04)

#define	PRIORITY_CTL		(S2 | PAGE(0x0A) | 0x00)
#define	FLOW_CTL		(S2 | PAGE(0x0A) | 0x30)
#define	LOW_QUEUE_THRESHOLD_CTL	(S2 | PAGE(0x0A) | 0x4A)
#define	QUEUE2_CTL_1		(S2 | PAGE(0x0A) | 0x66)
#define	QUEUE2_CTL_2		(S2 | PAGE(0x0A) | 0x68)
#define	QUEUE2_CTL_3		(S2 | PAGE(0x0A) | 0x6A)
#define	QUEUE2_CTL_4		(S2 | PAGE(0x0A) | 0x6C)
#define	QUEUE3_CTL_1		(S2 | PAGE(0x0A) | 0x74)
#define	QUEUE3_CTL_2		(S2 | PAGE(0x0A) | 0x76)
#define	QUEUE3_CTL_3		(S2 | PAGE(0x0A) | 0x78)
#define	QUEUE3_CTL_4		(S2 | PAGE(0x0A) | 0x7A)
#define	QUEUE4_CTL_1		(S2 | PAGE(0x0A) | 0x82)
#define	QUEUE4_CTL_2		(S2 | PAGE(0x0A) | 0x84)
#define	QUEUE4_CTL_3		(S2 | PAGE(0x0A) | 0x86)
#define	QUEUE4_CTL_4		(S2 | PAGE(0x0A) | 0x88)

/* Standard MII
PRT = PAGE10 + port
Page PRT, Address 00h-01h	Switch Port Control Register
Page PRT, Address 02h-03h	Switch Port Status Register
Page PRT, Address 04h-05h 	PHY Identifier Registers
Page PRT, Address 06h-07h 	PHY Identifier Registers
Page PRT, Address 08h-09h 	Auto-Negotiation Advertisement Register
Page PRT, Address 0Ah-0Bh 	Auto-Negotiation Link Partner Ability Register
Page PRT, Address 0Ch-0Dh 	Auto-Negotiation Expansion Register
Page PRT, Address 0Eh-0Fh 	Next Page Transmit Register
Page PRT, Address 10h-11h 	Link Partner Next Page Register
Page PRT, Address 20h-21h 	100BASE-X Auxiliary Control Register
Page PRT, Address 22h-23h 	100BASE-X Auxiliary Status Register
Page PRT, Address 24h-25h 	100BASE-X Receive Error Counter
Page PRT, Address 26h-27h 	100BASE-X False Carrier Sense Counter
Page PRT, Address 30h-31h 	Auxiliary Control/Status Register
Page PRT, Address 32h-33h 	Auxiliary Status Summary Register
Page PRT, Address 36h-37h 	Auxiliary Mode 2
Page PRT, Address 38h-39h 	10BASE-T Auxiliary Error & General Status
Page PRT, Address 3Ch-3Dh 	Auxiliary Multiple PHY Register
Page PRT, Address 3Eh-3Fh 	Broadcom Test
Page PRT, Address 1Eh-1Fh 	DPM Register
Page PRT, Address 34h-35h 	DPM Interrupt Register
*/

/* 5325 */
#define	MIB_5325_TX_GoodPkts(_p)	(S2 | PAGE(0x20 + (_p)) | 0x00)
#define	MIB_5325_TX_UnicastPkts(_p)	(S2 | PAGE(0x20 + (_p)) | 0x02)
#define	MIB_5325_RX_GoodPkts(_p)	(S2 | PAGE(0x20 + (_p)) | 0x04)
#define	MIB_5325_RX_UnicastPkts(_p)	(S2 | PAGE(0x20 + (_p)) | 0x06)

/* 53115 */
#define	MIB_53115_TX_DropPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x08)
#define	MIB_53115_TX_UnicastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x18)
#define	MIB_53115_TX_MulticastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x14)
#define	MIB_53115_TX_BroadcastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x10)
#define	MIB_53115_RX_DropPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x90)
#define	MIB_53115_RX_UnicastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x94)
#define	MIB_53115_RX_MulticastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x98)
#define	MIB_53115_RX_BroadcastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x9C)

#define	MIB_TX_OCTETS(_p)		(S8 | PAGE(0x20 + (_p)) | 0x00)
#define	MIB_TX_DROPS(_p)		(S4 | PAGE(0x20 + (_p)) | 0x08)
#define	MIB_TX_DropPkts(_p)		(S4 | PAGE(0x20 + (_p)) | 0x08)
#define	MIB_TX_BroadcastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x10)
#define	MIB_TX_MulticastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x14)
#define	MIB_TX_UnicastPkts(_p)		(S4 | PAGE(0x20 + (_p)) | 0x18)
#define	MIB_TX_Collisions(_p)		(S4 | PAGE(0x20 + (_p)) | 0x1C)
#define	MIB_TX_SingleCollision(_p)	(S4 | PAGE(0x20 + (_p)) | 0x20)
#define	MIB_TX_Multiple Collision(_p)	(S4 | PAGE(0x20 + (_p)) | 0x24)
#define	MIB_TX_DeferredTransmit(_p)	(S4 | PAGE(0x20 + (_p)) | 0x28)
#define	MIB_TX_LateCollision(_p)	(S4 | PAGE(0x20 + (_p)) | 0x2C)
#define	MIB_TX_ExcessiveCollision(_p)	(S4 | PAGE(0x20 + (_p)) | 0x30)
#define	MIB_TX_FrameInDisc(_p)		(S4 | PAGE(0x20 + (_p)) | 0x34)
#define	MIB_TX_PausePkts(_p)		(S4 | PAGE(0x20 + (_p)) | 0x38)

#define	MIB_RX_OCTETS_531xx(_p)		(S8 | PAGE(0x20 + (_p)) | 0x50)
#define	MIB_RX_OCTETS(_p)		(S8 | PAGE(0x20 + (_p)) | 0x44)
#define	MIB_RX_Undersize(_p)		(S4 | PAGE(0x20 + (_p)) | 0x4c)
#define	MIB_RX_PausePkts(_p)		(S4 | PAGE(0x20 + (_p)) | 0x50)
#define	MIB_RX_Pkts64Octets(_p)		(S4 | PAGE(0x20 + (_p)) | 0x54)
#define	MIB_RX_Pkts65to127Octets(_p)	(S4 | PAGE(0x20 + (_p)) | 0x58)
#define	MIB_RX_Pkts128to255Octets(_p)	(S4 | PAGE(0x20 + (_p)) | 0x5C)
#define	MIB_RX_Pkts256to511Octets(_p)	(S4 | PAGE(0x20 + (_p)) | 0x60)
#define	MIB_RX_Pkts512to1023Octets(_p)	(S4 | PAGE(0x20 + (_p)) | 0x64)
#define	MIB_RX_Pkts1024to1522Octets(_p)	(S4 | PAGE(0x20 + (_p)) | 0x68)
#define	MIB_RX_OversizePkts(_p)		(S4 | PAGE(0x20 + (_p)) | 0x6C)
#define	MIB_RX_Jabbers(_p)		(S4 | PAGE(0x20 + (_p)) | 0x70)
#define	MIB_RX_AlignmentErrors(_p)	(S4 | PAGE(0x20 + (_p)) | 0x74)
#define	MIB_RX_FCSErrors(_p)		(S4 | PAGE(0x20 + (_p)) | 0x78)
#define	MIB_RX_GOODOCTETS(_p)		(S8 | PAGE(0x20 + (_p)) | 0x7c)
#define	MIB_RX_DropPkts(_p)		(S4 | PAGE(0x20 + (_p)) | 0x84)
#define	MIB_RX_UnicastPkts(_p)		(S4 | PAGE(0x20 + (_p)) | 0x88)
#define	MIB_RX_MulticastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x8C)
#define	MIB_RX_BroadcastPkts(_p)	(S4 | PAGE(0x20 + (_p)) | 0x90)
#define	MIB_RX_SAChanges(_p)		(S4 | PAGE(0x20 + (_p)) | 0x94)
#define	MIB_RX_Fragments(_p)		(S4 | PAGE(0x20 + (_p)) | 0x98)
#define	MIB_RX_ExcessSizeDisc(_p)	(S4 | PAGE(0x20 + (_p)) | 0x9C)
#define	MIB_RX_SymbolError(_p)		(S4 | PAGE(0x20 + (_p)) | 0xA0)

#define	QOS_CTL			(S2 | PAGE(0x30) | 0x00)
#define		QOS_CTL_CPU_EN			0x8000
#define		QOS_CTL_AUTO_SET_QOS_REGS	0x1000
#define		QOS_CTL_NQUEUE_MASK		0x0c00
#define		QOS_CTL_4QUEUE			0x0c00
#define		QOS_CTL_3QUEUE			0x0800
#define		QOS_CTL_2QUEUE			0x0400
#define		QOS_CTL_1QUEUE			0x0000
#define		QOS_CTL_HIPRIO_PORTS_MASK	0x03ff
#define	QOS_PRIORITY_QUEUE_CTL	(S1 | PAGE(0x30) | 0x02)
#define	QOS_802_1P_ENABLE	(S2 | PAGE(0x30) | 0x04)
#define	QOS_TOS_DIFFSERV_ENABLE	(S2 | PAGE(0x30) | 0x06)
#define	QOS_PAUSE_ENABLE	(S2 | PAGE(0x30) | 0x13)
#define	DOT1P_PRIO_THRESHOLD	(S2 | PAGE(0x30) | 0x15)
#define		DOT1P_PRIO_TAG_QUEUE(_t, _q)	((_q) << ((_t) * 2))
#define	TOS_DIFFSERV_CTL	(S1 | PAGE(0x30) | 0x19)
#define		TOS_DIFFSERV_CTL_TOS		0x01
#define		TOS_DIFFSERV_CTL_DIFFSERV	0x00
#define	D_TYPE_TOS_PRIORITY	(S2 | PAGE(0x30) | 0x1A)
#define		D_TYPE_TOS_PREC_QUEUE(_t, _q)	((_q) << ((_t) * 2))
#define	T_TYPE_TOS_PRIORITY	(S2 | PAGE(0x30) | 0x1C)
#define		T_TYPE_TOS_PREC_QUEUE(_t, _q)	((_q) << ((_t) * 2))
#define	R_TYPE_TOS_PRIORITY	(S2 | PAGE(0x30) | 0x1E)
#define		R_TYPE_TOS_PREC_QUEUE(_t, _q)	((_q) << ((_t) * 2))
#define	M_TYPE_TOS_PRIORITY	(S2 | PAGE(0x30) | 0x20)
#define		M_TYPE_TOS_PREC_QUEUE(_t, _q)	((_q) << ((_t) * 2))
#define	DIFFSERV_DSCP_PRIORITYL	(S8 | PAGE(0x30) | 0x30)
#define		DSCP_TO_QUEUE(_d, _q)	((_q) << (((_d) & 0x3f) * 2))
#define	DIFFSERV_DSCP_PRIORITYH	(S8 | PAGE(0x30) | 0x38)
#define		DSCP_TO_QUEUE(_d, _q)	((_q) << (((_d) & 0x3f) * 2))

/* PAGE31, Port Based VLAN */
#define	PBVLAN_ALLOWED_PORTS(_p)	(S2 | PAGE(0x31) | ((_p) * 2))
#define 	PBVLAN_ALLOWED_PORTS_MASK	0x1FF

#define	VLAN_GLOBAL_CTL0		(S1 | PAGE(0x34) | 0x00)
#define		VLAN_GLOBAL_CTL0_1Q_ENABLE	0x80
#define		VLAN_GLOBAL_CTL0_MATCH_VIDMAC	0x40
#define		VLAN_GLOBAL_CTL0_MATCH_ONLYMAC	0x00
#define		VLAN_GLOBAL_CTL0_HASH_VIDADDR	0x20
#define		VLAN_GLOBAL_CTL0_HASH_ONLYADDR	0x00
#define		VLAN_GLOBAL_CTL0_INGRESS_CHECK	0x10
#define		VLAN_GLOBAL_CTL0_NO_CHANGE	0x00
#define		VLAN_GLOBAL_CTL0_CHANGE_P	0x04
#define		VLAN_GLOBAL_CTL0_CHANGE_V	0x08
#define		VLAN_GLOBAL_CTL0_CHANGE_PV	0x0c
#define		VLAN_GLOBAL_CTL0_NULV_NO_CHANGE	0x00
#define		VLAN_GLOBAL_CTL0_NULV_CHANGE_P	0x01
#define		VLAN_GLOBAL_CTL0_NULV_CHANGE_V	0x02
#define		VLAN_GLOBAL_CTL0_NULV_CHANGE_PV	0x03
#define	VLAN_GLOBAL_CTL1		(S1 | PAGE(0x34) | 0x01)
#define 	VLAN_GLOBAL_CTL1_MCAST_FWDMAP_CHECK	(1 << 3)
#define 	VLAN_GLOBAL_CTL1_MCAST_UNTAGMAP_CHECK	(1 << 2)
#define 	VLAN_GLOBAL_CTL1_MCAST_TAGGING		(1 << 1)
#define	VLAN_GLOBAL_CTL2		(S1 | PAGE(0x34) | 0x02)
#define	VLAN_DROP_UNTAGGED		(S1 | PAGE(0x34) | 0x03)
#define		VLAN_DROP_UNTAGGED_ONPORT(_p)	(1 << (_p))
#define	VLAN_GLOBAL_CTL4		(S1 | PAGE(0x34) | 0x04)
#define	VLAN_GLOBAL_CTL4_531xx		(S1 | PAGE(0x34) | 0x05)
#define	VLAN_GLOBAL_CTL4_63xx		(S1 | PAGE(0x34) | 0x06)
#define 	VLAN_GLOBAL_CTL4_DROP_VID_VIOLATION	(1 << 6)
#define		VLAN_GLOBAL_CTL4_DONT_CHECK		(2 << 6)
#define		VLAN_GLOBAL_CTL4_FWD_TO_MII		(3 << 6)
#define		VLAN_GLOBAL_CTL4_VID_MASK		(3 << 6)
#define	VLAN_GLOBAL_CTL5		(S1 | PAGE(0x34) | 0x05)
#define	VLAN_GLOBAL_CTL5_531xx		(S1 | PAGE(0x34) | 0x06)
#define	VLAN_GLOBAL_CTL5_63xx		(S1 | PAGE(0x34) | 0x07)
#define 	VLAN_GLOBAL_CTL5_DROP_VTAB_MISS		(1 << 3)
#define		VLAN_GLOBAL_CTL5_VID_4095_ENABLE	(1 << 2)
#define 	VLAN_GLOBAL_CTL5_CRC_TX			(1 << 0)
#define	VLAN_TABLE_ACCESS		(S2 | PAGE(0x34) | 0x06)
#define		VLAN_TABLE_ACCESS_RW_ENABLE	0x2000
#define		VLAN_TABLE_ACCESS_WRITE		0x1000
#define		VLAN_TABLE_ACCESS_VID_MASK	0x0fff

#define	VLAN_ID_MAX 				255
#define VLAN_ID_MAX5350 			15
#define VLAN_ID_MAX5395				4094

#define	VLAN_TABLE_ACCESS_5350	(S1 | PAGE(0x05) | 0x60)
#define	VLAN_TABLE_INDX_5350	(S2 | PAGE(0x05) | 0x61)
#define	VLAN_TABLE_ENTRY_5350	(S4 | PAGE(0x05) | 0x63)
/* BCM5395/5397/5398/53115/53118 */
#define	VLAN_TABLE_ACCESS_5395	(S1 | PAGE(0x05) | 0x80)
#define		VLAN_TABLE_ACCESS_5395_RUN	0x80
#define		VLAN_TABLE_ACCESS_5395_READ	0x01
#define	VLAN_TABLE_INDX_5395	(S1 | PAGE(0x05) | 0x81)
#define	VLAN_TABLE_ENTRY_5395	(S4 | PAGE(0x05) | 0x83)

/* XXX: Valid for BCM535x */
#define	VLAN_WRITE			(S4 | PAGE(0x34) | 0x08)
#define		VLAN_RW_VALID_5350		0x00100000
#define		VLAN_RW_VALID_5380 		0x04000000
#define		VLAN_RW_VALID 			0x00100000
/* XXX: need more testing / boards */
#define		VLAN_RW_VALID_5358		0x01000000
#define		VLAN_RW_UNTAG_SHIFT_5395	9
#define		VLAN_RW_UNTAG_SHIFT_5380	13
#define		VLAN_RW_UNTAG_SHIFT_5350	6
#define		VLAN_RW_UNTAG_SHIFT		7

#define		VLAN_RW_MEMBER(_p)		(1 << (_p))
#define		VLAN_RW_MEMBER_MASK		0x003f
#define		VLAN_RW_MEMBER_SHIFT		0
#define		VLAN_RW_MEMBER_5395_SHIFT	0
#define		VLAN_RW_MEMBER_5395_MASK	0x1ff
#define		VLAN_RW_UNTAGGED(_p)		(1 << ((_p) + 6))
#define		VLAN_RW_UNTAGGED_MASK		0x0fc0
#define		VLAN_RW_UNTAGGED_SHIFT		6
#define		VLAN_RW_UNTAGGED_5395_SHIFT	9
#define		VLAN_RW_UNTAGGED_5395_MASK	0x3fe00

#define	VLAN_READ			(S4 | PAGE(0x34) | 0x0C)
#define		VLAN_READ_HIGHVID_MASK		0xff000
#define		VLAN_READ_HIGHVID_SHIFT		12

/* XXX: END Valid for BCM535x */
#define	VLAN_DEFAULT_PORT_TAG(_p)	(S2 | PAGE(0x34) | (0x10 + ((_p) * 2)))
#define	VLAN_PRIORITY_REMAP		(S3 | PAGE(0x34) | 0x20)
#define		VLAN_PRIORITY_REMAP_OLD_NEW(_o, _n)	((_n) << ((_o) * 3))

#define	PORT_SUPPRESSION_CTL		(S1 | PAGE(0x35) | (0x00 + (_p)))
#define		PORT_SUPPRESSION_CTL_HAS_DROPS	0x80
#define		PORT_SUPPRESSION_CTL_PASS_MCAST	0x40
#define		PORT_SUPPRESSION_CTL_PASS_BCAST	0x20
#define		PORT_SUPPRESSION_CTL_PASS_DLF	0x10
#define		PORT_SUPPRESSION_CTL_MCAST_BURST_2k	0x00
#define		PORT_SUPPRESSION_CTL_MCAST_BURST_4k	0x04
#define		PORT_SUPPRESSION_CTL_MCAST_BURST_6k	0x08
#define		PORT_SUPPRESSION_CTL_MCAST_BURST_8k	0x0c
#define		PORT_SUPPRESSION_CTL_MCAST_RATE_10ps	0x00
#define		PORT_SUPPRESSION_CTL_MCAST_RATE_20ps	0x01
#define		PORT_SUPPRESSION_CTL_MCAST_RATE_30ps	0x02
#define		PORT_SUPPRESSION_CTL_MCAST_RATE_40ps	0x03

#define	ROBOSW_DEF_NPORTS	6
#define	ROBOSW_DEF_NVLANS	16
#endif /* _ROBOSWSWITCHREG_H_ */
