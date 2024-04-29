/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2024 Hiroki Mori
 * Copyright (c) 2013 Adrian Chadd <adrian@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <machine/bus.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_bus.h>

#include <mips/broadcom/bcm338x/bcm3383reg.h>

#include "uart_if.h"

/*
 * Default system clock is 25MHz; see bcm338x_chip.c for how
 * the startup process determines whether it's 25MHz or 40MHz.
 */
#define	DEFAULT_RCLK	(25 * 1000 * 1000)

#define	bcm338x_getreg(bas, reg)           \
	bus_space_read_4((bas)->bst, (bas)->bsh, reg)
#define	bcm338x_setreg(bas, reg, value)    \
	bus_space_write_4((bas)->bst, (bas)->bsh, reg, value)

static int
bcm338x_drain(struct uart_bas *bas, int what)
{
	int limit;

#if 0
	if (what & UART_DRAIN_TRANSMITTER) {
		limit = 10*1024;

		/* Loop over until the TX FIFO shows entirely clear */
		while (--limit) {
			if ((bcm338x_getreg(bas, AR933X_UART_CS_REG)
			    & AR933X_UART_CS_TX_BUSY) == 0)
				break;
		}
		if (limit == 0) {
			return (EIO);
		}
	}

	if (what & UART_DRAIN_RECEIVER) {
		limit=10*4096;
		while (--limit) {

			/* XXX duplicated from bcm338x_getc() */
			/* XXX TODO: refactor! */

			/* If there's nothing to read, stop! */
			if ((bcm338x_getreg(bas, AR933X_UART_DATA_REG) &
			    AR933X_UART_DATA_RX_CSR) == 0) {
				break;
			}

			/* Read the top of the RX FIFO */
			(void) bcm338x_getreg(bas, AR933X_UART_DATA_REG);

			/* Remove that entry from said RX FIFO */
			bcm338x_setreg(bas, AR933X_UART_DATA_REG,
			    AR933X_UART_DATA_RX_CSR);

			uart_barrier(bas);
			DELAY(2);
		}
		if (limit == 0) {
			return (EIO);
		}
	}
#endif
	return (0);
}

/*
 * Calculate the baud from the given chip configuration parameters.
 */
static unsigned long
bcm338x_uart_get_baud(unsigned int clk, unsigned int scale,
    unsigned int step)
{
	uint64_t t;
	uint32_t div;

	div = (2 << 16) * (scale + 1);
	t = clk;
	t *= step;
	t += (div / 2);
	t = t / div;

	return (t);
}

/*
 * Calculate the scale/step with the lowest possible deviation from
 * the target baudrate.
 */
static void
bcm338x_uart_get_scale_step(struct uart_bas *bas, unsigned int baud,
    unsigned int *scale, unsigned int *step)
{
	unsigned int tscale;
	uint32_t clk;
	long min_diff;
#if 0

	clk = bas->rclk;
	*scale = 0;
	*step = 0;

	min_diff = baud;
	for (tscale = 0; tscale < AR933X_UART_MAX_SCALE; tscale++) {
		uint64_t tstep;
		int diff;

		tstep = baud * (tscale + 1);
		tstep *= (2 << 16);
		tstep = tstep / clk;

		if (tstep > AR933X_UART_MAX_STEP)
			break;

		diff = abs(bcm338x_uart_get_baud(clk, tscale, tstep) - baud);
		if (diff < min_diff) {
			min_diff = diff;
			*scale = tscale;
			*step = tstep;
		}
	}
#endif
}

static int
bcm338x_param(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{
#if 0
	/* UART always 8 bits */

	/* UART always 1 stop bit */

	/* UART parity is controllable by bits 0:1, ignore for now */

	/* Set baudrate if required. */
	if (baudrate > 0) {
		uint32_t clock_scale, clock_step;

		/* Find the best fit for the given baud rate */
		bcm338x_uart_get_scale_step(bas, baudrate, &clock_scale,
		    &clock_step);

		/*
		 * Program the clock register in its entirety - no need
		 * for Read-Modify-Write.
		 */
		bcm338x_setreg(bas, AR933X_UART_CLOCK_REG,
		    ((clock_scale & AR933X_UART_CLOCK_SCALE_M)
		      << AR933X_UART_CLOCK_SCALE_S) |
		    (clock_step & AR933X_UART_CLOCK_STEP_M));
	}

	uart_barrier(bas);
#endif
	return (0);
}


/*
 * Low-level UART interface.
 */
static int bcm338x_probe(struct uart_bas *bas);
static void bcm338x_init(struct uart_bas *bas, int, int, int, int);
static void bcm338x_term(struct uart_bas *bas);
static void bcm338x_putc(struct uart_bas *bas, int);
static int bcm338x_rxready(struct uart_bas *bas);
static int bcm338x_getc(struct uart_bas *bas, struct mtx *);

static struct uart_ops uart_bcm338x_ops = {
	.probe = bcm338x_probe,
	.init = bcm338x_init,
	.term = bcm338x_term,
	.putc = bcm338x_putc,
	.rxready = bcm338x_rxready,
	.getc = bcm338x_getc,
};

static int
bcm338x_probe(struct uart_bas *bas)
{

	/* We always know this will be here */
	return (0);
}

static void
bcm338x_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{
#if 0
	uint32_t reg;

	/* Setup default parameters */
	bcm338x_param(bas, baudrate, databits, stopbits, parity);

	/* XXX Force enable UART in case it was disabled */

	/* Disable all interrupts */
	bcm338x_setreg(bas, AR933X_UART_INT_EN_REG, 0x00000000);

	/* Disable the host interrupt */
	reg = bcm338x_getreg(bas, AR933X_UART_CS_REG);
	reg &= ~AR933X_UART_CS_HOST_INT_EN;
	bcm338x_setreg(bas, AR933X_UART_CS_REG, reg);

	uart_barrier(bas);

	/* XXX Set RTS/DTR? */
#endif
}

/*
 * Detach from console.
 */
static void
bcm338x_term(struct uart_bas *bas)
{

	/* XXX TODO */
}

static void
bcm338x_putc(struct uart_bas *bas, int c)
{
	unsigned int tx_level;

	while(1)
	{
		tx_level = bcm338x_getreg(bas, 0x08);
		tx_level = tx_level >> 24;
		tx_level &= 0x1f;
		if(tx_level < 14)
		break;
	}

	bcm338x_setreg(bas, 0x14, (c & 0xff));
}

static int
bcm338x_rxready(struct uart_bas *bas)
{
	unsigned int rx_level;

	rx_level = bcm338x_getreg(bas, 0x08);
	rx_level = rx_level >> 16;
	rx_level &= 0x1f;

	return rx_level ? 1 : 0;
}

static int
bcm338x_getc(struct uart_bas *bas, struct mtx *hwmtx)
{
	int c;

	uart_lock(hwmtx);

	c = bcm338x_getreg(bas, 0x14) & 0xff;

	uart_unlock(hwmtx);

	return (c);
}

/*
 * High-level UART interface.
 */
struct bcm338x_softc {
	struct uart_softc base;

	uint32_t	u_ier;
};

static int bcm338x_bus_attach(struct uart_softc *);
static int bcm338x_bus_detach(struct uart_softc *);
static int bcm338x_bus_flush(struct uart_softc *, int);
static int bcm338x_bus_getsig(struct uart_softc *);
static int bcm338x_bus_ioctl(struct uart_softc *, int, intptr_t);
static int bcm338x_bus_ipend(struct uart_softc *);
static int bcm338x_bus_param(struct uart_softc *, int, int, int, int);
static int bcm338x_bus_probe(struct uart_softc *);
static int bcm338x_bus_receive(struct uart_softc *);
static int bcm338x_bus_setsig(struct uart_softc *, int);
static int bcm338x_bus_transmit(struct uart_softc *);
static void bcm338x_bus_grab(struct uart_softc *);
static void bcm338x_bus_ungrab(struct uart_softc *);

static kobj_method_t bcm338x_methods[] = {
	KOBJMETHOD(uart_attach,		bcm338x_bus_attach),
	KOBJMETHOD(uart_detach,		bcm338x_bus_detach),
	KOBJMETHOD(uart_flush,		bcm338x_bus_flush),
//	KOBJMETHOD(uart_getsig,		bcm338x_bus_getsig),
	KOBJMETHOD(uart_ioctl,		bcm338x_bus_ioctl),
	KOBJMETHOD(uart_ipend,		bcm338x_bus_ipend),
	KOBJMETHOD(uart_param,		bcm338x_bus_param),
	KOBJMETHOD(uart_probe,		bcm338x_bus_probe),
	KOBJMETHOD(uart_receive,	bcm338x_bus_receive),
//	KOBJMETHOD(uart_setsig,		bcm338x_bus_setsig),
	KOBJMETHOD(uart_transmit,	bcm338x_bus_transmit),
//	KOBJMETHOD(uart_grab,		bcm338x_bus_grab),
//	KOBJMETHOD(uart_ungrab,		bcm338x_bus_ungrab),
	{ 0, 0 }
};

struct uart_class uart_bcm338x_class = {
	"bcm338x",
	bcm338x_methods,
	sizeof(struct bcm338x_softc),
	.uc_ops = &uart_bcm338x_ops,
	.uc_range = 8,
	.uc_rclk = DEFAULT_RCLK,
	.uc_rshift = 0
};

#define	SIGCHG(c, i, s, d)				\
	if (c) {					\
		i |= (i & s) ? s : s | d;		\
	} else {					\
		i = (i & s) ? (i & ~s) | d : i;		\
	}

static int
bcm338x_bus_attach(struct uart_softc *sc)
{
	struct bcm338x_softc *u = (struct bcm338x_softc *)sc;
	struct uart_bas *bas = &sc->sc_bas;
	uint32_t reg;

	/* XXX TODO: flush transmitter */
#if 0

	/*
	 * Setup initial interrupt notifications.
	 *
	 * XXX for now, just RX FIFO valid.
	 * Later on (when they're handled), also handle
	 * RX errors/overflow.
	 */
	u->u_ier = AR933X_UART_INT_RX_VALID;

	/* Enable RX interrupts to kick-start things */
	bcm338x_setreg(bas, AR933X_UART_INT_EN_REG, u->u_ier);

	/* Enable the host interrupt now */
	reg = bcm338x_getreg(bas, AR933X_UART_CS_REG);
	reg |= AR933X_UART_CS_HOST_INT_EN;
	bcm338x_setreg(bas, AR933X_UART_CS_REG, reg);
#endif

	return (0);
}

static int
bcm338x_bus_detach(struct uart_softc *sc)
{
	struct uart_bas *bas = &sc->sc_bas;
	uint32_t reg;
#if 0

	/* Disable all interrupts */
	bcm338x_setreg(bas, AR933X_UART_INT_EN_REG, 0x00000000);

	/* Disable the host interrupt */
	reg = bcm338x_getreg(bas, AR933X_UART_CS_REG);
	reg &= ~AR933X_UART_CS_HOST_INT_EN;
	bcm338x_setreg(bas, AR933X_UART_CS_REG, reg);
	uart_barrier(bas);
#endif

	return (0);
}

static int
bcm338x_bus_flush(struct uart_softc *sc, int what)
{
	struct uart_bas *bas;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
//	bcm338x_drain(bas, what);
	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
bcm338x_bus_getsig(struct uart_softc *sc)
{
	uint32_t sig = sc->sc_hwsig;

	/*
	 * For now, let's just return that DSR/DCD/CTS is asserted.
	 */
	SIGCHG(1, sig, SER_DSR, SER_DDSR);
	SIGCHG(1, sig, SER_CTS, SER_DCTS);
	SIGCHG(1, sig, SER_DCD, SER_DDCD);
	SIGCHG(1, sig,  SER_RI,  SER_DRI);

	sc->sc_hwsig = sig & ~SER_MASK_DELTA;

	return (sig);
}

/*
 * XXX TODO: actually implement the rest of this!
 */
static int
bcm338x_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{
	int error = 0;

	/* XXX lock */
	switch (request) {
	case UART_IOCTL_BREAK:
	case UART_IOCTL_IFLOW:
	case UART_IOCTL_OFLOW:
		break;
	case UART_IOCTL_BAUD:
		*(int*)data = 115200;
		break;
	default:
		error = EINVAL;
		break;
	}

	/* XXX unlock */

	return (error);
}

/*
 * Bus interrupt handler.
 *
 * For now, system interrupts are disabled.
 * So this is just called from a callout in uart_core.c
 * to poll various state.
 */
static int
bcm338x_bus_ipend(struct uart_softc *sc)
{
	struct bcm338x_softc *u = (struct bcm338x_softc *)sc;
	struct uart_bas *bas = &sc->sc_bas;
	int ipend = 0;
	uint32_t isr;

#if 0
	uart_lock(sc->sc_hwmtx);

	/*
	 * Fetch/ACK the ISR status.
	 */
	isr = bcm338x_getreg(bas, AR933X_UART_INT_REG);
	bcm338x_setreg(bas, AR933X_UART_INT_REG, isr);
	uart_barrier(bas);

	/*
	 * RX ready - notify upper layer.
	 */
	if (isr & AR933X_UART_INT_RX_VALID) {
		ipend |= SER_INT_RXREADY;
	}

	/*
	 * If we get this interrupt, we should disable
	 * it from the interrupt mask and inform the uart
	 * driver appropriately.
	 *
	 * We can't keep setting SER_INT_TXIDLE or SER_INT_SIGCHG
	 * all the time or IO stops working.  So we will always
	 * clear this interrupt if we get it, then we only signal
	 * the upper layer if we were doing active TX in the
	 * first place.
	 *
	 * Also, the name is misleading.  This actually means
	 * "the FIFO is almost empty."  So if we just write some
	 * more data to the FIFO without checking whether it can
	 * take said data, we'll overflow the thing.
	 *
	 * Unfortunately the FreeBSD uart device has no concept of
	 * partial UART writes - it expects that the whole buffer
	 * is written to the hardware.  Thus for now, bcm338x_bus_transmit()
	 * will wait for the FIFO to finish draining before it pushes
	 * more frames into it.
	 */
	if (isr & AR933X_UART_INT_TX_EMPTY) {
		/*
		 * Update u_ier to disable TX notifications; update hardware
		 */
		u->u_ier &= ~AR933X_UART_INT_TX_EMPTY;
		bcm338x_setreg(bas, AR933X_UART_INT_EN_REG, u->u_ier);
		uart_barrier(bas);
	}

	/*
	 * Only signal TX idle if we're not busy transmitting.
	 *
	 * XXX I never get _out_ of txbusy? Debug that!
	 */
	if (sc->sc_txbusy) {
		if (isr & AR933X_UART_INT_TX_EMPTY) {
			ipend |= SER_INT_TXIDLE;
		} else {
			ipend |= SER_INT_SIGCHG;
		}
	}

	uart_unlock(sc->sc_hwmtx);
#endif
	return (ipend);
}

static int
bcm338x_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{
	struct uart_bas *bas;
	int error;
#if 0

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	error = bcm338x_param(bas, baudrate, databits, stopbits, parity);
	uart_unlock(sc->sc_hwmtx);
#endif
	error = 0;
	return (error);
}

static int
bcm338x_bus_probe(struct uart_softc *sc)
{
	struct uart_bas *bas;
	int error;

	bas = &sc->sc_bas;
#if 0

	error = bcm338x_probe(bas);
	if (error)
		return (error);

	/* Reset FIFOs. */
	bcm338x_drain(bas, UART_FLUSH_RECEIVER|UART_FLUSH_TRANSMITTER);

	/* XXX TODO: actually find out what the FIFO depth is! */
	sc->sc_rxfifosz = 16;
	sc->sc_txfifosz = 16;

#endif
	return (0);
}

static int
bcm338x_bus_receive(struct uart_softc *sc)
{
#if 0
	struct uart_bas *bas = &sc->sc_bas;
	int xc;

	uart_lock(sc->sc_hwmtx);

	/* Loop over until we are full, or no data is available */
	while (bcm338x_rxready(bas)) {
		if (uart_rx_full(sc)) {
			sc->sc_rxbuf[sc->sc_rxput] = UART_STAT_OVERRUN;
			break;
		}

		/* Read the top of the RX FIFO */
		xc = bcm338x_getreg(bas, AR933X_UART_DATA_REG) & 0xff;

		/* Remove that entry from said RX FIFO */
		bcm338x_setreg(bas, AR933X_UART_DATA_REG,
		    AR933X_UART_DATA_RX_CSR);
		uart_barrier(bas);

		/* XXX frame, parity error */
		uart_rx_put(sc, xc);
	}

	/*
	 * XXX TODO: Discard everything left in the Rx FIFO?
	 * XXX only if we've hit an overrun condition?
	 */

	uart_unlock(sc->sc_hwmtx);
#endif

	return (0);
}

static int
bcm338x_bus_setsig(struct uart_softc *sc, int sig)
{
#if 0
	struct bcm338x_softc *ns8250 = (struct bcm338x_softc*)sc;
	struct uart_bas *bas;
	uint32_t new, old;

	bas = &sc->sc_bas;
	do {
		old = sc->sc_hwsig;
		new = old;
		if (sig & SER_DDTR) {
			SIGCHG(sig & SER_DTR, new, SER_DTR,
			    SER_DDTR);
		}
		if (sig & SER_DRTS) {
			SIGCHG(sig & SER_RTS, new, SER_RTS,
			    SER_DRTS);
		}
	} while (!atomic_cmpset_32(&sc->sc_hwsig, old, new));
	uart_lock(sc->sc_hwmtx);
	ns8250->mcr &= ~(MCR_DTR|MCR_RTS);
	if (new & SER_DTR)
		ns8250->mcr |= MCR_DTR;
	if (new & SER_RTS)
		ns8250->mcr |= MCR_RTS;
	uart_setreg(bas, REG_MCR, ns8250->mcr);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
#endif
	return (0);
}

/*
 * Write the current transmit buffer to the TX FIFO.
 *
 * Unfortunately the FreeBSD uart device has no concept of
 * partial UART writes - it expects that the whole buffer
 * is written to the hardware.  Thus for now, this will wait for
 * the FIFO to finish draining before it pushes more frames into it.
 *
 * If non-blocking operation is truely needed here, either
 * the FreeBSD uart device will need to handle partial writes
 * in xxx_bus_transmit(), or we'll need to do TX FIFO buffering
 * of our own here.
 */
static int
bcm338x_bus_transmit(struct uart_softc *sc)
{
	struct uart_bas *bas = &sc->sc_bas;
	struct bcm338x_softc *u = (struct bcm338x_softc *)sc;
	int i;
#if 0

	uart_lock(sc->sc_hwmtx);

	/* Wait for the FIFO to be clear - see above */
	while (bcm338x_getreg(bas, AR933X_UART_CS_REG) &
	    AR933X_UART_CS_TX_BUSY)
		;

	/*
	 * Write some data!
	 */
	for (i = 0; i < sc->sc_txdatasz; i++) {
		/* Write the TX data */
		bcm338x_setreg(bas, AR933X_UART_DATA_REG,
		    (sc->sc_txbuf[i] & 0xff) | AR933X_UART_DATA_TX_CSR);
		uart_barrier(bas);
	}

	/*
	 * Now that we're transmitting, get interrupt notification
	 * when the FIFO is (almost) empty - see above.
	 */
	u->u_ier |= AR933X_UART_INT_TX_EMPTY;
	bcm338x_setreg(bas, AR933X_UART_INT_EN_REG, u->u_ier);
	uart_barrier(bas);

	/*
	 * Inform the upper layer that we are presently transmitting
	 * data to the hardware; this will be cleared when the
	 * TXIDLE interrupt occurs.
	 */
	sc->sc_txbusy = 1;
	uart_unlock(sc->sc_hwmtx);
#endif

	return (0);
}

static void
bcm338x_bus_grab(struct uart_softc *sc)
{
#if 0
	struct uart_bas *bas = &sc->sc_bas;
	uint32_t reg;

	/* Disable the host interrupt now */
	uart_lock(sc->sc_hwmtx);
	reg = bcm338x_getreg(bas, AR933X_UART_CS_REG);
	reg &= ~AR933X_UART_CS_HOST_INT_EN;
	bcm338x_setreg(bas, AR933X_UART_CS_REG, reg);
	uart_unlock(sc->sc_hwmtx);
#endif
}

static void
bcm338x_bus_ungrab(struct uart_softc *sc)
{
#if 0
	struct uart_bas *bas = &sc->sc_bas;
	uint32_t reg;

	/* Enable the host interrupt now */
	uart_lock(sc->sc_hwmtx);
	reg = bcm338x_getreg(bas, AR933X_UART_CS_REG);
	reg |= AR933X_UART_CS_HOST_INT_EN;
	bcm338x_setreg(bas, AR933X_UART_CS_REG, reg);
	uart_unlock(sc->sc_hwmtx);
#endif
}
