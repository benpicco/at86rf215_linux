// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Microchip (Atmel) AT86RF215 dual-band IEEE 802.15.4
 * transceiver.
 *
 * The transceiver contains a sub-GHz and a 2.4 GHz radio, each with its own
 * baseband core, that share the SPI interface and the interrupt line. Each
 * radio is registered as a separate IEEE 802.15.4 PHY. Only the legacy
 * O-QPSK PHY (250 kb/s) is supported:
 *  - 2.4 GHz radio: channel page 0, channels 11-26 (2000 kchip/s)
 *  - sub-GHz radio: channel page 2, channels 1-10 (915 MHz band, 1000 kchip/s)
 *
 * Copyright (C) 2026 ML!PA Consulting GmbH
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/ieee802154.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <net/cfg802154.h>
#include <net/mac802154.h>

#include "at86rf215.h"

#define AT86RF215_RADIO_SUBGHZ		0
#define AT86RF215_RADIO_2_4GHZ		1
#define AT86RF215_NUM_RADIOS		2

/* IEEE 802.15.4 O-QPSK PHY symbol duration (2450 MHz and 915 MHz band) */
#define AT86RF215_SYMBOL_TIME_US	16
/* macAckWaitDuration: aUnitBackoffPeriod + aTurnaroundTime + phySHRDuration + 6 * 2 */
#define AT86RF215_ACK_WAIT_SYMBOLS	54
#define AT86RF215_ACK_TIMEOUT_US	(AT86RF215_ACK_WAIT_SYMBOLS * AT86RF215_SYMBOL_TIME_US)
/* Upper bound for the transmission of an automatic ACK frame after RXFE */
#define AT86RF215_AACK_TIMEOUT_US	2000
/* aUnitBackoffPeriod: 20 symbols */
#define AT86RF215_UNIT_BACKOFF_US	(20 * AT86RF215_SYMBOL_TIME_US)
/*
 * Upper bound for CSMA-CA, retransmissions and ACK reception of a frame,
 * the worst case with the maximum CSMA-CA parameters is about 3.5 s.
 */
#define AT86RF215_TX_TIMEOUT_MS		5000

/* t_RST: minimum reset pulse width */
#define AT86RF215_RESET_PULSE_US	16
#define AT86RF215_WAKEUP_TIMEOUT_US	10000
#define AT86RF215_STATE_TIMEOUT_US	1000

/* Rough mapping of PAC.TXPWR to output power with PAC.PACUR = 3 (TXPWR=31: 14 dBm) */
#define AT86RF215_TXPWR_OFFSET_DBM	17
#define AT86RF215_TXPWR_NUM		32
#define AT86RF215_PACUR_MAX		3

/* Supported CCA energy detection thresholds */
#define AT86RF215_CCA_ED_MIN_DBM	-100
#define AT86RF215_CCA_ED_MAX_DBM	-60
#define AT86RF215_CCA_ED_NUM		(AT86RF215_CCA_ED_MAX_DBM - AT86RF215_CCA_ED_MIN_DBM + 1)
/* 10 dB above the receiver sensitivity of the O-QPSK PHY (-85 dBm) */
#define AT86RF215_CCA_ED_DEF_DBM	-75

/* Channel configuration of the 2450 MHz band O-QPSK PHY (page 0, channels 11-26) */
#define AT86RF215_RF24_CCF0_OFFSET_KHZ	1500000
#define AT86RF215_2_4GHZ_SPACING_KHZ	5000
#define AT86RF215_2_4GHZ_CENTER0_KHZ	2350000
#define AT86RF215_2_4GHZ_PAGE		0
#define AT86RF215_2_4GHZ_CHANNELS	GENMASK(26, 11)
#define AT86RF215_2_4GHZ_CHANNEL_DEF	11

/* Channel configuration of the 915 MHz band O-QPSK PHY (page 2, channels 1-10) */
#define AT86RF215_SUBGHZ_SPACING_KHZ	2000
#define AT86RF215_SUBGHZ_CENTER0_KHZ	904000
#define AT86RF215_SUBGHZ_PAGE		2
#define AT86RF215_SUBGHZ_CHANNELS	GENMASK(10, 1)
#define AT86RF215_SUBGHZ_CHANNEL_DEF	1

#define AT86RF215_FREQ_RES_KHZ		25

/* Frame filter: accept beacon, data and MAC command frames */
#define AT86RF215_AFFTM_DEFAULT		(BIT(IEEE802154_FC_TYPE_BEACON) | \
					 BIT(IEEE802154_FC_TYPE_DATA) | \
					 BIT(IEEE802154_FC_TYPE_MAC_CMD))
#define AT86RF215_AFFTM_ACK		BIT(IEEE802154_FC_TYPE_ACK)

/* Default values of macMaxCSMABackoffs, macMinBE, macMaxBE and macMaxFrameRetries */
#define AT86RF215_CSMA_MAX_BO		4
#define AT86RF215_CSMA_MIN_BE		3
#define AT86RF215_CSMA_MAX_BE		5
#define AT86RF215_FRAME_RETRIES		3

/* Bits of at86rf215_radio.events, set by the timers */
#define AT86RF215_EV_TIMEOUT		0
#define AT86RF215_EV_BACKOFF		1
#define AT86RF215_EV_WATCHDOG		2

/* Upper bound of interrupt status reads per interrupt thread run */
#define AT86RF215_IRQ_LOOPS		8

/* Software state of a radio */
enum at86rf215_trx_state {
	/* Receiver on, waiting for frames */
	AT86RF215_TRX_IDLE,
	/* CCA with automatic transmission (CCATX) in progress */
	AT86RF215_TRX_CCATX,
	/* Frame with ACK request received, waiting for AACK to finish */
	AT86RF215_TRX_RX_SEND_ACK,
	/* Frame transmission in progress */
	AT86RF215_TRX_TX,
	/* Frame transmitted, waiting for the ACK frame */
	AT86RF215_TRX_TX_WAIT_ACK,
};

struct at86rf215_chip;

struct at86rf215_radio {
	struct ieee802154_hw *hw;
	struct at86rf215_chip *chip;

	/* ACK timeout and AACK watchdog */
	struct hrtimer timer;
	/* CSMA-CA backoff period */
	struct hrtimer backoff_timer;
	/* Upper bound for the transmission of a frame */
	struct hrtimer watchdog;
	/* AT86RF215_EV_* bits */
	unsigned long events;

	/* Frame handed over by .xmit_async, picked up by the IRQ thread */
	struct sk_buff *tx_next;
	/* Frame being transmitted */
	struct sk_buff *tx_skb;

	s32 tx_powers[AT86RF215_TXPWR_NUM];
	s32 cca_ed_levels[AT86RF215_CCA_ED_NUM];

	/* All following members are protected by chip->lock */
	u16 rf_base;
	u16 bbc_base;
	u16 fb_rx;
	u16 fb_tx;
	enum at86rf215_trx_state state;
	u8 idx;
	u8 pc;
	u8 amcs;
	u8 tx_seq;
	u8 retries;
	u8 max_frame_retries;
	/* CSMA-CA number of backoffs (NB) and backoff exponent (BE) */
	u8 nb;
	u8 be;
	u8 min_be;
	u8 max_be;
	u8 max_bo;
	bool started:1;
	bool tx_pending:1;
	bool ack_requested:1;
	bool agc_hold:1;
};

struct at86rf215_chip {
	struct spi_device *spi;
	struct at86rf215_radio *radio[AT86RF215_NUM_RADIOS];
	/* Protects SPI access and the radio state machines */
	struct mutex lock;
	u8 pn;
	u8 vn;
	/* SPI transfer buffer: 16 bit address followed by the data */
	u8 buf[2 + IEEE802154_MTU] ____cacheline_aligned;
};

/* SPI access, called with the chip lock held */

static int at86rf215_access(struct at86rf215_chip *chip, bool write, u16 addr,
			    void *data, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = chip->buf,
		.rx_buf = chip->buf,
		.len = 2 + len,
	};
	int ret;

	lockdep_assert_held(&chip->lock);

	if (WARN_ON(len > sizeof(chip->buf) - 2))
		return -EINVAL;

	put_unaligned_be16((addr & AT86RF215_SPI_ADDR_MASK) |
			   (write ? AT86RF215_SPI_WRITE : 0), chip->buf);
	if (write)
		memcpy(chip->buf + 2, data, len);

	ret = spi_sync_transfer(chip->spi, &xfer, 1);
	if (ret) {
		dev_err_ratelimited(&chip->spi->dev, "SPI %s of 0x%04x failed: %d\n",
				    write ? "write" : "read", addr, ret);
		return ret;
	}

	if (!write)
		memcpy(data, chip->buf + 2, len);

	return 0;
}

static int at86rf215_read(struct at86rf215_chip *chip, u16 addr, void *data,
			  size_t len)
{
	return at86rf215_access(chip, false, addr, data, len);
}

static int at86rf215_write(struct at86rf215_chip *chip, u16 addr,
			   const void *data, size_t len)
{
	return at86rf215_access(chip, true, addr, (void *)data, len);
}

static u8 at86rf215_reg_read(struct at86rf215_chip *chip, u16 addr)
{
	u8 val = 0;

	at86rf215_read(chip, addr, &val, 1);

	return val;
}

static int at86rf215_reg_write(struct at86rf215_chip *chip, u16 addr, u8 val)
{
	return at86rf215_write(chip, addr, &val, 1);
}

static u8 rf_read(struct at86rf215_radio *r, u8 reg)
{
	return at86rf215_reg_read(r->chip, r->rf_base + reg);
}

static int rf_write(struct at86rf215_radio *r, u8 reg, u8 val)
{
	return at86rf215_reg_write(r->chip, r->rf_base + reg, val);
}

static u8 bbc_read(struct at86rf215_radio *r, u8 reg)
{
	return at86rf215_reg_read(r->chip, r->bbc_base + reg);
}

static int bbc_write(struct at86rf215_radio *r, u8 reg, u8 val)
{
	return at86rf215_reg_write(r->chip, r->bbc_base + reg, val);
}

static void rf_cmd(struct at86rf215_radio *r, u8 cmd)
{
	rf_write(r, AT86RF215_RF_CMD, cmd);
}

static u8 rf_state(struct at86rf215_radio *r)
{
	return rf_read(r, AT86RF215_RF_STATE) & AT86RF215_STATE_MASK;
}

static int rf_await_state(struct at86rf215_radio *r, u8 state)
{
	u8 val;

	return read_poll_timeout(rf_state, val, val == state, 10,
				 AT86RF215_STATE_TIMEOUT_US, false, r);
}

/* Wait until a transmission (e.g. of an automatic ACK) has finished */
static void rf_await_tx_end(struct at86rf215_radio *r)
{
	u8 val;

	read_poll_timeout(rf_state, val,
			  val != AT86RF215_STATE_TX && val != AT86RF215_STATE_TRANSITION,
			  10, AT86RF215_STATE_TIMEOUT_US, false, r);
}

/* Timers, run in hard interrupt context: defer the work to the IRQ thread */

static void at86rf215_wake(struct at86rf215_radio *r, unsigned int event)
{
	set_bit(event, &r->events);
	irq_wake_thread(r->chip->spi->irq, r->chip);
}

static enum hrtimer_restart at86rf215_timer_fn(struct hrtimer *t)
{
	struct at86rf215_radio *r = container_of(t, struct at86rf215_radio, timer);

	at86rf215_wake(r, AT86RF215_EV_TIMEOUT);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart at86rf215_backoff_fn(struct hrtimer *t)
{
	struct at86rf215_radio *r = container_of(t, struct at86rf215_radio,
						 backoff_timer);

	at86rf215_wake(r, AT86RF215_EV_BACKOFF);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart at86rf215_watchdog_fn(struct hrtimer *t)
{
	struct at86rf215_radio *r = container_of(t, struct at86rf215_radio,
						 watchdog);

	at86rf215_wake(r, AT86RF215_EV_WATCHDOG);

	return HRTIMER_NORESTART;
}

static void at86rf215_timer_cancel(struct at86rf215_radio *r, struct hrtimer *t,
				   unsigned int event)
{
	hrtimer_cancel(t);
	clear_bit(event, &r->events);
}

static void at86rf215_timer_arm(struct at86rf215_radio *r, struct hrtimer *t,
				unsigned int event, unsigned int usec)
{
	at86rf215_timer_cancel(r, t, event);
	hrtimer_start(t, us_to_ktime(usec), HRTIMER_MODE_REL);
}

/* Radio state machine, all functions are called with the chip lock held */

static void at86rf215_start_timer(struct at86rf215_radio *r, unsigned int usec)
{
	at86rf215_timer_arm(r, &r->timer, AT86RF215_EV_TIMEOUT, usec);
}

static void at86rf215_stop_timer(struct at86rf215_radio *r)
{
	at86rf215_timer_cancel(r, &r->timer, AT86RF215_EV_TIMEOUT);
}

/* Restore the receive configuration after a transmission attempt */
static void at86rf215_restore_rx(struct at86rf215_radio *r)
{
	bbc_write(r, AT86RF215_BBC_AMCS, r->amcs);

	if (r->ack_requested)
		bbc_write(r, AT86RF215_BBC_AFFTM, AT86RF215_AFFTM_DEFAULT);
}

static void at86rf215_tx_end(struct at86rf215_radio *r, int result)
{
	struct sk_buff *skb = r->tx_skb;

	at86rf215_restore_rx(r);
	r->ack_requested = false;
	r->tx_pending = false;
	r->state = AT86RF215_TRX_IDLE;
	at86rf215_timer_cancel(r, &r->watchdog, AT86RF215_EV_WATCHDOG);

	r->tx_skb = NULL;
	if (WARN_ON(!skb))
		return;

	switch (result) {
	case 0:
		ieee802154_xmit_complete(r->hw, skb, true);
		break;
	case -EBUSY:
		ieee802154_xmit_error(r->hw, skb, IEEE802154_CHANNEL_ACCESS_FAILURE);
		break;
	case -ENOMSG:
		ieee802154_xmit_error(r->hw, skb, IEEE802154_NO_ACK);
		break;
	default:
		ieee802154_xmit_hw_error(r->hw, skb);
		break;
	}
}

static void at86rf215_set_idle(struct at86rf215_radio *r)
{
	r->state = AT86RF215_TRX_IDLE;
	rf_cmd(r, AT86RF215_CMD_RX);
}

static void at86rf215_csma_cca(struct at86rf215_radio *r);

static void at86rf215_csma_backoff(struct at86rf215_radio *r)
{
	u32 periods = get_random_u32_below(BIT(r->be));

	if (!periods) {
		at86rf215_csma_cca(r);
		return;
	}

	at86rf215_timer_arm(r, &r->backoff_timer, AT86RF215_EV_BACKOFF,
			    periods * AT86RF215_UNIT_BACKOFF_US);
}

/* Unslotted CSMA-CA, see IEEE 802.15.4-2020 section 6.2.5.1 */
static void at86rf215_csma_start(struct at86rf215_radio *r)
{
	r->nb = 0;
	r->be = r->min_be;
	at86rf215_csma_backoff(r);
}

static void at86rf215_csma_busy(struct at86rf215_radio *r)
{
	if (++r->nb > r->max_bo) {
		at86rf215_tx_end(r, -EBUSY);
		return;
	}

	r->be = min(r->be + 1, r->max_be);
	at86rf215_csma_backoff(r);
}

/* Backoff period expired: perform CCA, the frame is sent automatically if the channel is clear */
static void at86rf215_csma_cca(struct at86rf215_radio *r)
{
	/* an ongoing reception or ACK transmission occupies the channel */
	if (r->state != AT86RF215_TRX_IDLE || r->agc_hold ||
	    rf_await_state(r, AT86RF215_STATE_RX)) {
		at86rf215_csma_busy(r);
		return;
	}

	r->state = AT86RF215_TRX_CCATX;

	/* AACK must not be active during the transmission, CCATX excludes TX2RX */
	bbc_write(r, AT86RF215_BBC_AMCS,
		  (r->amcs & ~AT86RF215_AMCS_AACK) | AT86RF215_AMCS_CCATX);
	/* do not receive frames during the energy measurement */
	bbc_write(r, AT86RF215_BBC_PC, r->pc & ~AT86RF215_PC_BBEN);
	rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_SINGLE);
}

/* Cancel all pending operations and complete a pending frame with an error */
static void at86rf215_abort(struct at86rf215_radio *r, int result)
{
	at86rf215_stop_timer(r);
	at86rf215_timer_cancel(r, &r->backoff_timer, AT86RF215_EV_BACKOFF);

	if (r->state == AT86RF215_TRX_CCATX) {
		rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);
		bbc_write(r, AT86RF215_BBC_PC, r->pc);
	}

	if (r->tx_skb)
		at86rf215_tx_end(r, result);

	r->state = AT86RF215_TRX_IDLE;
}

static void at86rf215_tx_start(struct at86rf215_radio *r, struct sk_buff *skb)
{
	u8 len[2];
	int ret;

	r->tx_skb = skb;
	at86rf215_timer_arm(r, &r->watchdog, AT86RF215_EV_WATCHDOG,
			    AT86RF215_TX_TIMEOUT_MS * USEC_PER_MSEC);

	/* the FCS is appended by the transceiver */
	put_unaligned_le16(skb->len + IEEE802154_FCS_LEN, len);
	ret = at86rf215_write(r->chip, r->fb_tx, skb->data, skb->len);
	if (!ret)
		ret = at86rf215_write(r->chip, r->bbc_base + AT86RF215_BBC_TXFLL,
				      len, sizeof(len));
	if (ret) {
		at86rf215_tx_end(r, ret);
		return;
	}

	r->ack_requested = get_unaligned_le16(skb->data) & IEEE802154_FCTL_ACKREQ;
	r->tx_seq = skb->data[2];
	r->retries = r->max_frame_retries;
	r->tx_pending = true;

	at86rf215_csma_start(r);
}

static bool at86rf215_ack_received(struct at86rf215_radio *r)
{
	u8 len[2];
	u8 ack[IEEE802154_ACK_PSDU_LEN - IEEE802154_FCS_LEN];

	if (at86rf215_read(r->chip, r->bbc_base + AT86RF215_BBC_RXFLL, len, sizeof(len)))
		return false;

	if ((get_unaligned_le16(len) & AT86RF215_FL_MASK) != IEEE802154_ACK_PSDU_LEN)
		return false;

	if (at86rf215_read(r->chip, r->fb_rx, ack, sizeof(ack)))
		return false;

	return IEEE802154_FC_TYPE(ack[0]) == IEEE802154_FC_TYPE_ACK &&
	       ack[2] == r->tx_seq;
}

/* Determine if the AACK procedure will acknowledge the received frame */
static bool at86rf215_aack_pending(struct at86rf215_radio *r, const u8 *psdu,
				   size_t len)
{
	u16 fc = get_unaligned_le16(psdu);
	u8 type = IEEE802154_FC_TYPE(fc);

	if (!(r->amcs & AT86RF215_AMCS_AACK) || !(fc & IEEE802154_FCTL_ACKREQ))
		return false;

	if (type != IEEE802154_FC_TYPE_DATA && type != IEEE802154_FC_TYPE_MAC_CMD)
		return false;

	/* broadcast frames are not acknowledged */
	if ((fc & IEEE802154_FCTL_DADDR) == IEEE802154_FCTL_DADDR_SHORT && len >= 7 &&
	    get_unaligned_le16(&psdu[5]) == IEEE802154_ADDR_SHORT_BROADCAST)
		return false;

	return true;
}

/* Pass a received frame to the stack, returns true if it will be acknowledged */
static bool at86rf215_rx(struct at86rf215_radio *r)
{
	struct sk_buff *skb;
	u8 buf[7];
	u16 len;
	s8 rssi;
	bool aack;

	if (at86rf215_read(r->chip, r->bbc_base + AT86RF215_BBC_RXFLL, buf, 2))
		return false;

	len = get_unaligned_le16(buf) & AT86RF215_FL_MASK;
	if (len < IEEE802154_ACK_PSDU_LEN || len > IEEE802154_MTU) {
		dev_dbg(&r->chip->spi->dev, "invalid frame length %u\n", len);
		return false;
	}

	skb = alloc_skb(IEEE802154_MTU, GFP_KERNEL);
	if (!skb) {
		/* the AACK state still has to be tracked */
		if (at86rf215_read(r->chip, r->fb_rx, buf, min_t(size_t, len, sizeof(buf))))
			return false;
		return at86rf215_aack_pending(r, buf, len);
	}

	/* the FCS has been checked by the transceiver and is passed on */
	if (at86rf215_read(r->chip, r->fb_rx, skb_put(skb, len), len)) {
		kfree_skb(skb);
		return false;
	}

	aack = at86rf215_aack_pending(r, skb->data, len);

	/* energy detection is performed automatically during frame reception */
	rssi = (s8)rf_read(r, AT86RF215_RF_EDV);

	/* the transceiver does not provide a link quality, derive it from the RSSI */
	ieee802154_rx_irqsafe(r->hw, skb, clamp((rssi + 100) * 4, 0, 255));

	return aack;
}

static void at86rf215_isr(struct at86rf215_radio *r, u8 rf_irq, u8 bb_irq)
{
	unsigned long events = xchg(&r->events, 0);
	bool timeout = events & BIT(AT86RF215_EV_TIMEOUT);
	struct sk_buff *skb;

	if (!r->started)
		return;

	if (bb_irq & AT86RF215_BB_IRQ_AGCH)
		r->agc_hold = true;

	if (bb_irq & AT86RF215_BB_IRQ_AGCR)
		r->agc_hold = false;

	if ((rf_irq & AT86RF215_RF_IRQ_EDC) && r->state == AT86RF215_TRX_CCATX) {
		if (bbc_read(r, AT86RF215_BBC_AMCS) & AT86RF215_AMCS_CCAED) {
			/* channel busy: TXFE is issued as well, radio remains in RX */
			bb_irq &= ~AT86RF215_BB_IRQ_TXFE;
			rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);
			bbc_write(r, AT86RF215_BBC_AMCS, r->amcs);
			bbc_write(r, AT86RF215_BBC_PC, r->pc);
			r->state = AT86RF215_TRX_IDLE;
			at86rf215_csma_busy(r);
		} else {
			/* channel clear: transceiver transmits the frame */
			r->tx_pending = false;
			r->state = AT86RF215_TRX_TX;
		}
	}

	/* frame transmitted - must be handled before a following ACK frame */
	if ((bb_irq & AT86RF215_BB_IRQ_TXFE) && r->state == AT86RF215_TRX_TX) {
		bb_irq &= ~AT86RF215_BB_IRQ_TXFE;

		/* CCATX ends in TXPREP, switch to RX quickly to receive the ACK */
		rf_cmd(r, AT86RF215_CMD_RX);
		rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);
		bbc_write(r, AT86RF215_BBC_AMCS, r->amcs & ~AT86RF215_AMCS_AACK);
		bbc_write(r, AT86RF215_BBC_PC, r->pc);

		if (r->ack_requested) {
			/* only receive ACK frames while waiting for the ACK */
			bbc_write(r, AT86RF215_BBC_AFFTM, AT86RF215_AFFTM_ACK);
			r->state = AT86RF215_TRX_TX_WAIT_ACK;
			at86rf215_start_timer(r, AT86RF215_ACK_TIMEOUT_US);
		} else {
			at86rf215_tx_end(r, 0);
		}
	}

	if (bb_irq & AT86RF215_BB_IRQ_RXFE) {
		switch (r->state) {
		case AT86RF215_TRX_IDLE:
			if (at86rf215_rx(r)) {
				r->state = AT86RF215_TRX_RX_SEND_ACK;
				at86rf215_start_timer(r, AT86RF215_AACK_TIMEOUT_US);
			} else {
				at86rf215_set_idle(r);
			}
			break;
		case AT86RF215_TRX_TX_WAIT_ACK:
			if (at86rf215_ack_received(r)) {
				at86rf215_stop_timer(r);
				timeout = false;
				at86rf215_tx_end(r, 0);
			}
			rf_cmd(r, AT86RF215_CMD_RX);
			break;
		default:
			break;
		}
	}

	/* automatic ACK transmitted */
	if ((bb_irq & AT86RF215_BB_IRQ_TXFE) && r->state == AT86RF215_TRX_RX_SEND_ACK) {
		at86rf215_stop_timer(r);
		timeout = false;
		at86rf215_set_idle(r);
	}

	if (timeout) {
		switch (r->state) {
		case AT86RF215_TRX_RX_SEND_ACK:
			at86rf215_set_idle(r);
			break;
		case AT86RF215_TRX_TX_WAIT_ACK:
			if (r->agc_hold) {
				/* a frame (likely the ACK) is still being received */
				at86rf215_start_timer(r, AT86RF215_ACK_TIMEOUT_US);
			} else if (r->retries > 0) {
				r->retries--;
				r->tx_pending = true;
				/* receive any frame during the backoff periods */
				at86rf215_restore_rx(r);
				r->state = AT86RF215_TRX_IDLE;
				at86rf215_csma_start(r);
			} else {
				at86rf215_tx_end(r, -ENOMSG);
			}
			break;
		default:
			break;
		}
	}

	if ((events & BIT(AT86RF215_EV_BACKOFF)) && r->tx_pending)
		at86rf215_csma_cca(r);

	if ((events & BIT(AT86RF215_EV_WATCHDOG)) && r->tx_skb) {
		dev_warn_ratelimited(&r->chip->spi->dev, "TX timeout\n");
		at86rf215_abort(r, -ETIMEDOUT);
		rf_cmd(r, AT86RF215_CMD_RX);
	}

	skb = xchg(&r->tx_next, NULL);
	if (skb)
		at86rf215_tx_start(r, skb);
}

static irqreturn_t at86rf215_irq_thread(int irq, void *data)
{
	struct at86rf215_chip *chip = data;
	unsigned int loops = 0;
	bool pending;
	u8 irqs[4];
	int i;

	mutex_lock(&chip->lock);

	do {
		/* RF09_IRQS, RF24_IRQS, BBC0_IRQS, BBC1_IRQS - cleared on read */
		if (at86rf215_read(chip, AT86RF215_REG_RF09_IRQS, irqs, sizeof(irqs)))
			break;

		pending = irqs[0] | irqs[1] | irqs[2] | irqs[3];

		for (i = 0; i < AT86RF215_NUM_RADIOS; i++)
			if (chip->radio[i])
				at86rf215_isr(chip->radio[i], irqs[i], irqs[2 + i]);
	} while (pending && ++loops < AT86RF215_IRQ_LOOPS);

	mutex_unlock(&chip->lock);

	return IRQ_HANDLED;
}

/* Radio configuration, called with the chip lock held */

static int at86rf215_write_channel(struct at86rf215_radio *r, u8 channel)
{
	u32 spacing, center0;
	u8 regs[5];

	if (r->idx == AT86RF215_RADIO_2_4GHZ) {
		spacing = AT86RF215_2_4GHZ_SPACING_KHZ;
		center0 = AT86RF215_2_4GHZ_CENTER0_KHZ - AT86RF215_RF24_CCF0_OFFSET_KHZ;
	} else {
		spacing = AT86RF215_SUBGHZ_SPACING_KHZ;
		center0 = AT86RF215_SUBGHZ_CENTER0_KHZ;
	}

	/* CS, CCF0L, CCF0H, CNL, CNM - writing CNM applies the new channel */
	regs[0] = spacing / AT86RF215_FREQ_RES_KHZ;
	put_unaligned_le16(center0 / AT86RF215_FREQ_RES_KHZ, &regs[1]);
	regs[3] = channel;
	regs[4] = 0;

	return at86rf215_write(r->chip, r->rf_base + AT86RF215_RF_CS, regs, sizeof(regs));
}

static void at86rf215_configure_legacy_oqpsk(struct at86rf215_radio *r)
{
	bool is_2_4ghz = r->idx == AT86RF215_RADIO_2_4GHZ;

	/* baseband must be disabled while it is reconfigured */
	bbc_write(r, AT86RF215_BBC_PC, 0);

	/* transmitter frontend, see datasheet table 6-103 */
	rf_write(r, AT86RF215_RF_TXCUTC,
		 FIELD_PREP(AT86RF215_TXCUTC_PARAMP_MASK, 0) |
		 FIELD_PREP(AT86RF215_TXCUTC_LPFCUT_MASK, 0xB));
	rf_write(r, AT86RF215_RF_TXDFE,
		 FIELD_PREP(AT86RF215_TXDFE_RCUT_MASK, is_2_4ghz ? 4 : 3) |
		 FIELD_PREP(AT86RF215_TXDFE_SR_MASK, 1));

	/* receiver frontend, see datasheet tables 6-105 and 6-106 */
	rf_write(r, AT86RF215_RF_RXBWC,
		 FIELD_PREP(AT86RF215_RXBWC_BW_MASK, is_2_4ghz ? 0xB : 0x8));
	rf_write(r, AT86RF215_RF_RXDFE,
		 FIELD_PREP(AT86RF215_RXDFE_RCUT_MASK, is_2_4ghz ? 2 : 0) |
		 FIELD_PREP(AT86RF215_RXDFE_SR_MASK, 1));
	rf_write(r, AT86RF215_RF_AGCC,
		 AT86RF215_AGCC_EN | FIELD_PREP(AT86RF215_AGCC_AVGS_MASK, 0));
	rf_write(r, AT86RF215_RF_AGCS, FIELD_PREP(AT86RF215_AGCS_TGT_MASK, 3));

	/* energy detection over 8 symbols (128 us), the CCA duration of the O-QPSK PHY */
	rf_write(r, AT86RF215_RF_EDD,
		 FIELD_PREP(AT86RF215_EDD_DF_MASK, 4) |
		 FIELD_PREP(AT86RF215_EDD_DTB_MASK, AT86RF215_EDD_DTB_32US));
	rf_write(r, AT86RF215_RF_EDC, AT86RF215_EDC_EDM_AUTO);

	/* legacy O-QPSK, 250 kb/s */
	bbc_write(r, AT86RF215_BBC_OQPSKC0,
		  is_2_4ghz ? AT86RF215_OQPSKC0_FCHIP_2000 : AT86RF215_OQPSKC0_FCHIP_1000);
	bbc_write(r, AT86RF215_BBC_OQPSKC2,
		  FIELD_PREP(AT86RF215_OQPSKC2_RXM_MASK, AT86RF215_OQPSKC2_RXM_LEGACY) |
		  AT86RF215_OQPSKC2_FCSTLEG);
	bbc_write(r, AT86RF215_BBC_OQPSKC3, 0);
	bbc_write(r, AT86RF215_BBC_OQPSKPHRTX, AT86RF215_OQPSKPHR_LEG);

	/* enable baseband with 16 bit FCS, automatic FCS generation and filter */
	r->pc = FIELD_PREP(AT86RF215_PC_PT_MASK, AT86RF215_PC_PT_MROQPSK) |
		AT86RF215_PC_BBEN | AT86RF215_PC_FCST | AT86RF215_PC_TXAFCS |
		AT86RF215_PC_FCSFE;
	bbc_write(r, AT86RF215_BBC_PC, r->pc);
}

static int at86rf215_radio_setup(struct at86rf215_radio *r)
{
	int ret;

	rf_cmd(r, AT86RF215_CMD_TRXOFF);
	ret = rf_await_state(r, AT86RF215_STATE_TRXOFF);
	if (ret)
		return ret;

	rf_write(r, AT86RF215_RF_IRQM, AT86RF215_RF_IRQ_EDC);
	bbc_write(r, AT86RF215_BBC_IRQM,
		  AT86RF215_BB_IRQ_RXFE | AT86RF215_BB_IRQ_TXFE |
		  AT86RF215_BB_IRQ_AGCH | AT86RF215_BB_IRQ_AGCR);

	at86rf215_configure_legacy_oqpsk(r);

	/* maximum output power */
	rf_write(r, AT86RF215_RF_PAC,
		 FIELD_PREP(AT86RF215_PAC_PACUR_MASK, AT86RF215_PACUR_MAX) |
		 AT86RF215_PAC_TXPWR_MASK);

	/* address filter, automatic ACK with FCS type and data rate of the received frame */
	bbc_write(r, AT86RF215_BBC_AFC0, AT86RF215_AFC0_AFEN0);
	bbc_write(r, AT86RF215_BBC_AFC1, 0);
	bbc_write(r, AT86RF215_BBC_AFFTM, AT86RF215_AFFTM_DEFAULT);
	bbc_write(r, AT86RF215_BBC_AMAACKPD, 0);
	bbc_write(r, AT86RF215_BBC_AMEDT, (u8)AT86RF215_CCA_ED_DEF_DBM);
	r->amcs = AT86RF215_AMCS_AACK | AT86RF215_AMCS_AACKFA | AT86RF215_AMCS_AACKDR;
	bbc_write(r, AT86RF215_BBC_AMCS, r->amcs);

	return at86rf215_write_channel(r, r->hw->phy->current_channel);
}

/* mac802154 operations */

static int at86rf215_op_start(struct ieee802154_hw *hw)
{
	struct at86rf215_radio *r = hw->priv;

	mutex_lock(&r->chip->lock);

	r->state = AT86RF215_TRX_IDLE;
	r->agc_hold = false;
	r->started = true;
	rf_cmd(r, AT86RF215_CMD_RX);

	mutex_unlock(&r->chip->lock);

	return 0;
}

static void at86rf215_op_stop(struct ieee802154_hw *hw)
{
	struct at86rf215_radio *r = hw->priv;
	struct sk_buff *skb;

	mutex_lock(&r->chip->lock);

	r->started = false;
	at86rf215_abort(r, -ESHUTDOWN);

	skb = xchg(&r->tx_next, NULL);
	if (skb)
		ieee802154_xmit_hw_error(hw, skb);

	rf_cmd(r, AT86RF215_CMD_TRXOFF);

	mutex_unlock(&r->chip->lock);
}

/* Called in atomic context: the frame is written to the transceiver by the IRQ thread */
static int at86rf215_op_xmit_async(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct at86rf215_radio *r = hw->priv;

	/* frame control field and sequence number are evaluated */
	if (skb->len < IEEE802154_FC_LEN + 1 ||
	    skb->len + IEEE802154_FCS_LEN > IEEE802154_MTU)
		return -EINVAL;

	if (WARN_ON_ONCE(cmpxchg(&r->tx_next, NULL, skb)))
		return -EBUSY;

	irq_wake_thread(r->chip->spi->irq, r->chip);

	return 0;
}

static int at86rf215_op_ed(struct ieee802154_hw *hw, u8 *level)
{
	return -EOPNOTSUPP;
}

static int at86rf215_op_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct at86rf215_radio *r = hw->priv;
	int ret;

	mutex_lock(&r->chip->lock);

	/* mac802154 does not stop the queue: a frame in progress is not sent */
	at86rf215_abort(r, -EAGAIN);
	rf_await_tx_end(r);

	/* the channel can only be changed in TRXOFF or TXPREP */
	if (rf_state(r) == AT86RF215_STATE_RX) {
		rf_cmd(r, AT86RF215_CMD_TXPREP);
		rf_await_state(r, AT86RF215_STATE_TXPREP);
	}

	ret = at86rf215_write_channel(r, channel);

	if (r->started)
		at86rf215_set_idle(r);

	mutex_unlock(&r->chip->lock);

	return ret;
}

static int at86rf215_op_set_hw_addr_filt(struct ieee802154_hw *hw,
					 struct ieee802154_hw_addr_filt *filt,
					 unsigned long changed)
{
	struct at86rf215_radio *r = hw->priv;
	u16 base = r->bbc_base;
	int ret = 0;
	u8 afc1;

	mutex_lock(&r->chip->lock);

	/* the registers hold the addresses in little endian byte order */
	if (changed & IEEE802154_AFILT_SADDR_CHANGED)
		ret = at86rf215_write(r->chip, base + AT86RF215_BBC_MACSHA0F0,
				      &filt->short_addr, sizeof(filt->short_addr));

	if (!ret && (changed & IEEE802154_AFILT_PANID_CHANGED))
		ret = at86rf215_write(r->chip, base + AT86RF215_BBC_MACPID0F0,
				      &filt->pan_id, sizeof(filt->pan_id));

	if (!ret && (changed & IEEE802154_AFILT_IEEEADDR_CHANGED))
		ret = at86rf215_write(r->chip, base + AT86RF215_BBC_MACEA0,
				      &filt->ieee_addr, sizeof(filt->ieee_addr));

	if (!ret && (changed & IEEE802154_AFILT_PANC_CHANGED)) {
		afc1 = bbc_read(r, AT86RF215_BBC_AFC1);
		if (filt->pan_coord)
			afc1 |= AT86RF215_AFC1_PANC0;
		else
			afc1 &= ~AT86RF215_AFC1_PANC0;
		ret = bbc_write(r, AT86RF215_BBC_AFC1, afc1);
	}

	mutex_unlock(&r->chip->lock);

	return ret;
}

static int at86rf215_op_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct at86rf215_radio *r = hw->priv;
	int txpwr = clamp(mbm / 100 + AT86RF215_TXPWR_OFFSET_DBM, 0,
			  AT86RF215_TXPWR_NUM - 1);
	int ret;

	mutex_lock(&r->chip->lock);
	ret = rf_write(r, AT86RF215_RF_PAC,
		       FIELD_PREP(AT86RF215_PAC_PACUR_MASK, AT86RF215_PACUR_MAX) |
		       FIELD_PREP(AT86RF215_PAC_TXPWR_MASK, txpwr));
	mutex_unlock(&r->chip->lock);

	return ret;
}

static int at86rf215_op_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	struct at86rf215_radio *r = hw->priv;
	int ret;

	/* AMEDT holds the threshold in dBm as signed value */
	mutex_lock(&r->chip->lock);
	ret = bbc_write(r, AT86RF215_BBC_AMEDT, (u8)(s8)(mbm / 100));
	mutex_unlock(&r->chip->lock);

	return ret;
}

static int at86rf215_op_set_csma_params(struct ieee802154_hw *hw, u8 min_be,
					u8 max_be, u8 retries)
{
	struct at86rf215_radio *r = hw->priv;

	mutex_lock(&r->chip->lock);
	r->min_be = min_be;
	r->max_be = max_be;
	r->max_bo = retries;
	mutex_unlock(&r->chip->lock);

	return 0;
}

static int at86rf215_op_set_frame_retries(struct ieee802154_hw *hw, s8 retries)
{
	struct at86rf215_radio *r = hw->priv;

	if (retries < 0)
		return -EINVAL;

	mutex_lock(&r->chip->lock);
	r->max_frame_retries = retries;
	mutex_unlock(&r->chip->lock);

	return 0;
}

static int at86rf215_op_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	struct at86rf215_radio *r = hw->priv;
	int ret;
	u8 afc0;

	mutex_lock(&r->chip->lock);

	afc0 = bbc_read(r, AT86RF215_BBC_AFC0);
	if (on)
		afc0 |= AT86RF215_AFC0_PM;
	else
		afc0 &= ~AT86RF215_AFC0_PM;
	ret = bbc_write(r, AT86RF215_BBC_AFC0, afc0);

	/* do not acknowledge frames in promiscuous mode */
	if (on)
		r->amcs &= ~AT86RF215_AMCS_AACK;
	else
		r->amcs |= AT86RF215_AMCS_AACK;
	if (!ret)
		ret = bbc_write(r, AT86RF215_BBC_AMCS, r->amcs);

	mutex_unlock(&r->chip->lock);

	return ret;
}

static const struct ieee802154_ops at86rf215_ops = {
	.owner = THIS_MODULE,
	.start = at86rf215_op_start,
	.stop = at86rf215_op_stop,
	.xmit_async = at86rf215_op_xmit_async,
	.ed = at86rf215_op_ed,
	.set_channel = at86rf215_op_set_channel,
	.set_hw_addr_filt = at86rf215_op_set_hw_addr_filt,
	.set_txpower = at86rf215_op_set_txpower,
	.set_cca_ed_level = at86rf215_op_set_cca_ed_level,
	.set_csma_params = at86rf215_op_set_csma_params,
	.set_frame_retries = at86rf215_op_set_frame_retries,
	.set_promiscuous_mode = at86rf215_op_set_promiscuous_mode,
};

/* Device setup */

static int at86rf215_chip_init(struct at86rf215_chip *chip, struct gpio_desc *reset)
{
	struct device *dev = &chip->spi->dev;
	unsigned long irqflags;
	u8 irqs[4], cfg, trim;
	int i, ret;

	if (reset) {
		fsleep(AT86RF215_RESET_PULSE_US);
		gpiod_set_value_cansleep(reset, 0);
	} else {
		at86rf215_reg_write(chip, AT86RF215_REG_RF_RST, AT86RF215_RST_CMD_RESET);
	}

	/* registers read 0xff until the transceiver has woken up */
	ret = read_poll_timeout(at86rf215_reg_read, chip->pn,
				chip->pn != 0xff && chip->pn != 0x00, 100,
				AT86RF215_WAKEUP_TIMEOUT_US, false, chip,
				AT86RF215_REG_RF_PN);
	if (ret)
		return dev_err_probe(dev, -ENODEV, "transceiver not responding\n");

	chip->vn = at86rf215_reg_read(chip, AT86RF215_REG_RF_VN);

	switch (chip->pn) {
	case AT86RF215_PN_AT86RF215:
		dev_info(dev, "AT86RF215 rev. %u\n", chip->vn);
		break;
	case AT86RF215_PN_AT86RF215M:
		dev_info(dev, "AT86RF215M rev. %u\n", chip->vn);
		break;
	case AT86RF215_PN_AT86RF215IQ:
		return dev_err_probe(dev, -ENODEV, "AT86RF215IQ has no baseband core\n");
	default:
		return dev_err_probe(dev, -ENODEV, "unknown part number 0x%02x\n",
				     chip->pn);
	}

	/* disable clock output */
	at86rf215_reg_write(chip, AT86RF215_REG_RF_CLKO, 0);

	if (!device_property_read_u8(dev, "xtal-trim", &trim)) {
		if (trim > FIELD_MAX(AT86RF215_XOC_TRIM_MASK))
			return dev_err_probe(dev, -EINVAL, "invalid xtal-trim %u\n", trim);

		at86rf215_reg_write(chip, AT86RF215_REG_RF_XOC,
				    FIELD_PREP(AT86RF215_XOC_TRIM_MASK, trim) |
				    AT86RF215_XOC_FS);
	}

	/* the IRQ pin is active high after reset */
	irqflags = irq_get_trigger_type(chip->spi->irq);
	cfg = at86rf215_reg_read(chip, AT86RF215_REG_RF_CFG);
	if (irqflags & (IRQF_TRIGGER_LOW | IRQF_TRIGGER_FALLING))
		cfg |= AT86RF215_CFG_IRQP;
	else
		cfg &= ~AT86RF215_CFG_IRQP;
	at86rf215_reg_write(chip, AT86RF215_REG_RF_CFG, cfg);

	/* disable the interrupts of both radios */
	for (i = 0; i < AT86RF215_NUM_RADIOS; i++) {
		if (i == AT86RF215_RADIO_2_4GHZ && chip->pn == AT86RF215_PN_AT86RF215M)
			continue;

		at86rf215_reg_write(chip, (i == AT86RF215_RADIO_2_4GHZ ?
					   AT86RF215_RF24_BASE : AT86RF215_RF09_BASE) +
				    AT86RF215_RF_IRQM, 0);
		at86rf215_reg_write(chip, (i == AT86RF215_RADIO_2_4GHZ ?
					   AT86RF215_BBC1_BASE : AT86RF215_BBC0_BASE) +
				    AT86RF215_BBC_IRQM, 0);
	}

	/* clear pending interrupts */
	return at86rf215_read(chip, AT86RF215_REG_RF09_IRQS, irqs, sizeof(irqs));
}

static int at86rf215_add_radio(struct at86rf215_chip *chip, unsigned int idx)
{
	bool is_2_4ghz = idx == AT86RF215_RADIO_2_4GHZ;
	struct ieee802154_hw *hw;
	struct at86rf215_radio *r;
	struct wpan_phy *phy;
	int i, ret;

	hw = ieee802154_alloc_hw(sizeof(*r), &at86rf215_ops);
	if (!hw)
		return -ENOMEM;

	r = hw->priv;
	r->hw = hw;
	r->chip = chip;
	r->idx = idx;
	r->rf_base = is_2_4ghz ? AT86RF215_RF24_BASE : AT86RF215_RF09_BASE;
	r->bbc_base = is_2_4ghz ? AT86RF215_BBC1_BASE : AT86RF215_BBC0_BASE;
	r->fb_rx = is_2_4ghz ? AT86RF215_BBC1_FBRXS : AT86RF215_BBC0_FBRXS;
	r->fb_tx = is_2_4ghz ? AT86RF215_BBC1_FBTXS : AT86RF215_BBC0_FBTXS;
	r->min_be = AT86RF215_CSMA_MIN_BE;
	r->max_be = AT86RF215_CSMA_MAX_BE;
	r->max_bo = AT86RF215_CSMA_MAX_BO;
	r->max_frame_retries = AT86RF215_FRAME_RETRIES;

	hrtimer_setup(&r->timer, at86rf215_timer_fn, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_setup(&r->backoff_timer, at86rf215_backoff_fn, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);
	hrtimer_setup(&r->watchdog, at86rf215_watchdog_fn, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);

	hw->parent = &chip->spi->dev;
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_CSMA_PARAMS |
		    IEEE802154_HW_FRAME_RETRIES | IEEE802154_HW_AFILT |
		    IEEE802154_HW_PROMISCUOUS;

	phy = hw->phy;
	phy->flags = WPAN_PHY_FLAG_TXPOWER | WPAN_PHY_FLAG_CCA_ED_LEVEL;

	if (is_2_4ghz) {
		phy->supported.channels[AT86RF215_2_4GHZ_PAGE] = AT86RF215_2_4GHZ_CHANNELS;
		phy->current_page = AT86RF215_2_4GHZ_PAGE;
		phy->current_channel = AT86RF215_2_4GHZ_CHANNEL_DEF;
	} else {
		phy->supported.channels[AT86RF215_SUBGHZ_PAGE] = AT86RF215_SUBGHZ_CHANNELS;
		phy->current_page = AT86RF215_SUBGHZ_PAGE;
		phy->current_channel = AT86RF215_SUBGHZ_CHANNEL_DEF;
	}

	for (i = 0; i < AT86RF215_TXPWR_NUM; i++)
		r->tx_powers[i] = (AT86RF215_TXPWR_NUM - 1 - i -
				   AT86RF215_TXPWR_OFFSET_DBM) * 100;
	phy->supported.tx_powers = r->tx_powers;
	phy->supported.tx_powers_size = ARRAY_SIZE(r->tx_powers);
	phy->transmit_power = r->tx_powers[0];

	for (i = 0; i < AT86RF215_CCA_ED_NUM; i++)
		r->cca_ed_levels[i] = (AT86RF215_CCA_ED_MIN_DBM + i) * 100;
	phy->supported.cca_ed_levels = r->cca_ed_levels;
	phy->supported.cca_ed_levels_size = ARRAY_SIZE(r->cca_ed_levels);
	phy->cca_ed_level = AT86RF215_CCA_ED_DEF_DBM * 100;

	phy->supported.cca_modes = BIT(NL802154_CCA_ENERGY);
	phy->cca.mode = NL802154_CCA_ENERGY;

	ieee802154_random_extended_addr(&phy->perm_extended_addr);

	mutex_lock(&chip->lock);
	ret = at86rf215_radio_setup(r);
	mutex_unlock(&chip->lock);
	if (ret) {
		ieee802154_free_hw(hw);
		return dev_err_probe(&chip->spi->dev, ret,
				     "failed to set up radio %u\n", idx);
	}

	chip->radio[idx] = r;

	return 0;
}

static void at86rf215_free_radios(struct at86rf215_chip *chip)
{
	int i;

	for (i = 0; i < AT86RF215_NUM_RADIOS; i++) {
		if (chip->radio[i]) {
			ieee802154_free_hw(chip->radio[i]->hw);
			chip->radio[i] = NULL;
		}
	}
}

static int at86rf215_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct at86rf215_chip *chip;
	struct gpio_desc *reset;
	unsigned long irqflags;
	int i, ret;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no IRQ specified\n");

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->spi = spi;
	spi_set_drvdata(spi, chip);

	ret = devm_mutex_init(dev, &chip->lock);
	if (ret)
		return ret;

	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "failed to get reset GPIO\n");

	mutex_lock(&chip->lock);
	ret = at86rf215_chip_init(chip, reset);
	mutex_unlock(&chip->lock);
	if (ret)
		return ret;

	ret = at86rf215_add_radio(chip, AT86RF215_RADIO_SUBGHZ);
	if (ret)
		return ret;

	if (chip->pn != AT86RF215_PN_AT86RF215M) {
		ret = at86rf215_add_radio(chip, AT86RF215_RADIO_2_4GHZ);
		if (ret)
			goto err_free;
	}

	irqflags = irq_get_trigger_type(spi->irq);
	if (!irqflags)
		irqflags = IRQF_TRIGGER_HIGH;

	ret = request_threaded_irq(spi->irq, NULL, at86rf215_irq_thread,
				   irqflags | IRQF_ONESHOT, dev_name(dev), chip);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request IRQ\n");
		goto err_free;
	}

	for (i = 0; i < AT86RF215_NUM_RADIOS; i++) {
		if (!chip->radio[i])
			continue;

		ret = ieee802154_register_hw(chip->radio[i]->hw);
		if (ret) {
			dev_err_probe(dev, ret, "failed to register radio %d\n", i);
			goto err_unregister;
		}
	}

	return 0;

err_unregister:
	while (--i >= 0)
		if (chip->radio[i])
			ieee802154_unregister_hw(chip->radio[i]->hw);
	free_irq(spi->irq, chip);
err_free:
	at86rf215_free_radios(chip);
	return ret;
}

static void at86rf215_remove(struct spi_device *spi)
{
	struct at86rf215_chip *chip = spi_get_drvdata(spi);
	int i;

	/* stopping the interfaces cancels all timers */
	for (i = AT86RF215_NUM_RADIOS - 1; i >= 0; i--)
		if (chip->radio[i])
			ieee802154_unregister_hw(chip->radio[i]->hw);

	free_irq(spi->irq, chip);
	at86rf215_free_radios(chip);
}

static const struct of_device_id at86rf215_of_match[] = {
	{ .compatible = "atmel,at86rf215" },
	{ .compatible = "atmel,at86rf215m" },
	{ }
};
MODULE_DEVICE_TABLE(of, at86rf215_of_match);

static const struct spi_device_id at86rf215_device_id[] = {
	{ .name = "at86rf215" },
	{ .name = "at86rf215m" },
	{ }
};
MODULE_DEVICE_TABLE(spi, at86rf215_device_id);

static struct spi_driver at86rf215_driver = {
	.id_table = at86rf215_device_id,
	.driver = {
		.of_match_table = at86rf215_of_match,
		.name = "at86rf215",
	},
	.probe = at86rf215_probe,
	.remove = at86rf215_remove,
};
module_spi_driver(at86rf215_driver);

MODULE_AUTHOR("Benjamin Valentin <benjamin.valentin@ml-pa.com>");
MODULE_DESCRIPTION("AT86RF215 IEEE 802.15.4 transceiver driver");
MODULE_LICENSE("GPL");
