// SPDX-License-Identifier: MIT
/*
 * wiegand.c - Wiegand receiver core. See wiegand.h.
 *
 * State machine (main context only):
 *
 *   RECOVER --(lines high for >= frame_gap_us)--> IDLE
 *   IDLE    --(first valid pulse)---------------> RECEIVING
 *   RECEIVING --(silence > frame_gap_us)--------> IDLE   (frame delivered)
 *   IDLE/RECEIVING --(fatal fault)--------------> RECOVER (frame discarded)
 *
 * Pulse validity is decided on the rising edge (width known). Tpi is measured
 * rising edge -> next falling edge exactly as defined by the SIA standard, section 2.2.
 */

#include "wiegand.h"
#include <string.h>

#define QMASK	(WIEGAND_QUEUE_SIZE - 1U)

enum {
	ST_RECOVER = 0,
	ST_IDLE,
	ST_RECEIVING,
};

/* Time helpers (wrap safe) */

static inline uint32_t elapsed(uint32_t now, uint32_t then)
{
	return now - then;
}

static inline bool is_after(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b) > 0;
}

/* Reporting and recovery */

static void report(wiegand_t *w, wiegand_status_t s)
{
	wiegand_stats_t *st = &w->stats;

	switch (s) {
	case WIEGAND_STATUS_PULSE_TOO_SHORT:
		st->pulse_too_short++;
		break;
	case WIEGAND_STATUS_PULSE_TOO_LONG:
		st->pulse_too_long++;
		break;
	case WIEGAND_STATUS_GAP_TOO_SHORT:
		st->gap_too_short++;
		break;
	case WIEGAND_STATUS_OVERLAP:
		st->overlap++;
		break;
	case WIEGAND_STATUS_STUCK_LOW:
		st->stuck_low++;
		break;
	case WIEGAND_STATUS_EDGE_SEQUENCE:
		st->edge_sequence++;
		break;
	case WIEGAND_STATUS_TOO_MANY_BITS:
		st->too_many_bits++;
		break;
	case WIEGAND_STATUS_BIT_COUNT:
		st->bit_count++;
		break;
	case WIEGAND_STATUS_PARITY:
		st->parity++;
		break;
	default:
		break;
	}

	if (w->cfg.on_error)
		w->cfg.on_error(w->cfg.ctx, s);
}

static void clear_frame(wiegand_t *w)
{
	memset(&w->frame, 0, sizeof(w->frame));
}

static void enter_recovery(wiegand_t *w, uint32_t t)
{
	clear_frame(w);
	w->state = ST_RECOVER;
	w->quiet_valid = !w->line[0].low && !w->line[1].low;
	w->quiet_start_us = t;
}

static void fail(wiegand_t *w, wiegand_status_t s, uint32_t t)
{
	enter_recovery(w, t);
	report(w, s);
}

/* During recovery: restart quiet timer whenever both lines are high */
static void quiet_update(wiegand_t *w, uint32_t t)
{
	if (!w->line[0].low && !w->line[1].low) {
		w->quiet_valid = true;
		w->quiet_start_us = t;
	} else {
		w->quiet_valid = false;
	}
}

/* Re-arm if the line has been quiet long enough as of time t */
static void recover_check(wiegand_t *w, uint32_t t)
{
	if (w->state == ST_RECOVER && w->quiet_valid && !w->resync_pending &&
	    elapsed(t, w->quiet_start_us) >= w->cfg.frame_gap_us)
		w->state = ST_IDLE;
}

/* Frame handling */

static void add_bit(wiegand_t *w, uint8_t bit, uint32_t fall_us)
{
	const uint16_t i = w->frame.bit_count;

	if (bit != 0U)
		w->frame.bits[i >> 3] |= (uint8_t)(0x80U >> (i & 7U));

	if (i == 0U)
		w->frame.first_us = fall_us;

	w->frame.last_us = fall_us;
	w->frame.bit_count = (uint16_t)(i + 1U);
}

/* Precondition: state == ST_RECEIVING. Silence has been established. */
static void finish_frame(wiegand_t *w)
{
	const uint16_t n = w->frame.bit_count;
	wiegand_status_t err = WIEGAND_STATUS_OK;

	if (n < w->cfg.min_bits || n > w->cfg.max_bits)
		err = WIEGAND_STATUS_BIT_COUNT;
	else if (w->cfg.validator && !w->cfg.validator(&w->frame, w->cfg.ctx))
		err = WIEGAND_STATUS_PARITY;

	w->state = ST_IDLE;

	if (err != WIEGAND_STATUS_OK) {
		report(w, err);
	} else {
		w->stats.frames_ok++;
		if (w->cfg.on_frame)
			w->cfg.on_frame(w->cfg.ctx, &w->frame);
	}

	clear_frame(w);
}

/* Fault detection that needs no edge: line held low */
static void check_long(wiegand_t *w, uint32_t t)
{
	unsigned int i;

	for (i = 0U; i < 2U; i++) {
		uint32_t d;

		if (!w->line[i].low)
			continue;

		d = elapsed(t, w->line[i].fall_us);

		if (!w->line[i].long_flag && d > w->cfg.pulse_max_us) {
			w->line[i].long_flag = true;
			if (w->state != ST_RECOVER)
				fail(w, WIEGAND_STATUS_PULSE_TOO_LONG, t);
		}

		if (!w->line[i].stuck_flag && d > w->cfg.stuck_us) {
			w->line[i].stuck_flag = true;
			report(w, WIEGAND_STATUS_STUCK_LOW);
		}
	}
}

/* Edges were lost in the ISR queue: line state and frame are unknown */
static void check_overflow(wiegand_t *w, uint32_t t)
{
	const uint32_t d = w->dropped;

	if (d == w->dropped_seen)
		return;

	w->stats.queue_overflow += d - w->dropped_seen;
	w->dropped_seen = d;
	w->resync_pending = true;

	fail(w, WIEGAND_STATUS_QUEUE_OVERFLOW, t);
	w->quiet_valid = false;
}

/* Edge handlers */

static void handle_fall(wiegand_t *w, unsigned int line, uint32_t ts)
{
	bool dup;

	/* Silence before this edge ends the previous frame */
	if (w->state == ST_RECEIVING &&
	    elapsed(ts, w->last_rise_us) > w->cfg.frame_gap_us)
		finish_frame(w);

	dup = w->line[line].low;

	w->line[line].low = true;
	w->line[line].fall_us = ts;
	w->line[line].long_flag = false;
	w->line[line].stuck_flag = false;

	if (w->state == ST_RECOVER)
		w->quiet_valid = false;
	else if (dup)
		fail(w, WIEGAND_STATUS_EDGE_SEQUENCE, ts);
}

static void handle_rise(wiegand_t *w, unsigned int line, uint32_t ts)
{
	uint32_t fall;
	uint32_t width;

	if (!w->line[line].low) {
		if (w->state == ST_RECOVER)
			quiet_update(w, ts);
		else
			fail(w, WIEGAND_STATUS_EDGE_SEQUENCE, ts);
		return;
	}

	fall = w->line[line].fall_us;
	width = elapsed(ts, fall);

	w->line[line].low = false;
	w->line[line].long_flag = false;
	w->line[line].stuck_flag = false;

	if (w->state == ST_RECOVER) {
		quiet_update(w, ts);
		return;
	}

	if (width < w->cfg.pulse_min_us) {
		/* Glitch / crosstalk: filtered */
		if (w->state == ST_RECEIVING && w->frame.runts < 0xFFU)
			w->frame.runts++;
		report(w, WIEGAND_STATUS_PULSE_TOO_SHORT);
		return;
	}

	if (width > w->cfg.pulse_max_us) {
		fail(w, WIEGAND_STATUS_PULSE_TOO_LONG, ts);
		return;
	}

	if (w->state == ST_RECEIVING) {
		const int32_t gap = (int32_t)(fall - w->last_rise_us);

		if (gap < 0) {
			fail(w, WIEGAND_STATUS_OVERLAP, ts);
			return;
		}
		if ((uint32_t)gap < w->cfg.gap_min_us) {
			fail(w, WIEGAND_STATUS_GAP_TOO_SHORT, ts);
			return;
		}
	} else {
		clear_frame(w);
		w->state = ST_RECEIVING;
	}

	if (w->frame.bit_count >= w->cfg.max_bits) {
		fail(w, WIEGAND_STATUS_TOO_MANY_BITS, ts);
		return;
	}

	add_bit(w, (line == (unsigned int)WIEGAND_LINE_D1) ? 1U : 0U, fall);
	w->last_rise_us = ts;

	if (line == (unsigned int)WIEGAND_LINE_D1)
		w->stats.d1_bits++;
	else
		w->stats.d0_bits++;
}

static void handle_event(wiegand_t *w, const wiegand_event_t *ev)
{
	const uint32_t ts = ev->ts_us;

	check_overflow(w, ts);
	w->last_event_us = ts;
	recover_check(w, ts);
	check_long(w, ts);

	if (ev->rising != 0U)
		handle_rise(w, ev->line, ts);
	else
		handle_fall(w, ev->line, ts);
}

/* Public API */

static bool config_valid(const wiegand_config_t *c)
{
	const uint32_t lim = 0x7FFFFFFFU;

	return c->pulse_min_us >= 1U &&
	       c->pulse_max_us >= c->pulse_min_us &&
	       c->gap_min_us >= 1U &&
	       c->frame_gap_us > c->gap_min_us &&
	       c->stuck_us > c->pulse_max_us &&
	       c->frame_gap_us <= lim && c->stuck_us <= lim &&
	       c->min_bits >= 1U &&
	       c->max_bits >= c->min_bits &&
	       c->max_bits <= WIEGAND_MAX_BITS;
}

void wiegand_config_default(wiegand_config_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->pulse_min_us = 10U;
	cfg->pulse_max_us = 150U;
	cfg->gap_min_us = 100U;
	cfg->frame_gap_us = 25000U;
	cfg->stuck_us = 50000U;
	cfg->min_bits = 4U;
	cfg->max_bits = (uint16_t)WIEGAND_MAX_BITS;
}

void wiegand_config_sia(wiegand_config_t *cfg)
{
	wiegand_config_default(cfg);
	cfg->pulse_min_us = 20U;
	cfg->pulse_max_us = 100U;
	cfg->gap_min_us = 200U;
	cfg->frame_gap_us = 20000U;
}

static void sync_lines(wiegand_t *w, uint32_t now_us, uint8_t levels)
{
	const bool high[2] = {
		(levels & WIEGAND_LEVEL_D0) != 0U,
		(levels & WIEGAND_LEVEL_D1) != 0U
	};
	unsigned int i;

	for (i = 0U; i < 2U; i++) {
		w->line[i].low = !high[i];
		w->line[i].fall_us = now_us;
		w->line[i].long_flag = false;
		w->line[i].stuck_flag = false;
	}

	clear_frame(w);
	w->state = ST_RECOVER;
	w->quiet_valid = high[0] && high[1];
	w->quiet_start_us = now_us;
}

bool wiegand_init(wiegand_t *w, const wiegand_config_t *cfg,
		  uint32_t now_us, uint8_t levels)
{
	if (!w || !cfg || !config_valid(cfg))
		return false;

	memset(w, 0, sizeof(*w));
	w->cfg = *cfg;
	w->last_event_us = now_us;
	w->last_rise_us = now_us;
	sync_lines(w, now_us, levels);

	return true;
}

bool wiegand_push_edge(wiegand_t *w, wiegand_line_t line, bool rising,
		       uint32_t ts_us)
{
	const uint32_t head = w->head;
	uint32_t used = head - w->tail;
	wiegand_event_t *e;

	if (used >= WIEGAND_QUEUE_SIZE) {
		w->dropped++;
		return false;
	}

	e = &w->q[head & QMASK];
	e->ts_us = ts_us;
	e->line = (uint8_t)line;
	e->rising = rising ? 1U : 0U;

	WIEGAND_BARRIER();
	w->head = head + 1U;

	used++;
	if (used > w->peak)
		w->peak = used;

	return true;
}

void wiegand_process(wiegand_t *w, uint32_t now_us, uint8_t levels)
{
	const uint32_t sample_us = now_us;
	uint32_t tail = w->tail;

	while (tail != w->head) {
		wiegand_event_t ev;

		WIEGAND_BARRIER();
		ev = w->q[tail & QMASK];
		WIEGAND_BARRIER();

		tail++;
		w->tail = tail;

		handle_event(w, &ev);
	}

	if (is_after(w->last_event_us, now_us))
		now_us = w->last_event_us;

	check_overflow(w, now_us);

	if (w->resync_pending && !is_after(w->last_event_us, sample_us)) {
		sync_lines(w, sample_us, levels);
		w->resync_pending = false;
	}

	recover_check(w, now_us);
	check_long(w, now_us);

	if (w->state == ST_RECEIVING &&
	    !w->line[0].low && !w->line[1].low &&
	    elapsed(now_us, w->last_rise_us) > w->cfg.frame_gap_us)
		finish_frame(w);
}

void wiegand_reset(wiegand_t *w, uint32_t now_us, uint8_t levels)
{
	w->tail = w->head;
	w->dropped_seen = w->dropped;
	w->resync_pending = false;
	w->last_event_us = now_us;
	w->last_rise_us = now_us;
	sync_lines(w, now_us, levels);
}

bool wiegand_is_receiving(const wiegand_t *w)
{
	return w->state == ST_RECEIVING;
}

void wiegand_get_stats(const wiegand_t *w, wiegand_stats_t *out)
{
	*out = w->stats;
	out->queue_peak = w->peak;
}

bool wiegand_get_bit(const wiegand_frame_t *frame, uint16_t index)
{
	if (index >= frame->bit_count)
		return false;

	return (frame->bits[index >> 3] & (0x80U >> (index & 7U))) != 0U;
}

static unsigned int ones(const wiegand_frame_t *f, uint16_t first, uint16_t last)
{
	unsigned int n = 0U;
	uint16_t i;

	for (i = first; i <= last; i++) {
		if (wiegand_get_bit(f, i))
			n++;
	}
	return n;
}

bool wiegand_validate_parity26(const wiegand_frame_t *frame, void *ctx)
{
	(void)ctx;

	if (frame->bit_count != 26U)
		return false;

	return ((ones(frame, 0U, 12U) & 1U) == 0U) &&
	       ((ones(frame, 13U, 25U) & 1U) == 1U);
}
