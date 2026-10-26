/* SPDX-License-Identifier: MIT */
/*
 * wiegand.h - Wiegand receiver core (wire level only, hardware independent).
 *
 * Reference: SIA Access Control Standard "Protocol for the 26-bit Wiegand Reader Interface" (1996)
 *   4.3  Tpw  20 us .. 100 us   (low pulse width, Vol -> Voh, measured at the reader per 2.1)
 *        Tpi 200 us ..  20 ms   (interval between pulses, Voh -> Vol)
 *        D0/D1 idle high, pulses never overlap.
 *   3.2.3 D1 (white) = binary one, 3.2.4 D0 (green) = binary zero.
 *   5    26-bit: P1 even over code bits 1..12, P2 odd over code bits 13..24.
 *
 * Execution model (single producer / single consumer, lock free):
 *   ISR   : wiegand_push_edge()     one call per D0/D1 edge, timestamp in us
 *   main  : wiegand_process()       drains edges, runs state machine, callbacks
 *
 * The core never touches hardware. Time is a free-running uint32_t in us
 * (wraps every 71.6 min, all arithmetic is wrap safe for spans < 2^31 us).
 */

#ifndef WIEGAND_H
#define WIEGAND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Build configuration */

/* Frame buffer capacity in bits. Multiple of 8. */
#ifndef WIEGAND_MAX_BITS
#define WIEGAND_MAX_BITS	256U
#endif

/*
 * Edge queue depth. Power of two. 2 edges per bit, so 2 * WIEGAND_MAX_BITS
 * lets a maximum length frame be buffered with no main-loop service at all.
 */
#ifndef WIEGAND_QUEUE_SIZE
#define WIEGAND_QUEUE_SIZE	512U
#endif

/*
 * Compiler barrier between queue data and index publication.
 * Sufficient on a single core (ISR vs. thread).
 */
#ifndef WIEGAND_BARRIER
#if defined(__GNUC__)
#define WIEGAND_BARRIER()	__asm__ __volatile__("" ::: "memory")
#else
#define WIEGAND_BARRIER()	((void)0)
#endif
#endif

typedef char wiegand_cfg_queue_pow2_[
	((WIEGAND_QUEUE_SIZE & (WIEGAND_QUEUE_SIZE - 1U)) == 0U) ? 1 : -1];
typedef char wiegand_cfg_bits_mult8_[
	(((WIEGAND_MAX_BITS % 8U) == 0U) && (WIEGAND_MAX_BITS <= 0xFFFFU)) ? 1 : -1];

#define WIEGAND_LEVEL_D0	0x01U	/* bit set = D0 currently high */
#define WIEGAND_LEVEL_D1	0x02U	/* bit set = D1 currently high */

typedef enum {
	WIEGAND_LINE_D0 = 0,		/* pulse = binary 0 */
	WIEGAND_LINE_D1 = 1,		/* pulse = binary 1 */
} wiegand_line_t;

/*
 * Reported through on_error. "fatal" = current frame discarded and the
 * receiver resynchronises (waits for frame_gap_us of silence).
 */
typedef enum {
	WIEGAND_STATUS_OK = 0,
	WIEGAND_STATUS_PULSE_TOO_SHORT,	/* non-fatal: runt ignored, frame intact   */
	WIEGAND_STATUS_PULSE_TOO_LONG,	/* fatal: low longer than pulse_max_us     */
	WIEGAND_STATUS_GAP_TOO_SHORT,	/* fatal: Tpi below gap_min_us             */
	WIEGAND_STATUS_OVERLAP,		/* fatal: D0/D1 valid pulses overlapped    */
	WIEGAND_STATUS_STUCK_LOW,	/* informational: line low > stuck_us      */
	WIEGAND_STATUS_EDGE_SEQUENCE,	/* fatal: missed or duplicated edge        */
	WIEGAND_STATUS_QUEUE_OVERFLOW,	/* fatal: edges lost in the ISR queue      */
	WIEGAND_STATUS_TOO_MANY_BITS,	/* fatal: more than max_bits pulses        */
	WIEGAND_STATUS_BIT_COUNT,	/* frame dropped: length outside min..max  */
	WIEGAND_STATUS_PARITY,		/* frame dropped: validator rejected it    */
} wiegand_status_t;

typedef struct {
	uint8_t		bits[WIEGAND_MAX_BITS / 8U];	/* first received bit = MSB of [0] */
	uint16_t	bit_count;
	uint8_t		runts;				/* filtered glitches inside the frame */
	uint32_t	first_us;			/* falling edge of first bit */
	uint32_t	last_us;			/* falling edge of last bit */
} wiegand_frame_t;

typedef struct {
	uint32_t	frames_ok;
	uint32_t	d0_bits;	/* accepted bits, includes discarded frames */
	uint32_t	d1_bits;
	uint32_t	pulse_too_short;
	uint32_t	pulse_too_long;
	uint32_t	gap_too_short;
	uint32_t	overlap;
	uint32_t	stuck_low;
	uint32_t	edge_sequence;
	uint32_t	queue_overflow;	/* edges dropped */
	uint32_t	too_many_bits;
	uint32_t	bit_count;
	uint32_t	parity;
	uint32_t	queue_peak;	/* max edges queued, tune WIEGAND_QUEUE_SIZE */
} wiegand_stats_t;

/* Return false to reject the frame (reported as WIEGAND_STATUS_PARITY). */
typedef bool (*wiegand_validator_t)(const wiegand_frame_t *frame, void *ctx);
typedef void (*wiegand_frame_cb_t)(void *ctx, const wiegand_frame_t *frame);
typedef void (*wiegand_error_cb_t)(void *ctx, wiegand_status_t status);

typedef struct {
	/* Pulse width, low time. SIA: 20..100 us. */
	uint32_t		pulse_min_us;
	uint32_t		pulse_max_us;
	/* Minimum Tpi (rising edge -> next falling edge). SIA: 200 us. */
	uint32_t		gap_min_us;
	/*
	 * Silence that ends a frame (SIA Tpi max = 20 ms) and, after a fault,
	 * the silence required before the receiver re-arms.
	 */
	uint32_t		frame_gap_us;
	/* Low time after which a line is additionally reported as stuck. */
	uint32_t		stuck_us;

	uint16_t		min_bits;
	uint16_t		max_bits;

	wiegand_validator_t	validator;	/* optional */
	wiegand_frame_cb_t	on_frame;	/* called from wiegand_process() */
	wiegand_error_cb_t	on_error;	/* optional, same context */
	void			*ctx;
} wiegand_config_t;

/* Edge queue entry (ISR -> main). */
typedef struct {
	uint32_t	ts_us;
	uint8_t		line;
	uint8_t		rising;
} wiegand_event_t;

/* Instance. Treat as opaque; one per reader port. */
typedef struct {
	/* ISR owned (producer) */
	volatile uint32_t	head;
	volatile uint32_t	dropped;
	volatile uint32_t	peak;

	/* shared */
	wiegand_event_t		q[WIEGAND_QUEUE_SIZE];

	/* main owned (consumer) */
	volatile uint32_t	tail;

	wiegand_config_t	cfg;
	wiegand_stats_t		stats;
	wiegand_frame_t		frame;

	struct {
		bool		low;
		bool		long_flag;
		bool		stuck_flag;
		uint32_t	fall_us;
	} line[2];

	uint8_t			state;
	bool			quiet_valid;
	bool			resync_pending;
	uint32_t		quiet_start_us;
	uint32_t		last_rise_us;	/* rising edge of last accepted bit */
	uint32_t		last_event_us;
	uint32_t		dropped_seen;
} wiegand_t;

/* Tolerant defaults: pulse 10..150 us, Tpi >= 100 us, frame gap 25 ms */
void wiegand_config_default(wiegand_config_t *cfg);

/* Exact SIA limits: pulse 20..100 us, Tpi >= 200 us, frame gap 20 ms */
void wiegand_config_sia(wiegand_config_t *cfg);

/*
 * Initialise. Call with the edge source disabled.
 * Starts in resync: first frame accepted only after frame_gap_us of silence.
 */
bool wiegand_init(wiegand_t *w, const wiegand_config_t *cfg,
		  uint32_t now_us, uint8_t levels);

/* ISR context. Single producer. Returns false if queue full. */
bool wiegand_push_edge(wiegand_t *w, wiegand_line_t line, bool rising,
		       uint32_t ts_us);

/* Main context, drain edges and evaluate timeouts. */
void wiegand_process(wiegand_t *w, uint32_t now_us, uint8_t levels);

/* Discard queued edges and frame in progress, resync. */
void wiegand_reset(wiegand_t *w, uint32_t now_us, uint8_t levels);

/* True while frame is being collected. */
bool wiegand_is_receiving(const wiegand_t *w);

void wiegand_get_stats(const wiegand_t *w, wiegand_stats_t *out);

/* Bit accessor, index 0 = first received bit. */
bool wiegand_get_bit(const wiegand_frame_t *frame, uint16_t index);

/* SIA 26-bit validator: bits 0..12 even parity, bits 13..25 odd parity. */
bool wiegand_validate_parity26(const wiegand_frame_t *frame, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* WIEGAND_H */
