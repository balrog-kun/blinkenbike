/*
 * 16-bit timer 1 used for Sensorino timekeeping as well as arbitrary
 * timeouts.
 *
 * Licensed under AGPLv3.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <Arduino.h>

#include "Timers.h"

Timers::Timers(void) {}
Timers::init Timers::initializer;

void Timers::begin(void) {
	TCCR1A = 0x00;
	TCCR1B = 0x01;
	TIMSK1 = 0x01;
}

static volatile uint16_t timer_cycles = 0;
static uint32_t last_sec = 0;
static uint32_t seconds = 0;

static void update_timeouts(void);

/* Always called with interrupts disabled */
static void timer_overflow(void) {
	uint32_t new_val;

	timer_cycles ++;

	new_val = (uint32_t) timer_cycles << 16;
	if (new_val >= last_sec + F_CPU || unlikely(new_val < last_sec)) {
		last_sec += F_CPU;
		seconds ++;
	}

	update_timeouts();
}

/* Timekeeping */
ISR(TIMER1_OVF_vect) {
	timer_overflow();
}

/* Read current time in cpu cycles (way more complex than it should be..) */
uint32_t Timers::now(void) {
	/*
	 * The simple, but not 100% race safe version, not even in interrupt
	 * context, is:
	 * return TCNT1 | ((uint32_t) timer_cycles << 16);
	 */
	uint16_t lo, hi, sreg;

	/*
	 * First make sure no overflow is pending.  This should only happen
	 * with interrupts disabled already..
	 * ..but for some reason it also happens when they're enabled, is
	 * that a cpu bug?
	 */
	while (unlikely((TIFR1 & 1) && !(SREG & 0x80))) {
		TIFR1 |= 1;

		timer_overflow();
	}

	sreg = SREG;
	cli();

	lo = TCNT1, hi = timer_cycles;
	/*
	 * If an overflow is now pending (must have happened after the cli)
	 * and lo is low, meaning that only a few cycles had happened since
	 * the last overflow, then we assume that hi contains the value from
	 * before the overflow, while lo contains the value from after it
	 * which is the only "interesting" scenario.  Compensate for that.
	 */
	if (unlikely((TIFR1 & 1) && lo < 0x8000))
		hi ++;

	SREG = sreg;
	return ((uint32_t) hi << 16) | lo;
}

uint32_t Timers::millis(void) {
	return now() / (F_CPU / 1000);
}

/* Burn some cycles */
void Timers::delay(uint16_t msecs) {
	uint32_t end = now() + (uint32_t) msecs * (F_CPU / 1000);
	while (now() < end);
}

/*
 * This is a super simple timeout support.  There is a limited number of
 * timeouts that can be set at any given point.  Timeouts that are far
 * enough from now and far enough from other timeouts, should be rather
 * accurate, but there are no guarantees.  The callback may happen a
 * little later than expected, but not sooner than expected.  This is
 * guaranteed, but the "little later" can be rather long in extreme
 * cases.  It will be reasonably short if callback routines are reasonably
 * short.  Timeouts being set need to be within 2^32 / F_CPU / 2 seconds
 * from now, which is about 120 seconds at 16MHz.
 *
 * Callbacks currently run with interrupts enabled in order let the timer
 * overflow do its job, but if the callbacks don't take too long, they
 * could be given the comfort of interrupts disabled which would simplify
 * the code here a little.
 */
#define MAX_TIMEOUTS	16

static struct timeout_s {
	uint32_t when;
	void *callback;
	uint8_t next, cls;
} timeouts[MAX_TIMEOUTS];
static uint8_t next = 0xff;
static uint8_t k = 0;

static void set_timeout(void *callback, uint32_t timeout, uint8_t cls) {
	uint8_t i, *j, sreg;
	uint32_t when = Timers::now() + timeout;

	sreg = SREG;
	cli();

	for (i = next, j = &next; i != 0xff; j = &timeouts[i].next, i = *j)
		if ((uint32_t) (when - timeouts[i].when) >> 31)
			break;
	for (; timeouts[k].callback; k = (k + 1) & (MAX_TIMEOUTS - 1));
	timeouts[k].when = when;
	timeouts[k].callback = callback;
	timeouts[k].next = i;
	timeouts[k].cls = cls;
	*j = k;

	if (j == &next)
		update_timeouts();

	SREG = sreg;
}

void Timers::setTimeout(void (*callback)(void), uint32_t timeout) {
	set_timeout((void *) callback, timeout, 0);
}

void Timers::setTimeout(GenCallback *callback, uint32_t timeout) {
	set_timeout(callback, timeout, 1);
}

static volatile uint8_t updating;
static volatile uint8_t updated;

ISR(TIMER1_COMPA_vect) {
	uint32_t now;

	TIMSK1 = 0x01;

#if 0
	updating = 1;
#endif

	do {
		void *callback = timeouts[next].callback;
		uint8_t cls = timeouts[next].cls;
		timeouts[next].callback = NULL;
		next = timeouts[next].next;
		updated = 0;

		if (cls) {
			GenCallback *cb = (GenCallback *) callback;
			sei();
			cb->call();
			cli();
			delete cb;
		} else {
			void (*cb)(void) = (void (*)(void)) callback;
			sei();
			cb();
			cli();
		}

		now = Timers::now();
	} while (next != 0xff && (uint32_t) (timeouts[next].when - now) >> 31);

	updating = 0;
	if (!updated)
		update_timeouts();
}

/*
 * Another assumption that we make, but which really depends on factors
 * like the compiler flags:
 *
 * setting the timer compare register to a new value will take less than
 * MIN_DELAY cycles.
 */
#define MIN_DELAY 60

/* Interrupts disabled here */
static void update_timeouts(void) {
	int16_t diff;
	uint16_t ocra, tcnt;

	if (unlikely(updating))
		return;

	TIMSK1 = 0x01;
	updated = 1;
	if (next == 0xff)
		return;
	diff = (timeouts[next].when >> 16) - timer_cycles;
	if (diff > 0)
		return;

	/*
	 * If the desired value is in the past or very close to now,
	 * we make it now + MIN_DELAY cycles to avoid any race
	 * conditions.  The hope is that this function will not take
	 * longer than MIN_DELAY cycles.
	 */
	ocra = timeouts[next].when;
	tcnt = TCNT1;
	if (unlikely(diff || unlikely(ocra < MIN_DELAY ||
					ocra - MIN_DELAY < tcnt)))
		ocra = tcnt + MIN_DELAY;
	OCR1A = ocra;
	TIFR1 |= 0x02; /* Why is this needed? CPU bug? */
	TIMSK1 = 0x03;
}
