/*
 * Author: Andrew Zaborowski <andrew.zaborowski@intel.com>
 *
 * Licensed under the BSD license.
 */
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <WS2811.h>

#include <Wire.h>
#include <SPI.h>

#include <math.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

/* Sensors */
static MPU60X0 accgyro;
static uint32_t gyro_offset[3];

static void zero_gyro(void) {
  int i;
  int reading[6];

  gyro_offset[0] = 0;
  gyro_offset[1] = 0;
  gyro_offset[2] = 0;

#define CALIB_SHIFT 7
  for (i = 0; i < (1 << CALIB_SHIFT); i ++) {
    accgyro.getMotion6(&reading[0], &reading[1], &reading[2], &reading[3],
        &reading[4], &reading[5]);

    gyro_offset[0] += reading[3];
    gyro_offset[1] += reading[4];
    gyro_offset[2] += reading[5];
  }
}

/* Define the output function, using pin 2 on port C. */
#define LED_DDR  DDRC
#define LED_PORT PORTC
#define LED_PIN  2
DEFINE_WS2811_FN(WS2811RGB, LED_PORT, LED_PIN)

/* LED count and placement data */
static int led_cnt;
static uint16_t led_angle[128];
static uint8_t led_dist[128];
static uint8_t min_dist, full_dist;
struct led_strip_s {
	int x0, y0, x1, y1, count;
} strips[] = {
	/*
	 * Wheel is in a "zero" position, LED coordinates are in the wheel's
	 * plane, X being the horizontal axis (bike's forward-back), Y the
	 * vertical axis (bike's up-down), in milimeters.
	 * x0, y0 is the first LED in the strip,
	 * x1, y1 is the last LED, count is the number of LEDs including
	 * first and last.
	 */
	{
		-32, 48,
		-51, 282,
		15,
	},
	{
		32, 48,
		51, 282,
		15,
	},
	{
		58, 4,
		270, -97,
		15,
	},
	{
		25, -52,
		219, -185,
		15,
	},
	{
		-25, -52,
		-219, -185,
		15,
	},
	{
		-58, 4,
		-270, -97,
		15,
	},

	{ 0, 0, 0, 0, 0 }
};

/* Max. supported LED distance from wheel centre */
#define DIST_MAX 300.0f

/* Local static variables moved here so they can be saved / restored
 * to/from eeprom as a block */
static struct {
#define EEPROM_VER 1
#define EEPROM_MAGIC (0x00abcdef + EEPROM_VER)
  uint32_t magic;

  uint8_t prog;
  uint16_t gyro_mult;
  float cf_acc[2];
  uint8_t cf_samples;
} config, prev_config;

static uint8_t signal_cnt;
static RGB_t signal_rgb;

/*
 * Don't actually use this by default, seem to kill arduino
 * boards left and right...
 */
//#define USE_EEPROM

static void eeprom_load(void) {
#ifdef USE_EEPROM
  eeprom_read_block(&config, NULL, sizeof(config));
#endif

  if (config.magic != EEPROM_MAGIC) {
    /* No saved config found, reset the config */
    config.magic = EEPROM_MAGIC;

    config.prog = 0;
    config.gyro_mult = 1 << 14;
    config.cf_acc[0] = 50.0f * 65536 * 16;
    config.cf_acc[1] = -150.0f * 65536 * 16;
    config.cf_samples = 0;
  }

  prev_config = config;
}

static void eeprom_save(void) {
#ifdef USE_EEPROM
  /* Check if there's any change */

#define DIFF(f) abs(config.f - prev_config.f)
#define DIFF_CHK(f) (abs(prev_config.f > 0.00001f) && \
    abs((config.f - prev_config.f) / prev_config.f) > 0.1f)

  if (!(/*DIFF_CHK(cf_acc[0]) || DIFF_CHK(cf_acc[1]) ||*/
      DIFF(gyro_mult) > 40))
    return;

  eeprom_write_block(&config, NULL, sizeof(config));

  prev_config = config;

  signal_cnt = 100;
  signal_rgb.r = 100;
  signal_rgb.g = 0;
  signal_rgb.b = 100;
#endif
}

#define DEGS_TO_ANGLE(x) (uint16_t) (x / 180.0f * (1uLL << 15))
#define DEG_PER_S_TO_RATE(x) (int16_t) (x * 16.4f)

void setup(void) {
  struct led_strip_s *s;
  int i;
  float x, y;

  /* Deactivate internal pull-ups for twi as per note from atmega8 manual */
  PORTC &= ~((1 << 4) | (1 << 5));

  /* Switch to 400KHz I2C */
  TWBR = ((F_CPU / 400000L) - 16) / 2;

  accgyro = MPU60X0(false, MPU60X0_DEFAULT_ADDRESS);
  accgyro.initialize();
  accgyro.setI2CMasterModeEnabled(0);
  accgyro.setI2CBypassEnabled(1);
  accgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000);
  delay(5);
  zero_gyro();

  /* Initialise LED stuff */
  LED_DDR |= 1 << LED_PIN;
  LED_PORT &= ~(1 << LED_PIN);

  led_cnt = 0;
  for (s = &strips[0]; s->count; s ++) {
    for (i = 0; i < s->count; i ++) {
      x = s->x0 + (s->x1 - s->x0) * i / (float) (s->count - 1);
      y = s->y0 + (s->y1 - s->y0) * i / (float) (s->count - 1);

      led_dist[led_cnt] =
        sqrt((float) x * x + (float) y * y) / DIST_MAX * 127.0;
      led_angle[led_cnt] = 32768L - (uint16_t)
        ((atan2(x, y) * (32768.0 / M_PI)));
      led_cnt ++;
    }
  }
  min_dist = led_dist[0];
  full_dist = led_dist[14] - led_dist[0];

  eeprom_load();
  
  Serial.begin(115200);
}

/* Text font data */
extern const prog_uchar fontdata_8x8[];

#define LED_ON 100

/*
 * The functions below animate the LEDs in different ways
 * as the wheel turns.  Only one program is active at any
 * time.
 */

/* Illuminate one half of the wheel disc */
static void prog_slow_set_leds(uint16_t zero_angle, RGB_t *rgb) {
  for (uint8_t i = 0; i < led_cnt; i ++) {
    uint16_t angle = zero_angle - led_angle[i];

    rgb[i].r = angle > DEGS_TO_ANGLE(180.0f) ? LED_ON : 0;
    rgb[i].g = angle > DEGS_TO_ANGLE(180.0f) ? LED_ON : 0;
    rgb[i].b = angle > DEGS_TO_ANGLE(180.0f) ? LED_ON : 0;
  }
}

/* Display hardcoded text */
static void prog_fast_multi_set_leds(uint16_t zero_angle, RGB_t *rgb) {
  static const char *label = "Bicicritica ";
  static const int label_len = 12;
  static const uint16_t angle_len = DEGS_TO_ANGLE(180.0f);
  static const uint16_t maxangle = min((uint32_t) 65535,
    (uint32_t) angle_len * (65536uLL / (uint32_t) angle_len));
  static const int fontwidth = 8;
  static const int fontheight = 8;

  for (uint8_t i = 0; i < led_cnt; i ++) {
    uint16_t angle = zero_angle - led_angle[i];
    uint8_t val, chnum;

    if (angle >= maxangle) {
      val = 0;
    } else {
      uint16_t pxpos = angle / pxangle;
      chnum = (pxpos / fontwidth) % label_len;
      char ch = label[chnum];
      uint8_t x = pxpos % fontwidth;
      uint8_t y = (int) 15 * (led_dist[i] - min_dist) / full_dist;

      val = y < (15 - fontheight) ? 0 :
        (pgm_read_byte(&fontdata_8x8[ch * fontheight + (15 - 1 - y)]) >>
	 (7 - x)) & 1;
    }

    rgb[i].r = val ? 200 : 0;
    rgb[i].g = (val && chnum) ? 200 : 0;
    rgb[i].b = (val && chnum) ? 200 : 0;
  }
}

/*
 * Illuminate all LEDs with colour set in signal_rgb to
 * signal some sort of event -- the colour tells the user what
 * happened.
 */
static void prog_signal_set_leds(RGB_t *rgb) {
  for (uint8_t i = 0; i < led_cnt; i ++) {
    rgb[i].r = signal_rgb.r;
    rgb[i].g = signal_rgb.g;
    rgb[i].b = signal_rgb.b;
  }
}

/* All LEDs off */
static void prog_off_set_leds(uint16_t zero_angle, RGB_t *rgb) {
  for (uint8_t i = 0; i < led_cnt; i ++) {
    rgb[i].r = 0;
    rgb[i].g = 0;
    rgb[i].b = 0;
  }
}

/* All LEDs full power */
static void prog_on_set_leds(uint16_t zero_angle, RGB_t *rgb) {
  for (uint8_t i = 0; i < led_cnt; i ++) {
    rgb[i].r = LED_ON;
    rgb[i].g = LED_ON;
    rgb[i].b = LED_ON;
  }
}

/* Moving spiral shape */
static void prog_spiral_set_leds(uint16_t zero_angle, RGB_t *rgb) {
  for (uint8_t i = 0; i < led_cnt; i ++) {
    uint16_t angle = zero_angle - led_angle[i];

    uint16_t pos = angle -
      (millis() << 4) +
      ((uint32_t) led_dist[i] << 17) / full_dist;

    uint8_t brightness = pos >> 6;
    uint8_t blue = pos >> 15;
    if (pos & (1 << 14))
      brightness ^= 255;

    /* Decrease "gamma", these LEDs are bright at low values */
    brightness -= min(brightness, (brightness ^ 255) / 3);

    rgb[i].r = blue ? 0 : brightness;
    rgb[i].g = blue ? 0 : brightness;
    rgb[i].b = brightness;
  }
}

static int16_t gyro_reading;
static void gyro_update(void) {
  gyro_reading = accgyro.getRotationZ();
  gyro_reading -= (gyro_offset[2] + (1 << (CALIB_SHIFT - 1))) >> CALIB_SHIFT;
  gyro_reading = -gyro_reading;
}

static int16_t acc_reading[3];
static int16_t acc[2];
static int16_t cf_acc_int[2];
static void acc_update(void) {
  accgyro.getAcceleration(acc_reading + 0,
        acc_reading + 1, acc_reading + 2);

  acc[0] = acc_reading[0];
  acc[1] = acc_reading[1];

  /*
   * If cf calibration is based on reasonably many samples, use it for
   * acc reading correction in gyro drift correction.
   */
  if (config.cf_samples > 128) {
    uint16_t factor = ((uint32_t) gyro_reading * gyro_reading) >> 20;
    acc[0] -= cf_acc_int[0] * factor;
    acc[1] -= cf_acc_int[1] * factor;
  }
}

static uint16_t angle;
static uint16_t angle_update(void) {
  unsigned long now = micros();
  static unsigned long prev = 0;
  uint16_t timediff = now - prev;
  prev = now;

#define MULT_BITS 4
  int16_t step = ((int32_t) gyro_reading * timediff *
      (1 << MULT_BITS)) /
    (int32_t) (360.0f * 16.4f * 1000000.0f / 65536.0f + 0.5f);
  /*            degs   lsb/deg    us/sec    lsb/360deg rounding */

  /* TODO: use optimised avr mult */
  step = ((int32_t) step * config.gyro_mult) >> (14 + MULT_BITS);

  angle += step;

  /*
   * Centrifugal force estimation.  We sum (acceleration vector / rotation
   * rate ^ 2) over time -- at least a few full turns at a high enough
   * rotation rate, to find the direction of the force and the relation
   * to rotation rate.  This way, knowing the rotation rate at any later point,
   * we can calculate the centrifugal force vector and subtract it from
   * accelerometer readings in order to better approximate the gravity
   * vector.
   *
   * The MPU6050 gyro has a max range of 2000deg/s and this is a slightly
   * faster than we expect a bicycle wheel to be able to turn, so we assume
   * a practical range of our centrifugal force calculations to be
   * 90deg/s - 2000deg/s.  We set a lower limit too because the force is
   * proportional to the square of rotation rate and below a given value
   * the force should be negligible.
   */
  static uint16_t angle_accum;
  static uint8_t iter_accum;

  angle_accum += abs(step);
  iter_accum += 1;
  if (iter_accum > 80) {
    acc_update();

    uint32_t len = ((int32_t) acc[0] * acc[0]) +
      ((int32_t) acc[1] * acc[1]);
    uint8_t correct = len > 0x10000000 / 2 && len < 0x10000000 * 2;

    uint16_t acc_angle = atan2(acc[0], acc[1]) *
      (-32768.0f / M_PI);
    int16_t err_angle = acc_angle - angle;

    /* Correct the current angle */
    if (correct)
      angle += (err_angle + 8) >> 4;

    /* Correct the gyro zero offset (angle integral) */
    if (correct)
      gyro_offset[2] += ((int32_t) err_angle << 5) / iter_accum;

    iter_accum = 0;

    if (angle_accum > DEGS_TO_ANGLE(30.0f) &&
        abs(gyro_reading) > DEG_PER_S_TO_RATE(90.0f)) {
      /*
       * Quite literally what is described in comment above.
       * Use floats so we don't have to worry about ranges.  Since this is
       * only done every now and then, the overhead should be fine.
       * TODO: increase the sample weight with rotation rate?
       */
      config.cf_acc[0] += (acc_reading[0] / ((float) gyro_reading * gyro_reading) -
        config.cf_acc[0]) * 0.01f;
      config.cf_acc[1] += (acc_reading[1] / ((float) gyro_reading * gyro_reading) -
        config.cf_acc[1]) * 0.01f;
      if (config.cf_samples < 255)
        config.cf_samples += 1;

      cf_acc_int[0] = config.cf_acc[0] * (65536.0f * 16.0f);
      cf_acc_int[1] = config.cf_acc[1] * (65536.0f * 16.0f);

      /* TODO: add phase shift between the cf calibration and acc-based gyro
       * drift correction. */

      /*
       * Update gyro rate multiplier, use 1/16 weight (the 14-bit
       * shift is reduced by 4 bits).
       * TODO: decrease weight with rotation rate? TODO: rounding?
       */
      static uint16_t prev_acc_angle = 0;
      static uint16_t prev_gyro_angle = 0;

      int16_t gyro_velo = prev_gyro_angle - angle;
      int16_t acc_velo = prev_acc_angle - acc_angle;
      prev_gyro_angle = angle;
      prev_acc_angle = acc_angle;

      if (correct && !(((uint16_t) acc_velo ^ gyro_velo) & 0x8000) &&
          angle_accum < DEGS_TO_ANGLE(150.0f))
        config.gyro_mult += ((uint32_t) abs(acc_velo) - abs(gyro_velo)) << (14 - 4);
//      config.gyro_mult += ((int32_t) err_angle << (14 - 4)) / angle_accum;

      if (config.gyro_mult < 0x3200)
        config.gyro_mult = 0x3200;
      if (config.gyro_mult > 0x5000)
        config.gyro_mult = 0x5000;

      eeprom_save();
      angle_accum = 0;
    }
  }

  return angle;
}

static uint8_t prog;
static void prog_update(void) {
  static uint8_t brakes = 0, braking = 0;
  static uint16_t brake_millis = 0;
  static int16_t prev_gyro = 0, brake_gyro = 0;
  static int16_t gyro_diff_lpf = 0;

  uint8_t now_braking = 0;

  static uint16_t prev_millis = 0;

  uint16_t now = millis();
  uint16_t timediff = now - prev_millis;
  prev_millis = now;

  int16_t gyro_diff = gyro_reading - prev_gyro;
  prev_gyro = gyro_reading;
  gyro_diff_lpf += gyro_diff - ((gyro_diff_lpf + 8) >> 4);

  /* Are we braking?  For now use hardcoded values + some deadband */
  if (gyro_diff_lpf < -50)
    now_braking = 1;
  else if (gyro_diff_lpf > -20)
    now_braking = 0;
  else
    now_braking = braking;

  if (now_braking != braking) {
    uint16_t millis_since;

    millis_since = now - brake_millis;
    brake_millis = now;

    if (millis_since > 100 && millis_since < 400 &&
        (brakes != 0 || braking)) {
      brakes += 1;
    } else {
      /*
       * If we've detected exactly three braking periods of given
       * length (100-400ms) separated by periods of the similar
       * length and then a 1 - 3s of no braking, switch to the
       * next program.  If there were two breaks, switch one program
       * back.
       */
      if (millis_since > 1000 && millis_since < 3000 &&
          (brakes == 3 || brakes == 5)) {
        if (brakes == 5) {
          config.prog += 1;
          if (config.prog >= 5)
            config.prog = 0;
        } else {
          config.prog -= 1;
          if ((int8_t) config.prog < 0)
            config.prog = 4;
        }
      }
      brakes = 0;
    }
    braking = now_braking;
  }

  if (signal_cnt) {
    signal_cnt --;
    prog = 100;
    return;
  }

  prog = config.prog;

  /* Stop display the text label if spinning too slow or backwards */
  if (prog == 1 && gyro_reading < DEG_PER_S_TO_RATE(300.0))
    prog = 0;

  /* TODO: draw reverse if spinnig backwards? */
}

void loop(void) {
  gyro_update();
  angle_update();
  prog_update();

  RGB_t ledsrgb[128];

  switch (prog) {
  case 0:
    prog_slow_set_leds(angle, ledsrgb);
    break;
  case 1:
    prog_fast_multi_set_leds(angle, ledsrgb);
    break;
  case 2:
    prog_off_set_leds(angle, ledsrgb);
    break;
  case 3:
    prog_on_set_leds(angle, ledsrgb);
    break;
  case 4:
    prog_spiral_set_leds(angle, ledsrgb);
    break;
  case 100:
    prog_signal_set_leds(ledsrgb);
    break;
  }

  WS2811RGB(ledsrgb, led_cnt);
}
