/*
 *
 * Copyright (c) [2023] by InvenSense, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "ICUX0201.h"
#include <invn/soniclib/ch_rangefinder.h>
#include <invn/soniclib/sensor_fw/icu_gpt/icu_gpt.h>

#include "board/chbsp_chirp.h"

/* Target detection thresholds
 * These definitions set the sensor's target detection thresholds.  These
 * thresholds specify how large a signal must be received, and at what point in
 * the measurement, for the sensor to indicate that a target was detected and
 * calculate range, etc.
 *
 * Each threshold consists of a starting sample number within the measurement
 * and the corresponding amplitude value that must be reached.  A threshold
 * extends until the starting sample number of the next threshold, if any.
 *
 * These values are used to initialize the chirp_detect_thresholds structure
 * defined in main.c, which is passed to the ch_meas_init() function.
 */
#define CHIRP_THRESH_0_START (0) /* threshold 0 - must start at sample zero */
#define CHIRP_THRESH_0_LEVEL (2500)

#define CHIRP_THRESH_1_START (40) /* threshold 1 */
#define CHIRP_THRESH_1_LEVEL (800)

#define CHIRP_THRESH_2_START (70) /* threshold 2 */
#define CHIRP_THRESH_2_LEVEL (400)

#define CHIRP_THRESH_3_START (100) /* threshold 3 */
#define CHIRP_THRESH_3_LEVEL (200)

#define CHIRP_THRESH_4_START (160) /* threshold 4 */
#define CHIRP_THRESH_4_LEVEL (140)

#define CHIRP_THRESH_5_START (200) /* threshold 5 */
#define CHIRP_THRESH_5_LEVEL (95)

#define CHIRP_THRESH_6_START (240) /* threshold 6 */
#define CHIRP_THRESH_6_LEVEL (50)

#define CHIRP_THRESH_7_START (0) /* threshold 7 (not used) */
#define CHIRP_THRESH_7_LEVEL (0)

/* Configuration of measurement segments
 * The following symbols define the individual transmit and receive segments
 * within the measurement.
 */
#define CHIRP_TX_SEG_CYCLES (640)    /* Transmit segment - length (cycles) */
#define CHIRP_TX_SEG_PULSE_WIDTH (3) /* width of each cycle pulse */
#define CHIRP_TX_SEG_PHASE (8)       /* tx phase */
#define CHIRP_TX_SEG_INT_EN (0)      /* no interrupt when done */

#define CHIRP_RX_SEG_0_SAMPLES (1)      /* Receive segment 0 - single sample */
#define CHIRP_RX_SEG_0_GAIN_REDUCE (24) /* reduce gain */
#define CHIRP_RX_SEG_0_ATTEN (1)        /* use attenuation */
#define CHIRP_RX_SEG_0_INT_EN (0)       /* no interrupt when done */

/* CHIRP_RX_SEG_1_SAMPLES defined at runtime depending on range */
#define CHIRP_RX_SEG_1_GAIN_REDUCE (0) /* no gain reduction */
#define CHIRP_RX_SEG_1_ATTEN (0)       /* no attenuation */
#define CHIRP_RX_SEG_1_INT_EN (1)      /* generate interrupt when done (if eligible) */

/* Measurement configuration struct - starting/default values */
static ch_meas_config_t meas_config_gpt = {
    .odr = CH_ODR_FREQ_DIV_8,
    .meas_period = 0,
    .mode = CH_MEAS_MODE_ACTIVE,
};

static icu_gpt_algo_config_t gpt_algo_config = {
    .ringdown_cancel_samples = 20,
    .static_filter_samples = 0, /* may be changed later using ch_set_static_range() */
    .iq_output_format = CH_OUTPUT_IQ, /* return (Q, I) */
    .num_ranges = 1,
    .filter_update_interval = 0, /* update filter every sample */
};

ch_thresholds_t chirp_detect_thresholds = {
    .threshold = {
        {CHIRP_THRESH_0_START, CHIRP_THRESH_0_LEVEL}, /* threshold 0 */
        {CHIRP_THRESH_1_START, CHIRP_THRESH_1_LEVEL}, /* threshold 1 */
        {CHIRP_THRESH_2_START, CHIRP_THRESH_2_LEVEL}, /* threshold 2 */
        {CHIRP_THRESH_3_START, CHIRP_THRESH_3_LEVEL}, /* threshold 3 */
        {CHIRP_THRESH_4_START, CHIRP_THRESH_4_LEVEL}, /* threshold 4 */
        {CHIRP_THRESH_5_START, CHIRP_THRESH_5_LEVEL}, /* threshold 5 */
        {CHIRP_THRESH_6_START, CHIRP_THRESH_6_LEVEL}, /* threshold 6 */
        {CHIRP_THRESH_7_START, CHIRP_THRESH_7_LEVEL}, /* threshold 7 */
    }
};

static InvnAlgoRangeFinderConfig gpt_algo_instance;

int ICUX0201_GeneralPurpose::configure_measure(uint16_t range_mm) {
  int rc = 0;
  uint16_t range_samples;

  if (range_mm == 0) {
    /* get max range from sensor maximum range */
    range_mm = get_max_range();
  }

  rc |= ch_meas_init(&chirp_device, 0, &meas_config_gpt, NULL);
  rc |= ch_meas_add_segment_tx(&chirp_device, 0, CHIRP_TX_SEG_CYCLES,
                               CHIRP_TX_SEG_PULSE_WIDTH, CHIRP_TX_SEG_PHASE,
                               CHIRP_TX_SEG_INT_EN);
  rc |= ch_meas_add_segment_rx(&chirp_device, 0, CHIRP_RX_SEG_0_SAMPLES,
                               CHIRP_RX_SEG_0_GAIN_REDUCE, CHIRP_RX_SEG_0_ATTEN,
                               CHIRP_RX_SEG_0_INT_EN);
  /* convert the range in mm to samples (which is the unit used by the sensor)
   */
  range_samples = ch_meas_mm_to_samples(&chirp_device, 0, range_mm);
  rc |= ch_meas_add_segment_rx(&chirp_device, 0, range_samples,
                               CHIRP_RX_SEG_1_GAIN_REDUCE, CHIRP_RX_SEG_1_ATTEN,
                               CHIRP_RX_SEG_1_INT_EN);
  rc |= ch_meas_write_config(&chirp_device);

  rc |= icu_gpt_algo_init(&chirp_device, &gpt_algo_instance);
  rc |= icu_gpt_algo_configure(&chirp_device, 0, &gpt_algo_config,
                               &chirp_detect_thresholds);
  rc |= ch_set_algo_config(&chirp_device, &gpt_algo_instance);
  rc |= ch_init_algo(&chirp_device);

  return rc;
}

// ICUX0201 constructor for spi interface
ICUX0201_GeneralPurpose::ICUX0201_GeneralPurpose(SPIClass &spi_ref,
                                                 uint8_t cs_id, uint8_t int1_id,
                                                 uint8_t int2_id,
                                                 uint8_t mutclk_id)
    : ICUX0201(spi_ref, cs_id, int1_id, int2_id, mutclk_id) {
  this->fw_init_func = icu_gpt_init;
}

ICUX0201_GeneralPurpose::ICUX0201_GeneralPurpose(SPIClass &spi_ref,
                                                 uint8_t cs_id, uint8_t int1_id)
    : ICUX0201(spi_ref, cs_id, int1_id) {
  this->fw_init_func = icu_gpt_init;
}

float ICUX0201_GeneralPurpose::get_range(void) {
  float range_mm = 0.0;
  uint32_t range_q5;

  if (data_ready()) {
    range_q5 = ch_get_range(&chirp_device, CH_RANGE_ECHO_ONE_WAY);
    if (range_q5 != CH_NO_TARGET) {
      /* Display single detected target (if any) */
      range_mm = range_q5 / 32.0;
    } else {
      /* No target detected: range set to 0 */
      range_mm = 0.0;
    }
    clear_data_ready();
  }
  return range_mm;
}
