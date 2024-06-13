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
#include <invn/soniclib/sensor_fw/icu_gpt/icu_gpt.h>
#include <invn/soniclib/ch_rangefinder.h>

#include "board/chbsp_chirp.h"
#include <invn/soniclib/sensor_fw/icu_init/icu_init.h>

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

ch_thresholds_t chirp_detect_thresholds_icu10201 = {
    .threshold = {
        {0, 1600}, /* threshold 0 */
        {10, 800}, /* threshold 1 */
        {20, 400}, /* threshold 2 */
        {25, 200}, /* threshold 3 */
        {40, 140}, /* threshold 4 */
        {50, 95}, /* threshold 5 */
        {60, 50}, /* threshold 6 */
        {70, 25} /* threshold 7 */
    }
};
ch_thresholds_t chirp_detect_thresholds_icu20201 = {
    .threshold = {
        {0, 1000}, /* threshold 0 */
        {15, 10000}, /* threshold 1 */
        {20, 3500}, /* threshold 2 */
        {25, 2500}, /* threshold 3 */
        {60, 1200}, /* threshold 4 */
        {130, 600}, /* threshold 5 */
        {160, 450}, /* threshold 6 */
        {190, 200} /* threshold 7 */
    }
};
ch_thresholds_t chirp_detect_thresholds_icu30201 = {
    .threshold = {
        {0, 1600}, /* threshold 0 */
        {20, 1200}, /* threshold 1 */
        {40, 1000}, /* threshold 2 */
        {50, 800}, /* threshold 3 */
        {80, 600}, /* threshold 4 */
        {100, 300}, /* threshold 5 */
        {120, 150}, /* threshold 6 */
        {140, 85} /* threshold 7 */
    }
};

static InvnAlgoRangeFinderConfig gpt_algo_instance;

int ICUX0201_dev_GeneralPurpose::configure_measure(uint16_t range_mm, ch_mode_t mode) {
  int rc = 0;
  uint16_t range_samples;
  ch_thresholds_t *thresholds;
  uint16_t part = part_number();
  if(part == 10201)
  {
    thresholds = &chirp_detect_thresholds_icu10201;
  } else if(part = 30201) {
    thresholds = &chirp_detect_thresholds_icu30201;    
  } else {
    thresholds = &chirp_detect_thresholds_icu20201;
  }

  if (range_mm == 0) {
    /* get max range from sensor maximum range */
    range_mm = get_max_range();
  }

  rc |= ch_meas_init(this, 0, &meas_config_gpt, NULL);
  if ((mode != CH_MODE_TRIGGERED_RX_ONLY) && (mode != CH_MODE_CONTINUOUS_RX))
  {
    rc |= ch_meas_add_segment_tx(this, 0, CHIRP_TX_SEG_CYCLES,
                                 CHIRP_TX_SEG_PULSE_WIDTH, CHIRP_TX_SEG_PHASE,
                                 CHIRP_TX_SEG_INT_EN);
  } else {
    rc |= ch_meas_add_segment_count(this, 0, CHIRP_TX_SEG_CYCLES, 0);
  }
  rc |= ch_meas_add_segment_rx(this, 0, CHIRP_RX_SEG_0_SAMPLES,
                               CHIRP_RX_SEG_0_GAIN_REDUCE, CHIRP_RX_SEG_0_ATTEN,
                               CHIRP_RX_SEG_0_INT_EN);
  /* convert the range in mm to samples (which is the unit used by the sensor)
   */
  range_samples = ch_meas_mm_to_samples(this, 0, range_mm);
  rc |= ch_meas_add_segment_rx(this, 0, range_samples,
                               CHIRP_RX_SEG_1_GAIN_REDUCE, CHIRP_RX_SEG_1_ATTEN,
                               CHIRP_RX_SEG_1_INT_EN);
  rc |= ch_meas_write_config(this);
  if(part == 10201)
  {
    /* For Tahoe: tx optimized to reduce ringdown */
    rc |= ch_meas_optimize(this, NULL, NULL);
  }
  rc |= icu_gpt_algo_init(this, &gpt_algo_instance);
  rc |= icu_gpt_algo_configure(this, 0, &gpt_algo_config,
                               thresholds);
  rc |= ch_set_algo_config(this, &gpt_algo_instance);
  rc |= ch_init_algo(this);

  return rc;
}

// ICUX0201 constructor for spi interface
ICUX0201_dev_GeneralPurpose::ICUX0201_dev_GeneralPurpose(SPIClass &spi_ref, uint32_t freq,
                                                 int cs_id, int int1_id,
                                                 int int2_id,
                                                 int mutclk_id)
    : ICUX0201_dev(spi_ref, freq, cs_id, int1_id, int2_id, mutclk_id) {
  fw_init_func = icu_gpt_init;
  /* Needed for tx_opt for Tahoe */
  fw_sensor_init_func = icu_init_init;
}

ICUX0201_dev_GeneralPurpose::ICUX0201_dev_GeneralPurpose(SPIClass &spi_ref,
                                                 int cs_id, int int1_id)
    : ICUX0201_dev(spi_ref, cs_id, int1_id) {
  fw_init_func = icu_gpt_init;
  /* Needed for tx_opt for Tahoe */
  fw_sensor_init_func = icu_init_init;
}

float ICUX0201_dev_GeneralPurpose::get_range(void) {
  float range_mm = 0.0;
  uint32_t range_q5;

  if (data_ready()) {
    range_q5 = ch_get_target_range(this, 0, (ch_get_mode(this) == CH_MODE_TRIGGERED_RX_ONLY)? (CH_RANGE_DIRECT) : (CH_RANGE_ECHO_ONE_WAY));
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

