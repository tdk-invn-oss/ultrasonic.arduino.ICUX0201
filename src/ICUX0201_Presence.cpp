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

#include "Arduino.h"
#include "ICUX0201.h"

#include "board/chbsp_chirp.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <invn/soniclib/sensor_fw/icu_presence/icu_presence.h>
#include <invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.h>

#ifdef __cplusplus
}
#endif

/* Sensor configuration */
/* Tx step */
#define SENSOR_CFG_SEG0_TX_LENGTH_CYCLES (ICU_PRESENCE_RECOMMENDED_PULSE_LENGTH)
#define SENSOR_CFG_SEG0_TX_PULSE_WIDTH_SMP (3)
#define SENSOR_CFG_SEG0_TX_PHASE_SMP (8)
#define SENSOR_CFG_SEG0_TX_INT_EN (0)
/* Rx step to get the ultrasound echo */
#define SENSOR_CFG_SEG1_RX_GAIN_REDUCE (0)
#define SENSOR_CFG_SEG1_RX_ATTEN (0)
#define SENSOR_CFG_SEG1_RX_INT_EN (1)

extern measurement_queue_t sensor_settings_icu30201;

static ch_meas_config_t meas_config_presence = {
    .odr = (ch_odr_t)ICU_PRESENCE_RECOMMENDED_CIC_ODR,
    .meas_period = 0, /* will be set later */
};

ICUX0201_Presence::ICUX0201_Presence(SPIClass &spi_ref, uint8_t cs_id,
                                     uint8_t int1_id, uint8_t int2_id,
                                     uint8_t mutclk_id)
    : ICUX0201(spi_ref, cs_id, int1_id, int2_id, mutclk_id) {
  this->fw_init_func = icu_presence_init;
  this->fw_sensor_init_func = icu_init_no_txopt_init;
}

ICUX0201_Presence::ICUX0201_Presence(SPIClass &spi_ref, uint8_t cs_id,
                                     uint8_t int1_id)
    : ICUX0201(spi_ref, cs_id, int1_id) {
  this->fw_init_func = icu_presence_init;
  this->fw_sensor_init_func = icu_init_no_txopt_init;
}

int ICUX0201_Presence::configure_measure(uint16_t range_mm) {
  int rc = 0;
  uint16_t nb_samples    = 0;
  uint16_t range_samples;

  if (range_mm == 0) {
    range_mm = get_max_range();
  }

  /* convert the range in mm to samples (which is the unit used by the sensor) */
  range_samples = ch_meas_mm_to_samples(&this->chirp_device, 0, range_mm);
  if (range_samples == 0 || range_samples > ICU_PRESENCE_MAX_IQ_SAMPLES) {
#if (ICUX0201_DEBUG == 1)
    Serial.print("<ICUX0201> Can't set range to ");
    Serial.print(range_mm);
    Serial.print(" mm (= ");
    Serial.print(range_samples);
    Serial.print(" samples) > firmware internal buffer size (");
    Serial.print(ICU_PRESENCE_MAX_IQ_SAMPLES);
    Serial.println(" samples)");
#endif
    rc = 1;
  } else {
    if (ch_get_part_number(&this->chirp_device) == ICU30201_PART_NUMBER) {
      /* Initialize ICU30201 measure queue with multiple segments */
      rc = ch_meas_import(&this->chirp_device, &sensor_settings_icu30201, NULL); /* algo will be initialized later */
      /* Change last receive segment to reflect custom defined range */
      if (!rc) {
          rc = ch_meas_set_num_samples(&this->chirp_device, CH_DEFAULT_MEAS_NUM, nb_samples);
      }
    } else {
      rc |= ch_meas_init(&this->chirp_device, 0, &meas_config_presence, NULL);
      rc |= ch_meas_add_segment_tx(
          &this->chirp_device, 0, SENSOR_CFG_SEG0_TX_LENGTH_CYCLES,
          SENSOR_CFG_SEG0_TX_PULSE_WIDTH_SMP, SENSOR_CFG_SEG0_TX_PHASE_SMP,
          SENSOR_CFG_SEG0_TX_INT_EN);
      rc |= ch_meas_add_segment_rx(
          &this->chirp_device, 0, range_samples, SENSOR_CFG_SEG1_RX_GAIN_REDUCE,
          SENSOR_CFG_SEG1_RX_ATTEN, SENSOR_CFG_SEG1_RX_INT_EN);
      rc = ch_set_target_interrupt(&this->chirp_device, CH_TGT_INT_FILTER_OFF);
    }
  }

  return rc;
}

int ICUX0201_Presence::init_sensor_algorithm() {
  int rc;
  IcuPresenceSensorConfig sensor_config;
  IcuPresenceUserConfig user_config;
  uint16_t nb_range_samples = ch_get_num_samples(&this->chirp_device);

  nb_range_samples = ch_get_num_samples(&this->chirp_device);
  if (nb_range_samples > ICU_PRESENCE_MAX_IQ_SAMPLES) {
    nb_range_samples = ICU_PRESENCE_MAX_IQ_SAMPLES;
  }

  sensor_config.fop = ch_get_frequency(&this->chirp_device);
  sensor_config.odr = this->chirp_device.freerun_intvl_us;
  sensor_config.pulse_len = ch_get_tx_length(&this->chirp_device);
  sensor_config.sensor_type = ch_get_part_number(&this->chirp_device);
  sensor_config.cic_odr = ICU_PRESENCE_RECOMMENDED_CIC_ODR;
  sensor_config.listening_samples = ICU_PRESENCE_TYPICAL_LISTENING_SAMPLES;

  user_config.sensitivity = ICU_PRESENCE_DEFAULT_SENSITIVITY;
  user_config.hold_presence_s = ICU_PRESENCE_DEFAULT_HOLD_PRESENCE_S;
  user_config.event_mode = ICU_PRESENCE_DEFAULT_EVENT_MODE;

  rc = icu_presence_generate_config(&sensor_config, &user_config,
                                      &this->algo_config);
  return rc;
}

int ICUX0201_Presence::free_run(void) {
  uint16_t default_range;

  if (ch_get_part_number(&this->chirp_device) == ICU30201_PART_NUMBER) {
    default_range = icu30201_default_range_mm;
  } else {
    default_range = icu20201_default_range_mm;
  }
  return free_run(default_range);
}

int ICUX0201_Presence::free_run(uint16_t range_mm) {
  return free_run(range_mm, default_odr_ms);
}

int ICUX0201_Presence::free_run(uint16_t range_mm, uint16_t interval_ms) {
  int rc = 0;

  rc = configure_measure(range_mm);

  if (rc == 0) {
    rc = ch_set_freerun_interval(&this->chirp_device, interval_ms);
    rc |= ch_freerun_time_hop_disable(&this->chirp_device);
  }

  if (rc == 0) {
    rc = init_sensor_algorithm();
  }

  if (rc == 0) {
    rc = ch_meas_import(&this->chirp_device,
                        NULL, /* measure config already initialized */
                        &algo_config);
  }
  if (rc == 0) {
    rc = ch_meas_write_config(&this->chirp_device);
  }
  if (rc == 0) {
    rc = ch_init_algo(&this->chirp_device);
  }
  if (rc == 0) {
    rc = ch_set_mode(&this->chirp_device, CH_MODE_FREERUN);
  }
  if (rc == 0) {
    chbsp_set_int1_dir_in(&this->chirp_device);
    chbsp_int1_interrupt_enable(&this->chirp_device); // enable interrupt
  }
  return rc;
}

int ICUX0201_Presence::get_algo_output() {
  return ch_get_algo_output(&this->chirp_device, &this->last_algo_output);
}

bool ICUX0201_Presence::get_presence(void) {
  if (data_ready()) {
    get_algo_output();
    clear_data_ready();
  }

  return (this->last_algo_output.presence_detection == 1);
}

uint16_t ICUX0201_Presence::get_range(void) {
  if (data_ready()) {
    get_algo_output();
    clear_data_ready();
  }

  return this->last_algo_output.motion_range_cm;
}
