/*
 *
 * Copyright (c) [2022] by InvenSense, Inc.
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
#include "Arduino.h"
#include <invn/soniclib/sensor_fw/icu_init/icu_init.h>

#include "board/chbsp_chirp.h"

#define SEG_TYPE_TO_STR(segment_type)                                                                                  \
  (segment_type == CH_MEAS_SEG_TYPE_COUNT)                                                                           \
      ? "Count"                                                                                                  \
      : ((segment_type == CH_MEAS_SEG_TYPE_RX)                                                                   \
                     ? "RX   "                                                                                       \
                     : ((segment_type == CH_MEAS_SEG_TYPE_TX)                                                        \
                                ? "TX   "                                                                            \
                                : ((segment_type == CH_MEAS_SEG_TYPE_EOF) ? "EOF  " : "UNKNOWN")))

/* Configuration of measurement segments
 * The following symbols define the individual transmit and receive segments
 * within the measurement.
 */
#define CHIRP_TX_SEG_CYCLES (640) /* Transmit segment - length (cycles) */
#define CHIRP_TX_SEG_PULSE_WIDTH (3) /* width of each cycle pulse */
#define CHIRP_TX_SEG_PHASE (8) /* tx phase */
#define CHIRP_TX_SEG_INT_EN (0) /* no interrupt when done */

#define CHIRP_RX_SEG_0_SAMPLES (1) /* Receive segment 0 - single sample */
#define CHIRP_RX_SEG_0_GAIN_REDUCE (24) /* reduce gain */
#define CHIRP_RX_SEG_0_ATTEN (1) /* use attenuation */
#define CHIRP_RX_SEG_0_INT_EN (0) /* no interrupt when done */

/* CHIRP_RX_SEG_1_SAMPLES defined at runtime depending on range */
#define CHIRP_RX_SEG_1_GAIN_REDUCE (0) /* no gain reduction */
#define CHIRP_RX_SEG_1_ATTEN (0) /* no attenuation */
#define CHIRP_RX_SEG_1_INT_EN (1) /* generate interrupt when done (if eligible) */

// spi
static SPIClass *spi = NULL;
static uint8_t chip_select_id = 0;
static bool is_measure_ready = false;

/* Measurement configuration struct - starting/default values */
static ch_meas_config_t meas_config_init = {
    .odr = CH_ODR_FREQ_DIV_8,
    .meas_period = 0,
};

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num,
                                ch_interrupt_type_t int_type) {
  if (int_type == CH_INTERRUPT_TYPE_DATA_RDY) {
    is_measure_ready = true;
  }
}

// ICUX0201 constructor for spi interface
ICUX0201::ICUX0201(SPIClass &spi_ref, uint32_t freq, uint8_t cs_id,
                   uint8_t int1_id, uint8_t int2_id, uint8_t mutclk_id) {
  chbsp_module_init(spi_ref, freq, cs_id, int1_id, int2_id, mutclk_id);
  this->fw_init_func = icu_init_init; /* define a default firmware to use */
  this->fw_sensor_init_func = NULL;
}

// ICUX0201 constructor for spi interface
ICUX0201::ICUX0201(SPIClass &spi_ref, uint8_t cs_id, uint8_t int1_id) {
  chbsp_module_init(spi_ref, DEFAULT_SPI_CLOCK, cs_id, int1_id, UNUSED_PIN, UNUSED_PIN);
  this->fw_init_func = icu_init_init; /* define a default firmware to use */
  this->fw_sensor_init_func = NULL;
}

/* Initialize hardware and ICU sensor */
int ICUX0201::begin() {
  uint8_t rc = 0;
  chbsp_board_init(&chirp_group);
  /* TODO: make the firmware init mutable */
  rc |= ch_init(&chirp_device, &chirp_group, 0, this->fw_init_func);
  if (this->fw_sensor_init_func != NULL) {
    rc |= ch_set_init_firmware(&chirp_device, this->fw_sensor_init_func);
  }
  if (rc == 0) {
    rc = ch_group_start(&chirp_group);
  }
  if (rc == 0) {
    /* Register callback function to be called when Chirp sensor interrupts */
    ch_io_int_callback_set(&chirp_group, sensor_int_callback);
  }
  return rc;
}

uint16_t ICUX0201::get_max_range(void) {
  return ch_meas_samples_to_mm(&chirp_device, 0, ch_get_max_samples(&chirp_device));
};

uint16_t ICUX0201::get_measure_range(void) {
  uint16_t nb_samples = ch_meas_get_num_samples(&chirp_device, 0);
  return ch_meas_samples_to_mm(&chirp_device, 0, nb_samples);
};

/**
 * @brief      Configure a tx rx measure (using icu_init firmware)
 *
 * @param[in]  range_mm  The range in millimeters
 *
 * @return     0 if sensor configured !=0 otherwise
 */
int ICUX0201::configure_measure(uint16_t range_mm) {
  int rc = 0;
  uint16_t range_samples;

  if (range_mm == 0) {
    /* get max range from sensor maximum range */
    range_mm = get_max_range();
  }

  rc = ch_meas_init(&chirp_device, 0, &meas_config_init, NULL);
  if (rc == 0) {
    rc = ch_meas_add_segment_tx(&chirp_device, 0, CHIRP_TX_SEG_CYCLES,
                                CHIRP_TX_SEG_PULSE_WIDTH, CHIRP_TX_SEG_PHASE,
                                CHIRP_TX_SEG_INT_EN);
  }
  if (rc == 0) {
    rc = ch_meas_add_segment_rx(&chirp_device, 0, CHIRP_RX_SEG_0_SAMPLES,
                                CHIRP_RX_SEG_0_GAIN_REDUCE, CHIRP_RX_SEG_0_ATTEN,
                                CHIRP_RX_SEG_0_INT_EN);
  }
  /* convert the range in mm to samples (which is the unit used by the sensor)
   */
  if (rc == 0) {
    range_samples = ch_meas_mm_to_samples(&chirp_device, 0, range_mm);
    rc = ch_meas_add_segment_rx(&chirp_device, 0, range_samples,
                                CHIRP_RX_SEG_1_GAIN_REDUCE, CHIRP_RX_SEG_1_ATTEN,
                                CHIRP_RX_SEG_1_INT_EN);
  }
  if (rc == 0) {
    rc = ch_meas_write_config(&chirp_device);
  }
  return rc;
}

int ICUX0201::free_run(void) { return free_run(get_max_range()); }

int ICUX0201::free_run(uint16_t range_mm) {
  return free_run(range_mm, default_odr_ms);
}

int ICUX0201::free_run(uint16_t range_mm, uint16_t interval_ms) {
  int rc;

  rc = configure_measure(range_mm);

  if (rc == 0) {
    rc = ch_meas_set_interval(&this->chirp_device, 0, interval_ms);
  }
  if (rc == 0) {
    rc = ch_meas_write_config(&chirp_device);
  }
  if (rc == 0) {
    chbsp_set_int1_dir_in(&chirp_device);
    chbsp_int1_interrupt_enable(&chirp_device); // enable interrupt
    rc = ch_set_mode(&chirp_device, CH_MODE_FREERUN);
  }
  return rc;
}

bool ICUX0201::data_ready(void) { return is_measure_ready; }

void ICUX0201::clear_data_ready(void) { is_measure_ready = false; }

uint16_t ICUX0201::part_number(void) {
  return ch_get_part_number(&chirp_device);
}

const char *ICUX0201::sensor_id(void) {
  return ch_get_sensor_id(&chirp_device);
}

uint32_t ICUX0201::frequency(void) { return ch_get_frequency(&chirp_device); }

uint16_t ICUX0201::bandwidth(void) { return ch_get_bandwidth(&chirp_device); }

uint16_t ICUX0201::rtc_cal(void) {
  return ch_get_rtc_cal_result(&chirp_device);
}

uint16_t ICUX0201::rtc_cal_pulse_length(void) {
  return ch_get_rtc_cal_pulselength(&chirp_device);
}

float ICUX0201::cpu_freq(void) {
  return ch_get_cpu_frequency(&chirp_device) / 1000000.0f;
}
const char *ICUX0201::fw_version(void) {
  return ch_get_fw_version_string(&chirp_device);
}

void ICUX0201::print_informations(HardwareSerial &serial) {
  serial.println("Sensor informations");
  serial.print("    Type: ");
  serial.println(part_number());
  serial.print("    ID: ");
  serial.println(sensor_id());
  serial.print("    Operating Frequency (Hz): ");
  serial.println(frequency());
  serial.print("    Bandwidth (Hz): ");
  serial.println(bandwidth());
  serial.print("    RTC Cal (lsb @ ms): ");
  serial.print(rtc_cal());
  serial.print("@");
  serial.println(rtc_cal_pulse_length());
  serial.print("    CPU Freq (MHz): ");
  serial.println(cpu_freq());
  serial.print("    max range (mm): ");
  serial.println(get_max_range());
  serial.print("    Firmware: ");
  serial.println(fw_version());
}

void ICUX0201::print_configuration(HardwareSerial &serial) {
  serial.println("Sensor configuration");
  serial.print("    measure range (mm): ");
  serial.println(get_measure_range());

#if (ICUX0201_DEBUG == 1)
  /* Print the measure queues */
  ch_meas_info_t meas_info;
  ch_meas_seg_info_t seg_info;
  uint8_t dev_num = this->chirp_device.io_index;

  serial.println("");

  for (uint8_t meas_num = 0; meas_num < MEAS_QUEUE_MAX_MEAS; meas_num++) {
    ch_meas_get_info(&this->chirp_device, meas_num, &meas_info);

    if (meas_info.num_segments == 0)
      continue;

    serial.print("<ICUX0201> Device ");
    serial.print(dev_num);
    serial.print(" - Measurement Queue ");
    serial.print(meas_num);
    serial.println(" : ");

    serial.print("      Total samples = ");
    serial.print(meas_info.num_rx_samples);
    serial.print(" (");
    serial.print(ch_samples_to_mm(&this->chirp_device, meas_info.num_rx_samples));
    serial.println(" mm)");

    serial.print("      Active Segments = ");
    serial.print(meas_info.num_segments);
    serial.print("    CIC ODR = ");
    serial.println(meas_info.odr);

    for (int seg_num = 0; seg_num <= meas_info.num_segments; seg_num++) { /* also display EOF */
      ch_meas_get_seg_info(&(this->chirp_device), meas_num, seg_num, &seg_info);
      serial.print("      Seg ");
      serial.print(seg_num);
      serial.print("  ");
      serial.print(SEG_TYPE_TO_STR(seg_info.type));
      if (seg_info.type == CH_MEAS_SEG_TYPE_EOF) {
        serial.println("");
        continue;
      } else {
        serial.print(" ");
      }
      if (seg_info.type == CH_MEAS_SEG_TYPE_RX) {
        serial.print(seg_info.num_rx_samples);
        serial.print(" sample(s) ");
      } else {
        serial.print("            ");
      }
      serial.print(seg_info.num_cycles);
      serial.print(" cycles ");
      if (seg_info.type == CH_MEAS_SEG_TYPE_TX) {
        serial.print("Pulse width = ");
        serial.print(seg_info.tx_pulse_width);
        serial.print("  Phase = ");
        serial.print(seg_info.tx_phase);
        serial.print("  ");
      } else if (seg_info.type == CH_MEAS_SEG_TYPE_RX) {
        serial.print("Gain reduce = ");
        serial.print(seg_info.rx_gain);
        serial.print("  Atten = ");
        serial.print(seg_info.rx_atten);
        serial.print("  ");
      }
      if (seg_info.rdy_int_en) {
        serial.print("Rdy Int  ");
      }
      if (seg_info.done_int_en) {
        serial.print("Done Int");
      }
      serial.println("");
    }
    serial.println("");
  }
#endif /* ICUX0201_DEBUG == 1 */
}

uint8_t ICUX0201::get_iq_data(ch_iq_sample_t (&iq_data)[ICU_MAX_NUM_SAMPLES], uint16_t& nb_samples)
{
  nb_samples  = ch_meas_get_num_samples(&chirp_device, 0);

  /* Reading I/Q data in normal, blocking mode */
  uint8_t err = ch_get_iq_data(&chirp_device, iq_data, 0, nb_samples,
                        CH_IO_MODE_BLOCK);
  clear_data_ready();

  return err;
}
