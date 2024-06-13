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

#include "ICUX0201_dev.h"
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

// ICUX0201_dev constructor for spi interface
ICUX0201_dev::ICUX0201_dev(SPIClass &spi_ref, uint32_t freq, int cs_id,
                   int int1_id, int int2_id, int mutclk_id) {
  fw_init_func = icu_init_init; /* define a default firmware to use */
  fw_sensor_init_func = NULL;
  spi = &spi_ref;
  spi_freq = DEFAULT_SPI_CLOCK;
  if((freq < MAX_SPI_CLOCK)&&(freq >= 100000))
  {
    spi_freq = freq;
  }
  spi_cs_pin_id = cs_id;
  int1_pin_id = int1_id;
  int2_pin_id = int2_id;
  mutclk_pin_id = mutclk_id;
  int1_attached = false;
  int1_in_dir = false;
}

// ICUX0201_dev constructor for spi interface
ICUX0201_dev::ICUX0201_dev(SPIClass &spi_ref,int cs_id,
                   int int1_id) {
  fw_init_func = icu_init_init; /* define a default firmware to use */
  fw_sensor_init_func = NULL;
  spi = &spi_ref;
  spi_freq = DEFAULT_SPI_CLOCK;
  spi_cs_pin_id = cs_id;
  int1_pin_id = int1_id;
  int2_pin_id = UNUSED_PIN;
  mutclk_pin_id = UNUSED_PIN;
  int1_attached = false;
  int1_in_dir = false;
}

/* Initialize hardware and ICU sensor */
int ICUX0201_dev::begin(ch_group_t* group_ptr, int id) {
  uint8_t rc = 0;
  
  pinMode(spi_cs_pin_id,OUTPUT);
  digitalWrite(spi_cs_pin_id, HIGH);

  /* Assume sensors are on the same bus, only sensor 0 inits SPI */
  if(id == 0)
  {
    spi->begin();
  }

  /* Configure INT pins as input */
  set_int1_dir_in();

  /* Configure MUTCLK as output with high level (not used by ICU sensors as clk) */
  if(mutclk_pin_id != UNUSED_PIN)
  {
    pinMode(mutclk_pin_id, OUTPUT);
    digitalWrite(mutclk_pin_id,HIGH);
  }
  if(int2_pin_id != UNUSED_PIN)
    set_int2_dir_in();
  
  /* TODO: make the firmware init mutable */
  rc |= ch_init(this, group_ptr, id, fw_init_func);

  if (fw_sensor_init_func != NULL) {
    rc |= ch_set_init_firmware(this, fw_sensor_init_func);
  }
  return rc;
}

uint16_t ICUX0201_dev::get_max_range(void) {
  return ch_meas_samples_to_mm(this, 0, ch_get_max_samples(this));
};

uint16_t ICUX0201_dev::get_measure_range(void) {
  uint16_t nb_samples = ch_meas_get_num_samples(this, 0);
  return ch_meas_samples_to_mm(this, 0, nb_samples);
};

/**
 * @brief      Configure a tx rx measure (using icu_init firmware)
 *
 * @param[in]  range_mm  The range in millimeters
 * @param[in]  mode      Defines if the sensor has a TX phase
 *
 * @return     0 if sensor configured !=0 otherwise
 */
int ICUX0201_dev::configure_measure(uint16_t range_mm, ch_mode_t mode) {
  int rc = 0;
  uint16_t range_samples;

  if (range_mm == 0) {
    /* get max range from sensor maximum range */
    range_mm = get_max_range();
  }

  rc = ch_meas_init(this, 0, &meas_config_init, NULL);
  if ((mode != CH_MODE_TRIGGERED_RX_ONLY) && (mode != CH_MODE_CONTINUOUS_RX))
  {
    if (rc == 0) {
      rc = ch_meas_add_segment_tx(this, 0, CHIRP_TX_SEG_CYCLES,
                                  CHIRP_TX_SEG_PULSE_WIDTH, CHIRP_TX_SEG_PHASE,
                                  CHIRP_TX_SEG_INT_EN);
    }
  } else {
    rc = ch_meas_add_segment_count(this, 0, CHIRP_TX_SEG_CYCLES, 0);
  }
  if (rc == 0) {
    rc = ch_meas_add_segment_rx(this, 0, CHIRP_RX_SEG_0_SAMPLES,
                                CHIRP_RX_SEG_0_GAIN_REDUCE, CHIRP_RX_SEG_0_ATTEN,
                                CHIRP_RX_SEG_0_INT_EN);
  }
  /* convert the range in mm to samples (which is the unit used by the sensor)
   */
  if (rc == 0) {
    range_samples = ch_meas_mm_to_samples(this, 0, range_mm);
    rc = ch_meas_add_segment_rx(this, 0, range_samples,
                                CHIRP_RX_SEG_1_GAIN_REDUCE, CHIRP_RX_SEG_1_ATTEN,
                                CHIRP_RX_SEG_1_INT_EN);
  }
  if (rc == 0) {
    rc = ch_meas_write_config(this);
  }
  return rc;
}

int ICUX0201_dev::free_run(void) { return free_run(get_max_range()); }

int ICUX0201_dev::free_run(uint16_t range_mm) {
  return free_run(range_mm, default_odr_ms);
}

int ICUX0201_dev::free_run(uint16_t range_mm, uint16_t interval_ms) {
  int rc;

  rc = configure_measure(range_mm);

  if (rc == 0) {
    rc = ch_meas_set_interval(this, 0, interval_ms);
  }
  if (rc == 0) {
    rc = ch_meas_write_config(this);
  }
  if (rc == 0) {
    chbsp_set_int1_dir_in(this);
    chbsp_int1_interrupt_enable(this); // enable interrupt
    rc = ch_set_mode(this, CH_MODE_FREERUN);
  }
  return rc;
}

int ICUX0201_dev::start_trigger(uint16_t range_mm, ch_mode_t mode) {
  int rc;

  rc = configure_measure(range_mm, mode);

  if (rc == 0) {
    rc = ch_set_mode(this, mode);
  }

  return rc;
}

void ICUX0201_dev::trig(void) {
  ch_trigger(this);
}

bool ICUX0201_dev::data_ready(void) { return is_measure_ready; }

void ICUX0201_dev::clear_data_ready(void) { is_measure_ready = false; }

void ICUX0201_dev::set_data_ready(void) { is_measure_ready = true; }

uint16_t ICUX0201_dev::part_number(void) {
  return ch_get_part_number(this);
}

const char *ICUX0201_dev::sensor_id(void) {
  return ch_get_sensor_id(this);
}

uint32_t ICUX0201_dev::frequency(void) { return ch_get_frequency(this); }

uint16_t ICUX0201_dev::bandwidth(void) { return ch_get_bandwidth(this); }

uint16_t ICUX0201_dev::rtc_cal(void) {
  return ch_get_rtc_cal_result(this);
}

uint16_t ICUX0201_dev::rtc_cal_pulse_length(void) {
  return ch_get_rtc_cal_pulselength(this);
}

float ICUX0201_dev::cpu_freq(void) {
  return ch_get_cpu_frequency(this) / 1000000.0f;
}
const char *ICUX0201_dev::fw_version(void) {
  return ch_get_fw_version_string(this);
}

void ICUX0201_dev::print_informations(void) {
  Serial.print("Sensor ");
  Serial.print(io_index);
  Serial.println(" informations");
  Serial.print("    Type: ");
  Serial.println(part_number());
  Serial.print("    ID: ");
  Serial.println(sensor_id());
  Serial.print("    Operating Frequency (Hz): ");
  Serial.println(frequency());
  Serial.print("    Bandwidth (Hz): ");
  Serial.println(bandwidth());
  Serial.print("    RTC Cal (lsb @ ms): ");
  Serial.print(rtc_cal());
  Serial.print("@");
  Serial.println(rtc_cal_pulse_length());
  Serial.print("    CPU Freq (MHz): ");
  Serial.println(cpu_freq());
  Serial.print("    max range (mm): ");
  Serial.println(get_max_range());
  Serial.print("    Firmware: ");
  Serial.println(fw_version());
}

void ICUX0201_dev::print_configuration(void) {
  Serial.print("Sensor ");
  Serial.print(io_index);
  Serial.println(" configuration");
  Serial.print("    measure range (mm): ");
  Serial.println(get_measure_range());

#if (ICUX0201_DEBUG == 1)
  /* Print the measure queues */
  ch_meas_info_t meas_info;
  ch_meas_seg_info_t seg_info;

  Serial.println("");

  for (uint8_t meas_num = 0; meas_num < MEAS_QUEUE_MAX_MEAS; meas_num++) {
    ch_meas_get_info(this, meas_num, &meas_info);

    if (meas_info.num_segments == 0)
      continue;

    Serial.print("<ICUX0201> Device ");
    Serial.print(io_index);
    Serial.print(" - Measurement Queue ");
    Serial.print(meas_num);
    Serial.println(" : ");

    Serial.print("      Total samples = ");
    Serial.print(meas_info.num_rx_samples);
    Serial.print(" (");
    Serial.print(ch_samples_to_mm(this, meas_info.num_rx_samples));
    Serial.println(" mm)");

    Serial.print("      Active Segments = ");
    Serial.print(meas_info.num_segments);
    Serial.print("    CIC ODR = ");
    Serial.println(meas_info.odr);

    for (int seg_num = 0; seg_num <= meas_info.num_segments; seg_num++) { /* also display EOF */
      ch_meas_get_seg_info(this, meas_num, seg_num, &seg_info);
      Serial.print("      Seg ");
      Serial.print(seg_num);
      Serial.print("  ");
      Serial.print(SEG_TYPE_TO_STR(seg_info.type));
      if (seg_info.type == CH_MEAS_SEG_TYPE_EOF) {
        Serial.println("");
        continue;
      } else {
        Serial.print(" ");
      }
      if (seg_info.type == CH_MEAS_SEG_TYPE_RX) {
        Serial.print(seg_info.num_rx_samples);
        Serial.print(" sample(s) ");
      } else {
        Serial.print("            ");
      }
      Serial.print(seg_info.num_cycles);
      Serial.print(" cycles ");
      if (seg_info.type == CH_MEAS_SEG_TYPE_TX) {
        Serial.print("Pulse width = ");
        Serial.print(seg_info.tx_pulse_width);
        Serial.print("  Phase = ");
        Serial.print(seg_info.tx_phase);
        Serial.print("  ");
      } else if (seg_info.type == CH_MEAS_SEG_TYPE_RX) {
        Serial.print("Gain reduce = ");
        Serial.print(seg_info.rx_gain);
        Serial.print("  Atten = ");
        Serial.print(seg_info.rx_atten);
        Serial.print("  ");
      }
      if (seg_info.rdy_int_en) {
        Serial.print("Rdy Int  ");
      }
      if (seg_info.done_int_en) {
        Serial.print("Done Int");
      }
      Serial.println("");
    }
    Serial.println("");
  }
#endif /* ICUX0201_DEBUG == 1 */
}

uint8_t ICUX0201_dev::get_iq_data(ch_iq_sample_t (&iq_data)[ICU_MAX_NUM_SAMPLES], uint16_t& nb_samples)
{
  nb_samples  = ch_meas_get_num_samples(this, 0);

  /* Reading I/Q data in normal, blocking mode */
  uint8_t err = ch_get_iq_data(this, iq_data, 0, nb_samples,
                        CH_IO_MODE_BLOCK);
  clear_data_ready();

  return err;
}

void ICUX0201_dev::set_int1_dir_in(void)
{
  if(!int1_in_dir)
  {
    pinMode(int1_pin_id,INPUT_PULLUP);
    int1_in_dir = true;
  }
}

void ICUX0201_dev::set_int1_dir_out(void)
{
  if(int1_in_dir)
  {
    pinMode(int1_pin_id,OUTPUT);
    int1_in_dir = false;
  }
}

void ICUX0201_dev::set_int1(bool value)
{
  digitalWrite(int1_pin_id, value);
}


bool ICUX0201_dev::get_int1(void)
{
  return digitalRead(int1_pin_id);
}

void ICUX0201_dev::set_int2_dir_in(void)
{
  if(!int2_in_dir)
  {
    pinMode(int2_pin_id,INPUT_PULLUP);
    int2_in_dir = true;
  }
}

void ICUX0201_dev::set_int2_dir_out(void)
{
  if(int1_in_dir)
  {
    pinMode(int2_pin_id,OUTPUT);
    int2_in_dir = false;
  }
}

void ICUX0201_dev::set_int2(bool value)
{
  digitalWrite(int2_pin_id, value);
}

void ICUX0201_dev::enableInterrupt(icux0201_dev_irq_handler handler)
{
  if (!int1_attached) {
    set_int1_dir_in();

    attachInterrupt(digitalPinToInterrupt(int1_pin_id),handler,LOW);
    int1_attached = true;
  }
}

void ICUX0201_dev::disableInterrupt(void)
{
  if (int1_attached) {
    detachInterrupt(digitalPinToInterrupt(int1_pin_id));
    int1_attached = false;
  }
}

void ICUX0201_dev::enableInterrupt2(icux0201_dev_irq_handler handler)
{
  if (!int2_attached) {
    set_int2_dir_in();

    attachInterrupt(digitalPinToInterrupt(int2_pin_id),handler,FALLING);
    int2_attached = true;
  }
}

void ICUX0201_dev::disableInterrupt2(void)
{
  if (int2_attached) {
    detachInterrupt(digitalPinToInterrupt(int2_pin_id));
    int2_attached = false;
  }
}

void ICUX0201_dev::spi_begin_transaction(void)
{
  digitalWrite(spi_cs_pin_id, LOW);
  spi->beginTransaction(SPISettings(spi_freq, MSBFIRST, SPI_MODE3));
}

void ICUX0201_dev::spi_end_transaction(void)
{
  spi->endTransaction();
  digitalWrite(spi_cs_pin_id, HIGH);
}

void ICUX0201_dev::spi_write(const uint8_t *data, uint16_t num_bytes)
{
  for(uint16_t i = 0; i < num_bytes; i++) {
    spi->transfer(data[i]);
  }
}

void ICUX0201_dev::spi_read(uint8_t *data, uint16_t num_bytes)
{
  spi->transfer(data,num_bytes);
}
