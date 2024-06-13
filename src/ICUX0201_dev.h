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

#ifndef ICUX0201_DEV_H
#define ICUX0201_DEV_H

#include "Arduino.h"
#include "SPI.h"
#include <stdint.h>
#include <string.h>

#include "invn/soniclib/soniclib.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <invn/soniclib/sensor_fw/icu_presence/icu_presence.h>

#ifdef __cplusplus
}
#endif

#define ICUX0201_DEBUG 0 //!< Debug mode
#define UNUSED_PIN (0xFF)
#define DEFAULT_SPI_CLOCK 11000000
#define MAX_SPI_CLOCK 13000000

typedef void (icux0201_dev_irq_handler)(void);

class ICUX0201_dev : public ch_dev_t {
public:
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param freq SPI clock frequency to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   * @param int2_id ID of the interrupt 2 pin
   * @param mutclk_id ID of the mut clock pin
   */
  ICUX0201_dev(SPIClass &spi_ref, uint32_t freq, int cs_id, int int1_id,
           int int2_id=UNUSED_PIN, int mutclk_id=UNUSED_PIN);
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
  ICUX0201_dev(SPIClass &spi_ref, int cs_id, int int1_id);
           /*!
   * @brief Configure the sensor
   * @param group Sensor group the sensor belongs (CHx01 object pointer)
   * @param id Id of the sensor in the group
   * @return 0 if successful
   */
   int begin(ch_group_t* group, int id);
  /*!
   * @brief Get the sensor part number.
   * @return Sensor part number as an integer
   */
  uint16_t part_number(void);
  /*!
   * @brief Get the sensor id.
   * @return Returns sensor ID as a char pointer
   */
  const char *sensor_id(void);
  /*!
   * @brief Get sensor's operating frequency.
   * @return Acoustic operating frequency, in Hz
   */
  uint32_t frequency(void);
  /*!
   * @brief Get sensor's bandwidth.
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t bandwidth(void);
  /*!
   * @brief Get the real-time clock calibration value.
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t rtc_cal(void);
  /*!
   * @brief Get the real-time clock calibration pulse length.
   * @return RTC pulse length, in ms
   */
  uint16_t rtc_cal_pulse_length(void);
  /*!
   * @brief Get the sensor CPU frequency, in Hz.
   * @return sensor CPU frequency, in Hz
   */
  float cpu_freq(void);
  /*!
   * @brief Get the firmware version description string.
   * @return Pointer to character string describing sensor firmware version
   */
  const char *fw_version(void);
  /*!
   * @brief Set measure ready.
   */
  void set_data_ready(void);
  /*!
   * @brief Check if a measure is available.
   * @return True if a measure is available, false otherwise
   */
  bool data_ready(void);
  /*!
   * @brief Get the sensor's max range.
   * @return Sensor's max range in mm
   * This value depends on sensor internal sample buffer size
   */
  uint16_t get_max_range(void);
  /*!
   * @brief Get the sensor's range configured for measures.
   * @return Sensor's measure range in mm
   * This value depends on sensor configured range
   */
  uint16_t get_measure_range(void);
  /*!
   * @brief Starts sensor in free running mode, with maximum range and default ODR.
   * @return 0 in case of success
   */
  virtual int free_run();
  /*!
   * @brief Starts sensor in free running mode, with provided max range and default ODR.
   * @param range_mm Max detection range to be configured
   * @return 0 in case of success
   */
  virtual int free_run(uint16_t range_mm);
  /*!
   * @brief Starts sensor in free running mode, with provided max range and ODR.
   * @param range_mm Max detection range to be configured
   * @param interval_ms Interval between samples, in milliseconds
   * @return 0 in case of success
   */
  virtual int free_run(uint16_t range_mm, uint16_t interval_ms);
  /*!
   * @brief Starts sensor in trigger mode, with provided max range.
   * @param range_mm Max detection range to be configured
   * @param mode Select sensor mode: CH_MODE_TRIGGERED_TX_RX or CH_MODE_TRIGGERED_RX_ONLY
   * @return 0 in case of success
   */
  int start_trigger(uint16_t range_mm, ch_mode_t mode);
  /*!
   * @brief Trig sensor measurement.
   */
  void trig(void);
  /*!
   * @brief Displays sensor information.
   */
  void print_informations(void);
  /*!
   * @brief Displays sensor configuration.
   */
  void print_configuration(void);
  /*!
   * @brief Get the raw I/Q measurement data from a sensor.
   * @param iq_data pointer to data buffer where I/Q data will be written (length = ICU_MAX_NUM_SAMPLES)
   * @param num_samples number of samples read from sensor
   * @return 0 if successful, 1 if error
   */
  uint8_t get_iq_data(ch_iq_sample_t (&iq_data)[ICU_MAX_NUM_SAMPLES], uint16_t& nb_samples);
  /*!
   * @brief Set int1 input direction
   */
  void set_int1_dir_in(void);
  /*!
   * @brief Set int1 output direction
   */
  void set_int1_dir_out(void);
  /*!
   * @brief Get int1 status
   */
  bool get_int1(void);
  /*!
   * @brief Set int2 input direction
   */
  void set_int2_dir_in(void);
  /*!
   * @brief Set int2 output direction
   */
  void set_int2_dir_out(void);
  /*!
   * @brief Set int1 pin value
   * @param value Interrupt level to be set
   */
  void set_int1(bool value);
  /*!
   * @brief Set int2 pin value
   * @param value Interrupt level to be set
   */
  void set_int2(bool value);
  /*!
   * @brief Enable and attach int1 handler
   * @param handler Interrupt routine to be called
   */
  void enableInterrupt(icux0201_dev_irq_handler handler);
  /*!
   * @brief Disable and detach int1 handler
   */
  void disableInterrupt(void);
  /*!
   * @brief Enable and attach int2 handler
   * @param handler Interrupt routine to be called
   */
  void enableInterrupt2(icux0201_dev_irq_handler handler);
  /*!
   * @brief Disable and detach int2 handler
   */
  void disableInterrupt2(void);
  /*!
   * @brief Set the SPI chip select LOW and begin SPI transaction
   */
   void spi_begin_transaction(void);
  /*!
   * @brief End SPI transaction and set the SPI chip select HIGH
   */
   void spi_end_transaction(void);
  /*!
   * @brief Set the SPI chip select pin level
   * @param data bytes to be sent on SPI
   * @param num_bytes Number of bytes to be sent on SPI
   */
   void spi_write(const uint8_t *data, uint16_t num_bytes);
  /*!
   * @brief Set the SPI chip select pin level
   * @param data bytes read from SPI
   * @param num_bytes Number of bytes to be read on SPI
   */
   void spi_read(uint8_t *data, uint16_t num_bytes);
protected:
  /* Descriptor structure for group of sensors */
  SPIClass* spi;
  uint32_t spi_freq = DEFAULT_SPI_CLOCK;
  int spi_cs_pin_id;
  int int1_pin_id;
  int int2_pin_id;
  int mutclk_pin_id;
  bool int1_attached;
  bool int2_attached;
  bool int1_in_dir;
  bool int2_in_dir;
  ch_fw_init_func_t fw_init_func;
  ch_fw_init_func_t fw_sensor_init_func;
  const uint32_t default_odr_ms = 100; // 1 measure each 100 ms
  void clear_data_ready(void);
  bool is_measure_ready;
  /* Measurement configuration struct - starting/default values */
  const ch_meas_config_t meas_config_init = {
    .odr = CH_ODR_FREQ_DIV_8,
    .meas_period = 0,
  };
  virtual int configure_measure(uint16_t range_mm, ch_mode_t mode=CH_MODE_FREERUN);
};

/* Chirp ICU General Purpose Transceiver */
class ICUX0201_dev_GeneralPurpose : public ICUX0201_dev {
public:
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param freq SPI clock frequency to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   * @param int2_id ID of the interrupt 2 pin
   * @param mutclk_id ID of the MUTCLK pin
   */
   ICUX0201_dev_GeneralPurpose(SPIClass &spi_ref, uint32_t freq, int cs_id,
                          int int1_id, int int2_id, int mutclk_id);
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
  ICUX0201_dev_GeneralPurpose(SPIClass &spi_ref, int cs_id, int int1_id);
  /*!
   * @brief Get the measured range from a sensor.
   * @return Range in millimeters, or \a CH_NO_TARGET (0xFFFFFFFF) if no target was detected,
   *         or 0 if error
   */
   float get_range(void);

protected:
  int configure_measure(uint16_t range_mm=0, ch_mode_t mode=CH_MODE_FREERUN);
};

class ICUX0201_dev_Presence : public ICUX0201_dev {
public:
  /*! ICU20201 : 3.1 m max range with algo presence */
  static const uint16_t icu20201_default_range_mm = 3100;
  /*! ICU30201 : 4.5 m max range with algo presence */
  static const uint16_t icu30201_default_range_mm = 4500;

  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param freq SPI clock frequency to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   * @param int2_id ID of the interrupt 2 pin
   * @param mutclk_id ID of the MUTCLK pin
   */
  ICUX0201_dev_Presence(SPIClass &spi_ref, uint32_t freq, int cs_id,
                    int int1_id, int int2_id, int mutclk_id);
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
  ICUX0201_dev_Presence(SPIClass &spi_ref, int cs_id, int int1_id);
  /*!
   * @brief Starts sensor in free running mode, with default max range and ODR.
   * @return 0 in case of success
   */
  int free_run();
  /*!
   * @brief Starts sensor in free running mode, with provided max range and default ODR.
   * @param range_mm Max detection range to be configured
   * @return 0 in case of success
   */
  int free_run(uint16_t range_mm);
  /*!
   * @brief Starts sensor in free running mode, with provided max range and ODR.
   * @param range_mm Max detection range to be configured
   * @param interval_ms Interval between samples, in milliseconds
   * @return 0 in case of success
   */
  int free_run(uint16_t range_mm, uint16_t interval_ms);
  /*!
   * @brief Get Presence algorithm output.
   * @return true if a moving target was detected, false otherwise
   */
  bool get_presence(void);
  /*!
   * @brief Get Presence algorithm range.
   * @return distance to the target in cm
   */
  uint16_t get_range(void);

protected:
  int configure_measure(uint16_t range_mm, ch_mode_t mode=CH_MODE_FREERUN);
  int init_sensor_algorithm(void);
  int get_algo_output();
  IcuPresenceConfig algo_config;
  IcuPresenceOutput last_algo_output;
};

#endif // ICUX0201_DEV_H
