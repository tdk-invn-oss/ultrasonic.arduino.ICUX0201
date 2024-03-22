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

#ifndef ICUX0201_H
#define ICUX0201_H

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

class ICUX0201 {
public:
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param freq SPI clock frequency to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
  ICUX0201(SPIClass &spi_ref, uint32_t freq, uint8_t cs_id, uint8_t int1_id,
           uint8_t int2_id, uint8_t mutclk_id);
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   * @param int2_id ID of the interrupt 2 pin
   */
  ICUX0201(SPIClass &spi_ref, uint8_t cs_id, uint8_t int1_id);
  /*!
   * @brief Configure the sensor
   * @return 0 if successful
   */
   int begin();
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
   * @brief Displays sensor information.
   * @param serial the serial object to be used to print
   */
  void print_informations(HardwareSerial &serial);
  /*!
   * @brief Displays sensor configuration.
   * @param serial the serial object to be used to print
   */
  void print_configuration(HardwareSerial &serial);
  /*!
   * @brief Get the raw I/Q measurement data from a sensor.
   * @param iq_data pointer to data buffer where I/Q data will be written (length = ICU_MAX_NUM_SAMPLES)
   * @param num_samples number of samples read from sensor
   * @return 0 if successful, 1 if error
   */
  uint8_t get_iq_data(ch_iq_sample_t (&iq_data)[ICU_MAX_NUM_SAMPLES], uint16_t& nb_samples);

protected:
  /* Descriptor structure for group of sensors */
  ch_group_t chirp_group;
  ch_dev_t chirp_device;
  ch_fw_init_func_t fw_init_func;
  ch_fw_init_func_t fw_sensor_init_func;
  static const uint32_t default_odr_ms = 100; // 1 measure each 100 ms

  virtual int configure_measure(uint16_t range_mm = 0);
  void clear_data_ready();
};

/* Chirp ICU General Purpose Transceiver */
class ICUX0201_GeneralPurpose : public ICUX0201 {
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
   ICUX0201_GeneralPurpose(SPIClass &spi_ref, uint32_t freq, uint8_t cs_id,
                          uint8_t int1_id, uint8_t int2_id, uint8_t mutclk_id);
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
  ICUX0201_GeneralPurpose(SPIClass &spi_ref, uint8_t cs_id, uint8_t int1_id);
  /*!
   * @brief Get the measured range from a sensor.
   * @return Range in millimeters, or \a CH_NO_TARGET (0xFFFFFFFF) if no target was detected,
   *         or 0 if error
   */
   float get_range(void);

protected:
  int configure_measure(uint16_t range_mm = 0);
};

class ICUX0201_Presence : public ICUX0201 {
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
  ICUX0201_Presence(SPIClass &spi_ref, uint32_t freq, uint8_t cs_id,
                    uint8_t int1_id, uint8_t int2_id, uint8_t mutclk_id);
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
  ICUX0201_Presence(SPIClass &spi_ref, uint8_t cs_id, uint8_t int1_id);
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
  int configure_measure(uint16_t range_mm);
  int init_sensor_algorithm(void);
  int get_algo_output();
  IcuPresenceConfig algo_config;
  IcuPresenceOutput last_algo_output;
};

#endif // ICUX0201_H
