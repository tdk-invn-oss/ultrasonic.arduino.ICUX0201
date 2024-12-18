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
#include "ICUX0201_dev.h"

#include "invn/soniclib/soniclib.h"
#ifdef __cplusplus
extern "C" {
#endif

#include <invn/soniclib/sensor_fw/icu_presence/icu_presence.h>

#ifdef __cplusplus
}
#endif

class ICUX0201 : public ch_group_t {
public:
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param freq SPI clock frequency to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
  ICUX0201(SPIClass &spi_ref, uint32_t freq, int cs_id, int int1_id,
           int int2_id=UNUSED_PIN, int mutclk_id=UNUSED_PIN);
  /*!
   * @brief Class constructor.
   * @param dev0 First device of the group
   * @param dev1 Second device of the group
   */
  ICUX0201(ICUX0201_dev* dev0=NULL,ICUX0201_dev* dev1=NULL);
  /*!
   * @brief Get a device from the group
   * @param id Id of the device to get
   * @return A pointer to the device defined by id
   */
   ICUX0201_dev* get_device(int id);
  /*!
   * @brief Configure the sensor
   * @return 0 if successful
   */
   int begin();
  /*!
   * @brief Get the sensor part number.
   * @param id Id of the device in the group
   * @return Sensor part number as an integer
   */
  uint16_t part_number(int sensor_id=0);
  /*!
   * @brief Get the sensor id.
   * @param id Id of the device in the group
   * @return Returns sensor ID as a char pointer
   */
  const char *sensor_id(int sensor_id=0);
  /*!
   * @brief Get sensor's operating frequency.
   * @param id Id of the device in the group
   * @return Acoustic operating frequency, in Hz
   */
  uint32_t frequency(int sensor_id=0);
  /*!
   * @brief Get sensor's bandwidth.
   * @param id Id of the device in the group
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t bandwidth(int sensor_id=0);
  /*!
   * @brief Get the real-time clock calibration value.
   * @param id Id of the device in the group
   * @return Sensor bandwidth, in Hz, or 0 if error or bandwidth measurement is not available
   */
  uint16_t rtc_cal(int sensor_id=0);
  /*!
   * @brief Get the real-time clock calibration pulse length.
   * @param id Id of the device in the group
   * @return RTC pulse length, in ms
   */
  uint16_t rtc_cal_pulse_length(int sensor_id=0);
  /*!
   * @brief Get the sensor CPU frequency, in Hz.
   * @param id Id of the device in the group
   * @return sensor CPU frequency, in Hz
   */
  float cpu_freq(int sensor_id=0);
  /*!
   * @brief Get the firmware version description string.
   * @param id Id of the device in the group
   * @return Pointer to character string describing sensor firmware version
   */
  const char *fw_version(int sensor_id=0);
  /*!
   * @brief Check if a measure is available.
   * @param id Id of the device in the group
   * @return True if a measure is available, false otherwise
   */
  bool data_ready(int sensor_id=0);
  /*!
   * @brief Get the sensor's max range.
   * @param id Id of the device in the group
   * @return Sensor's max range in mm
   * This value depends on sensor internal sample buffer size
   */
  uint16_t get_max_range(int sensor_id=0);
  /*!
   * @brief Get the sensor's range configured for measures.
   * @param id Id of the device in the group
   * @return Sensor's measure range in mm
   * This value depends on sensor configured range
   */
  uint16_t get_measure_range(int sensor_id=0);
  /*!
   * @brief Starts sensor 0 in free running mode, with maximum range and default ODR.
   * @return 0 in case of success
   */
  virtual int free_run(void);
  /*!
   * @brief Starts sensor 0 in free running mode, with provided max range and default ODR.
   * @param range_mm Max detection range to be configured
   * @return 0 in case of success
   */
  virtual int free_run(uint16_t range_mm);
  /*!
   * @brief Starts sensor 0 in free running mode, with provided max range and ODR.
   * @param range_mm Max detection range to be configured
   * @param interval_ms Interval between samples, in milliseconds
   * @return 0 in case of success
   */
  virtual int free_run(uint16_t range_mm, uint16_t interval_ms);
  /*!
   * @brief Displays sensor informations.
   */
  void print_informations(void);
  /*!
   * @brief Displays sensor configurations.
   */
  void print_configuration(void);
  /*!
   * @brief Get the raw I/Q measurement data from sensor 0.
   * @param iq_data pointer to data buffer where I/Q data will be written (length = ICU_MAX_NUM_SAMPLES)
   * @param num_samples number of samples read from sensor
   * @return 0 if successful, 1 if error
   */
  uint8_t get_iq_data(ch_iq_sample_t (&iq_data)[ICU_MAX_NUM_SAMPLES], uint16_t& nb_samples);
  /*!
   * @brief Get the raw I/Q measurement data from a sensor.
   * @param id Id of the device in the group
   * @param iq_data pointer to data buffer where I/Q data will be written (length = ICU_MAX_NUM_SAMPLES)
   * @param num_samples number of samples read from sensor
   * @return 0 if successful, 1 if error
   */
  uint8_t get_iq_data(int sensor_id, ch_iq_sample_t (&iq_data)[ICU_MAX_NUM_SAMPLES], uint16_t& nb_samples);
  /*!
   * @brief Notify that some sensor iinterrupts have triggered.
   * @param mask Mask of devices corresponding to triggered interrupts
   */
  void set_triggered_interrupts(uint32_t mask);
  /*!
   * @brief Notify that some sensor iinterrupts have triggered.
   * @param mask Mask of devices corresponding to triggered interrupts to be reset
   */
   void reset_triggered_interrupts(uint32_t mask);
  /*!
   * @brief Get sensor iiterrupts that have triggered.
   * @return Mask of devices corresponding to triggered interrupts
   */
   uint32_t get_triggered_interrupts(void);

protected:
  uint32_t triggered_interrupts;
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
   ICUX0201_GeneralPurpose(SPIClass &spi_ref, uint32_t freq, int cs_id,
                          int int1_id, int int2_id=UNUSED_PIN, int mutclk_id=UNUSED_PIN) : ICUX0201(new ICUX0201_dev_GeneralPurpose(spi_ref, freq, cs_id, int1_id, int2_id, mutclk_id)) {};
  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
   ICUX0201_GeneralPurpose(SPIClass &spi_ref, int cs_id,
                          int int1_id) : ICUX0201(new ICUX0201_dev_GeneralPurpose(spi_ref, cs_id, int1_id)) {};
  /*!
   * @brief Class constructor.
   * @param dev0 First device of the group
   * @param dev1 Second device of the group
   */
  ICUX0201_GeneralPurpose(ICUX0201_dev_GeneralPurpose& dev0,ICUX0201_dev_GeneralPurpose& dev1);
  /*!
   * @brief Starts sensor in trigger mode, with provided max range.
   * @param range_mm Max detection range to be configured
   * @return 0 in case of success
   */
  int start_trigger(uint16_t range_mm=0);
  /*!
   * @brief Trig sensor measurement.
   */
  void trig(void);
  /*!
   * @brief Get the measured range from a sensor.
   * @param id Id of the device in the group
   * @return Range in millimeters, or \a CH_NO_TARGET (0xFFFFFFFF) if no target was detected,
   *         or 0 if error
   */
   float get_range(int sensor_id = 0) { return ((ICUX0201_dev_GeneralPurpose*)get_device(sensor_id))->get_range(); };
  /*!
   * @brief Triangulate an object using 2 sensors.
   * @param distance_between_sensors_mm The distance between the 2 sensors in mm
   * @param x Output object abscissa
   * @param y Output object ordinate
   * @param offset 2nd sensor offset in mm 
   * @return 0 in case of success
   */
  int triangulate(const float distance_between_sensors_mm, float& x, float& y, float offset=0);
};

class ICUX0201_Presence : public ICUX0201 {
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
  ICUX0201_Presence(SPIClass &spi_ref, uint32_t freq, int cs_id,
                    int int1_id, int int2_id=UNUSED_PIN, int mutclk_id=UNUSED_PIN) : ICUX0201(new ICUX0201_dev_Presence(spi_ref, freq, cs_id, int1_id, int2_id, mutclk_id)) {};

  /*!
   * @brief Class constructor.
   * @param spi_ref Reference of the SPI to be used
   * @param cs_id Pin ID to be used for the SPI
   * @param int1_id ID of the interrupt 1 pin
   */
  ICUX0201_Presence(SPIClass &spi_ref, int cs_id,
                    int int1_id) : ICUX0201(new ICUX0201_dev_Presence(spi_ref, cs_id, int1_id)) {};
  /*!
   * @brief Get Presence algorithm output.
   * @param id Id of the device in the group
   * @return true if a moving target was detected, false otherwise
   */
  bool get_presence(int sensor_id = 0) { return ((ICUX0201_dev_Presence*)get_device(sensor_id))->get_presence(); };
  /*!
   * @brief Get Presence algorithm range.
   * @param id Id of the device in the group
   * @return distance to the target in cm
   */
  uint16_t get_range(int sensor_id = 0) { return ((ICUX0201_dev_Presence*)get_device(sensor_id))->get_range(); };
};

#endif // ICUX0201_H
