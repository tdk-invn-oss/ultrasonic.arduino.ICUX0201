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

#define RTC_CAL_PULSE_MS (100)
#define MIN_RANGE_DIFF 150

static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num,
                                ch_interrupt_type_t int_type) {
  if (int_type == CH_INTERRUPT_TYPE_DATA_RDY) {
    ((ICUX0201*)grp_ptr)->get_device(dev_num)->set_data_ready();
  }
}

// ICUX0201 constructor for spi interface
ICUX0201::ICUX0201(SPIClass &spi_ref, uint32_t freq, int cs_id,
                   int int1_id, int int2_id, int mutclk_id) {

   /* Initialize group descriptor */
  ch_group_init(this, 1, 1 , RTC_CAL_PULSE_MS);
  device[0] = new ICUX0201_dev(spi_ref,freq,cs_id,int1_id, int2_id, mutclk_id);
}

// ICUX0201 constructor for spi interface
ICUX0201::ICUX0201(ICUX0201_dev* dev0,ICUX0201_dev* dev1) {
  if(dev0 != NULL)
  {
    if(dev1 == NULL)
    {
      /* Initialize group descriptor */
      ch_group_init(this, 1, 1 , RTC_CAL_PULSE_MS);
      device[0] = dev0;
    } else {
      /* Initialize group descriptor */
      ch_group_init(this, 2, 1 , RTC_CAL_PULSE_MS);
      device[0] = dev0;
      device[1] = dev1;
    }
  }
}

ICUX0201_dev* ICUX0201::get_device(int id)
{
  if (id < num_ports)
  {
    return (ICUX0201_dev*)device[id];
  }
  return NULL;
}

/* Initialize hardware and ICU sensor */
int ICUX0201::begin() {
  uint8_t rc = 0;
  
  board_init(this);

  
  for(int i =0; i< num_ports; i++)
  {
    get_device(i)->begin(this,i);
  }
  
  /* TODO: make the firmware init mutable */
  if (rc == 0) {
    rc = ch_group_start(this);
  }
  if (rc == 0) {
    /* Register callback function to be called when Chirp sensor interrupts */
    ch_io_int_callback_set(this, sensor_int_callback);
  }
  return rc;
}

uint16_t ICUX0201::get_max_range(int sensor_id) {
  return get_device(sensor_id)->get_max_range();
};

uint16_t ICUX0201::get_measure_range(int sensor_id) {
  return get_device(sensor_id)->get_measure_range();
};

int ICUX0201::free_run(void) { return get_device(0)->free_run(); }

int ICUX0201::free_run(uint16_t range_mm) {
  return get_device(0)->free_run(range_mm);
}

int ICUX0201::free_run(uint16_t range_mm, uint16_t interval_ms) {
  return get_device(0)->free_run(range_mm,interval_ms);
}

bool ICUX0201::data_ready(int sensor_id) { return get_device(sensor_id)->data_ready(); }

uint16_t ICUX0201::part_number(int sensor_id) {
  return get_device(sensor_id)->part_number();
}

const char *ICUX0201::sensor_id(int sensor_id) {
  return get_device(sensor_id)->sensor_id();
}

uint32_t ICUX0201::frequency(int sensor_id) { return get_device(sensor_id)->frequency(); }

uint16_t ICUX0201::bandwidth(int sensor_id) { return get_device(sensor_id)->bandwidth(); }

uint16_t ICUX0201::rtc_cal(int sensor_id) {
  return get_device(sensor_id)->rtc_cal();
}

uint16_t ICUX0201::rtc_cal_pulse_length(int sensor_id) {
  return get_device(sensor_id)->rtc_cal_pulse_length();
}

float ICUX0201::cpu_freq(int sensor_id) {
  return get_device(sensor_id)->cpu_freq();
}
const char *ICUX0201::fw_version(int sensor_id) {
  return get_device(sensor_id)->fw_version();
}

void ICUX0201::print_informations(void) {
  for(int i=0; i<num_ports;i++)
  {
    get_device(i)->print_informations();
  }
}

void ICUX0201::print_configuration(void) {
  for(int i=0; i<num_ports;i++)
  {
    get_device(i)->print_configuration();
  }
}

uint8_t ICUX0201::get_iq_data(ch_iq_sample_t (&iq_data)[ICU_MAX_NUM_SAMPLES], uint16_t& nb_samples)
{
  return get_iq_data(0,iq_data,nb_samples);
}

uint8_t ICUX0201::get_iq_data(int sensor_id, ch_iq_sample_t (&iq_data)[ICU_MAX_NUM_SAMPLES], uint16_t& nb_samples)
{
  return get_device(sensor_id)->get_iq_data(iq_data,nb_samples);
}


// ICUX0201 constructor for spi interface
ICUX0201_GeneralPurpose::ICUX0201_GeneralPurpose(ICUX0201_dev_GeneralPurpose& dev0,ICUX0201_dev_GeneralPurpose& dev1) {
  /* Initialize group descriptor */
  ch_group_init(this, 2, 1 , RTC_CAL_PULSE_MS);
  device[0] = &dev0;
  device[1] = &dev1;
}

int ICUX0201_GeneralPurpose::start_trigger(uint16_t range_mm) {
  int rc = 0;
  uint16_t range1_mm = range_mm;
  if(num_ports > 1)
  {
    uint32_t sensors_mean_fop;
    // When using 2 sensors, manage ranges to avoid both sensor to compute algo at the same time
    uint16_t max_range = get_max_range();
    if(range_mm == 0)
    {
      range_mm = max_range - (MIN_RANGE_DIFF);
      range1_mm = max_range;
    } else if( (range_mm + (MIN_RANGE_DIFF)) < max_range )
    {
      range1_mm = range_mm + (MIN_RANGE_DIFF);
    } else  {
      range_mm = max_range - (MIN_RANGE_DIFF);
      range1_mm = max_range;
    }
    sensors_mean_fop = (ch_get_frequency(get_device(0)) + ch_get_frequency(get_device(1)))/2;
    rc |= ch_set_frequency(get_device(0), sensors_mean_fop);
    rc |= ch_set_frequency(get_device(1), sensors_mean_fop);
    rc |= get_device(1)->start_trigger(range1_mm,CH_MODE_TRIGGERED_RX_ONLY);
  }
  rc |= get_device(0)->start_trigger(range_mm,CH_MODE_TRIGGERED_TX_RX);
  return rc;
}

void ICUX0201_GeneralPurpose::trig(void) {
  return ch_group_trigger(this);
}

int ICUX0201_GeneralPurpose::triangulate(const float distance_between_sensors_mm, float& x, float& y, float offset)
{
  int rc = 0;
  float range0_mm = get_range(0);
  float range1_mm = get_range(1);
  float diff_mm;

  if ((range0_mm == 0)||(range1_mm == 0)||(range1_mm<=range0_mm))
  {
    /* One of the sensor losts the target */
    return -1;
  }
  /* Remove transmit distance to the 2nd sensor distance */
  range1_mm -= range0_mm + offset;

  
  diff_mm = (range0_mm > range1_mm) ? (range0_mm - range1_mm) : (range1_mm - range0_mm);
  if(diff_mm > distance_between_sensors_mm)
  {
    /* This is not supposed to happen geometrically */
    return -2;
  }
  x =  (range0_mm*range0_mm - range1_mm*range1_mm) / (2*distance_between_sensors_mm);

  y = distance_between_sensors_mm/2 +x;
  y = (range0_mm-y)*(range0_mm + y);
  if(y>0)
  {
    y = sqrt(y);
  } else {
    y = 0;
    return -3;
  }
  return 0;
}