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
#include "SPI.h"

#include <stdint.h>
#include "chbsp_chirp.h"

// Chirp SonicLib API definitions
#include <invn/soniclib/chirp_bsp.h>
#include <invn/soniclib/soniclib.h>
#include "ICUX0201.h"

ch_group_t* sensor_group_ptr = NULL;
static bool in_interrupt_routine = false;

static void int1_0_handler(void);
static void int2_0_handler(void);
static void int1_1_handler(void);
static void int2_1_handler(void);
const icux0201_dev_irq_handler* irq_handlers[2][2] = {{int1_0_handler, int2_0_handler},{int1_1_handler, int2_1_handler}};

//void chbsp_print_str(const char *str) {
//  Serial.println(str);
//}

static void int1_handler(void)
{
  ICUX0201* group_ptr = (ICUX0201*)sensor_group_ptr;
  in_interrupt_routine = true;
  for(int i = 0; i < sensor_group_ptr->num_ports; i++)
  {
    ICUX0201_dev* device = group_ptr->get_device(i);
    if(!device->get_int1())
    {
      ch_interrupt(sensor_group_ptr, i);
    }
  }
  in_interrupt_routine = false;
}

static void int2_handler(void)
{
  /* Not implemented */
}
/*
static void int1_handler(int sensor_id)
{
  if(sensor_group_ptr != NULL)
  {
    ICUX0201_dev* device = (ICUX0201_dev*)ch_get_dev_ptr(sensor_group_ptr, sensor_id);
    if(!device->get_int1())
    {
      ch_interrupt(sensor_group_ptr, sensor_id);
      // Data ready handler disables interrupt, make it ready to fire again !
      chbsp_int1_interrupt_enable(ch_get_dev_ptr(sensor_group_ptr, sensor_id));
    }
  }
}

static void int1_0_handler(void)
{
  int1_handler(0);
}
*/
static void int2_0_handler(void)
{
  /* Not implemented */
}
/*
static void int1_1_handler(void)
{
  int1_handler(1);
}
*/
static void int2_1_handler(void)
{
  /* Not implemented */
}

void board_init(ch_group_t *grp_ptr)
{
  /* Make local copy of group pointer */
  sensor_group_ptr = grp_ptr;
}

extern "C" void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->set_int1_dir_out();
}

extern "C" void chbsp_set_int1_dir_in(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->set_int1_dir_in();
}

extern "C" void chbsp_int1_clear(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->set_int1(LOW);
}

extern "C" void chbsp_int1_set(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->set_int1(HIGH);
}

extern "C" void chbsp_int1_interrupt_enable(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->enableInterrupt(int1_handler);
//  device->enableInterrupt(irq_handlers[device->io_index][0]);
}

extern "C" void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr)
{
  /* Some boards (eg: nano ble sense) do not support detachInterrupt while being in the interrupt routine */
    if(!in_interrupt_routine)
    {
      ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
      device->disableInterrupt();
    }
}

extern "C" void chbsp_set_int2_dir_out(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->set_int2_dir_out();
}

extern "C" void chbsp_set_int2_dir_in(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->set_int2_dir_in();
}

extern "C" void chbsp_int2_clear(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->set_int2(LOW);
}

extern "C" void chbsp_int2_set(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->set_int2(HIGH);
}

extern "C" void chbsp_int2_interrupt_enable(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->enableInterrupt2(int2_handler);
}

extern "C" void chbsp_int2_interrupt_disable(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->disableInterrupt2();
}


extern "C" void chbsp_spi_cs_on(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->spi_begin_transaction();
}

extern "C" void chbsp_spi_cs_off(ch_dev_t *dev_ptr)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->spi_end_transaction();
}

extern "C" int chbsp_spi_write(ch_dev_t *dev_ptr, const uint8_t *data, uint16_t num_bytes)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->spi_write(data,num_bytes);
  return 0;
}

extern "C" int chbsp_spi_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes)
{
  ICUX0201_dev* device = (ICUX0201_dev*)dev_ptr;
  device->spi_read(data,num_bytes);
  return 0;
}

extern "C" void chbsp_group_set_int1_dir_out(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_set_int1_dir_out(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_set_int1_dir_in(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_set_int1_dir_in(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_int1_clear(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_int1_clear(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_int1_set(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_int1_set(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_int1_interrupt_enable(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_int1_interrupt_enable(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_group_int1_interrupt_disable(ch_group_t *grp_ptr)
{
  for(int i = 0; i < grp_ptr->num_ports; i++)
  {
    chbsp_int1_interrupt_disable(ch_get_dev_ptr(grp_ptr, i));
  }
}

extern "C" void chbsp_delay_us(uint32_t us)
{
    delayMicroseconds(us);
}

extern "C" void chbsp_delay_ms(uint32_t ms)
{
  delay(ms);
}

extern "C" uint32_t chbsp_timestamp_ms(void)
{
  return (uint32_t) millis();
}

void chbsp_event_wait_setup(uint32_t event_mask) {
  ICUX0201* group_ptr = (ICUX0201*)sensor_group_ptr;

  if(group_ptr != NULL)
    group_ptr->reset_triggered_interrupts(event_mask); 
}

void chbsp_event_notify(uint32_t event_mask) {
  ICUX0201* group_ptr = (ICUX0201*)sensor_group_ptr;
  if(group_ptr != NULL)
    group_ptr->set_triggered_interrupts(event_mask);
}


uint8_t chbsp_event_wait(uint16_t time_out_ms, uint32_t event_mask) {
  ICUX0201* group_ptr = (ICUX0201*)sensor_group_ptr;
	uint8_t err         = 0;
	uint32_t start_time = chbsp_timestamp_ms();

  if(group_ptr == NULL)
    return -1;

	/* Wait for sensor to interrupt */
	while (!err && ((group_ptr->get_triggered_interrupts() & event_mask) == 0)) {
		uint32_t new_time = chbsp_timestamp_ms();
		if (new_time >= (start_time + time_out_ms)) {
			err = 1;                         // timeout
			                                 // break;
		} else if (new_time < start_time) {  // if rollover
			start_time = new_time;           //  just re-start timeout
		}
	}

  /* Disable interrupts outside interrupt routine */
  for(int i = 0; i < 4; i++)
  {
    if(event_mask & (1<<i))
    {
      group_ptr->get_device(i)->disableInterrupt(); 
    }
  }

	return err;
}

