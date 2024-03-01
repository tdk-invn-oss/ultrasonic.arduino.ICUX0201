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
//#include "wiring_private.h" // pinPeripheral() function
#include <stdint.h>
#include "chbsp_chirp.h"

// Chirp SonicLib API definitions
#include <invn/soniclib/chirp_bsp.h>
#include <invn/soniclib/soniclib.h>

#define CHBSP_RTC_CAL_PULSE_MS       (100)

#define SPI_CLOCK 11000000
static SPIClass *spi = NULL;

static uint8_t cs_pin_id;
static uint8_t int1_pin_id;
static uint8_t int2_pin_id;
static uint8_t mutclk_pin_id;
static ch_group_t *sensor_group_ptr = NULL;

static bool int1_attached = false;
static bool int1_in_dir = false;

static void int1_handler(void);
static void int2_handler(void);

static void sensors_pin_init(ch_dev_t *dev_ptr)
{
  /* Configure INT pins as input */
  chbsp_set_int1_dir_in(dev_ptr);

  /* Enable pull-ups on the INT pins */
  pinMode(int1_pin_id, INPUT_PULLUP);

  /* Configure MUTCLK as output with high level (not used by ICU sensors as clk) */
  if(mutclk_pin_id != UNUSED_PIN)
  {
    pinMode(mutclk_pin_id, OUTPUT);
    digitalWrite(mutclk_pin_id,HIGH);
  }
  if(int2_pin_id != UNUSED_PIN)
    pinMode(int2_pin_id,INPUT_PULLUP);

}

static void spi_init(void)
{
  spi->begin();
  pinMode(cs_pin_id,OUTPUT);
  digitalWrite(cs_pin_id,HIGH);

}

static void int1_handler(void)
{
  if(sensor_group_ptr != NULL)
  {
    ch_interrupt(sensor_group_ptr, 0);
    // Data ready handler disables interrupt, make it ready to fire again !
    chbsp_int1_interrupt_enable(ch_get_dev_ptr(sensor_group_ptr, 0));
  }
}

static void int2_handler(void)
{
  /* Not implemented */
}

void chbsp_module_init(SPIClass &spi_ref, uint8_t cs_id,uint8_t int1_id, uint8_t int2_id, uint8_t mutclk_id)
{
  spi = &spi_ref;
  cs_pin_id = cs_id;
  int1_pin_id = int1_id;
  int2_pin_id = int2_id;
  mutclk_pin_id = mutclk_id;
}

void chbsp_board_init(ch_group_t *grp_ptr)
{
  /* Make local copy of group pointer */
  sensor_group_ptr = grp_ptr;

  /* Initialize group descriptor */
  grp_ptr->num_ports = 1;
  grp_ptr->num_buses = 1;
  grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;
  grp_ptr->disco_hook = NULL;

  spi_init();

  sensors_pin_init(grp_ptr->device[0]);
}

void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr)
{
  if(int1_in_dir)
  {
    pinMode(int1_pin_id,OUTPUT);
    int1_in_dir = false;
  }
}

void chbsp_set_int1_dir_in(ch_dev_t *dev_ptr)
{
  if(!int1_in_dir)
  {
    pinMode(int1_pin_id,INPUT_PULLUP);
    int1_in_dir = true;
  }
}

void chbsp_int1_clear(ch_dev_t *dev_ptr)
{
    digitalWrite(int1_pin_id,LOW);
}

void chbsp_int1_set(ch_dev_t *dev_ptr)
{
    digitalWrite(int1_pin_id,HIGH);
}


void chbsp_int1_interrupt_enable(ch_dev_t *dev_ptr)
{
  if (!int1_attached) {
    chbsp_set_int1_dir_in(dev_ptr);

    /* Warning :
     * pio_get_interrupt_status() clear all interrupts from port
     * In hw trigger mode we enable interrupts just after triggering sensor so we shouldn't miss another interrupt
     * In freerun we shall enable interrupts at init and don't change pins after so we shouln't miss interrupts
     */
    attachInterrupt(digitalPinToInterrupt(int1_pin_id),int1_handler,FALLING);
    int1_attached = true;
  }
}

void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr)
{
  if (int1_attached) {
    detachInterrupt(digitalPinToInterrupt(int1_pin_id));
    int1_attached = false;
  }
}

void chbsp_set_int2_dir_out(ch_dev_t __attribute__((unused)) *dev_ptr)
{
  pinMode(int2_pin_id,OUTPUT);
}

void chbsp_set_int2_dir_in(ch_dev_t __attribute__((unused)) *dev_ptr)
{
  pinMode(int2_pin_id,INPUT);
}

void chbsp_int2_clear(ch_dev_t __attribute__((unused)) *dev_ptr)
{
    digitalWrite(int2_pin_id,LOW);
}

void chbsp_int2_set(ch_dev_t __attribute__((unused)) *dev_ptr)
{
    digitalWrite(int2_pin_id,HIGH);
}

void chbsp_int2_interrupt_enable(ch_dev_t __attribute__((unused)) *dev_ptr)
{
  /* Warning : enable interrupt AND set pin as input */
  chbsp_set_int2_dir_in(dev_ptr);
  attachInterrupt(digitalPinToInterrupt(int2_pin_id),int2_handler,FALLING);
}

void chbsp_int2_interrupt_disable(ch_dev_t __attribute__((unused)) *dev_ptr)
{
  detachInterrupt(digitalPinToInterrupt(int2_pin_id));
}


void chbsp_spi_cs_on(ch_dev_t *dev_ptr)
{
  noInterrupts();
  spi->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_pin_id,LOW);
}

void chbsp_spi_cs_off(ch_dev_t *dev_ptr)
{
  spi->endTransaction();
  digitalWrite(cs_pin_id,HIGH);
  interrupts();
}

int chbsp_spi_write(ch_dev_t __attribute__((unused)) *dev_ptr, uint8_t *data, uint16_t num_bytes)
{
  for(uint16_t i = 0; i < num_bytes; i++) {
    spi->transfer(data[i]);
  }
  return 0;
}

int chbsp_spi_read(ch_dev_t __attribute__((unused)) *dev_ptr, uint8_t *data, uint16_t num_bytes)
{
  spi->transfer(data,num_bytes);
  return 0;
}

void chbsp_delay_us(uint32_t us)
{
    delayMicroseconds(us);
}

void chbsp_delay_ms(uint32_t ms)
{
  delay(ms);
}

void chbsp_group_set_int1_dir_out(ch_group_t *grp_ptr)
{
  chbsp_set_int1_dir_out(ch_get_dev_ptr(grp_ptr, 0));
}

void chbsp_group_set_int1_dir_in(ch_group_t *grp_ptr)
{
  chbsp_set_int1_dir_in(ch_get_dev_ptr(grp_ptr, 0));
}

void chbsp_group_int1_clear(ch_group_t *grp_ptr)
{
  chbsp_int1_clear(ch_get_dev_ptr(grp_ptr, 0));
}

void chbsp_group_int1_set(ch_group_t *grp_ptr)
{
  chbsp_int1_set(ch_get_dev_ptr(grp_ptr, 0));
}
void chbsp_group_int1_interrupt_enable(ch_group_t *grp_ptr)
{
  chbsp_int1_interrupt_enable(ch_get_dev_ptr(grp_ptr, 0));
}
void chbsp_group_int1_interrupt_disable(ch_group_t *grp_ptr)
{
  chbsp_int1_interrupt_disable(ch_get_dev_ptr(grp_ptr, 0));
}

uint32_t chbsp_timestamp_ms(void)
{
  return (uint32_t) millis();
}