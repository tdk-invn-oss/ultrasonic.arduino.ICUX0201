/*
 *
 * Copyright (c) 2024 by InvenSense, Inc.
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

#define DISTANCE_BETWEEN_SENSORS_MM 100 
// Hardware config : CS on pin 10, INT1 on pin 2
ICUX0201_dev_GeneralPurpose dev0(SPI, 10, 2);
// Hardware config : CS on pin 9, INT1 on pin 3
ICUX0201_dev_GeneralPurpose dev1(SPI, 9, 3);
ICUX0201_GeneralPurpose ICU(dev0,dev1);

void setup() {
  int ret;
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("ICU Triangulation");

  // Initializing the ICU device
  ret = ICU.begin();
  if (ret != 0) {
    Serial.print("ICU initialization failed: ");
    Serial.println(ret);
    while (1)
      ;
  } else {
    ICU.print_informations();
  }
  // Start ICU in trigger mode (using max range)
  ret = ICU.start_trigger();
  if (ret != 0) {
    Serial.print("ICU trig start failed: ");
    Serial.println(ret);
    while (1)
      ;
  } else {
    ICU.print_configuration();
  }
  ICU.trig();
}

void loop() {
  float range;
  /* Wait for new measure done */
  if (ICU.data_ready(0)&&ICU.data_ready(1)) {
    float x,y;
    if(ICU.triangulate(DISTANCE_BETWEEN_SENSORS_MM,x,y)==0)
    {
      Serial.print("X:");
      Serial.println(x);
      Serial.print(" Y:");
      Serial.println(y);
    }
    delay(100);
    /* Trig next measurement */
    ICU.trig();
  }
}
