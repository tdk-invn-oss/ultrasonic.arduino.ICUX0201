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
#include "ICUX0201.h"

// Instantiate an ICU20201 using Presence Detection
// Hardware config : CS on pin 10, INT1 on pin 2
ICUX0201_Presence ICU(SPI, 10, 2);

void setup() {
  int ret;

  Serial.begin(115200);
  while(!Serial) {}

  Serial.println("Presence detection Example");

  // Initializing the ICU20201
  ret = ICU.begin();
  if (ret != 0) {
    Serial.print("ICU20201 initialization failed: ");
    Serial.println(ret);
    while(1);
  } else {
    ICU.print_informations();
  }
  // Start ICU in free run mode
  ret = ICU.free_run();
  if (ret != 0) {
    Serial.print("ICU20201 measure start failed: ");
    Serial.println(ret);
    while(1);
  } else {
    ICU.print_configuration();
    Serial.println("ICU20201 Starting measures: ");
  }
}

void loop() {
  bool presence;
  uint16_t range;

  if (ICU.data_ready()) {
    presence = ICU.get_presence();
    range = ICU.get_range();

    Serial.print("Presence:");
    Serial.print(presence);
    Serial.print(" ");
    Serial.print("Range(cm):");
    Serial.println(range);

  }
}
