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

// Hardware config : CS on pin 10, INT1 on pin 2
// Sensor type will be detected at runtime
ICUX0201_GeneralPurpose ICU(SPI, 10, 2);

void setup() {
  int ret;
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("ICUx0201 Raw Data print");

  // Initializing the ICU device
  ret = ICU.begin();
  if (ret != 0) {
    Serial.print("ICUx0201 initialization failed: ");
    Serial.println(ret);
    while (1)
      ;
  } else {
    ICU.print_informations();
  }
  // Start ICU in free run mode (max range, interval 250ms)
  ret = ICU.free_run(ICU.get_max_range(),250);
  if (ret != 0) {
    Serial.print("ICUx0201 free run failed: ");
    Serial.println(ret);
    while (1)
      ;
  } else {
    ICU.print_configuration();
  }
}

void loop() {
  ch_iq_sample_t raw_data[ICU_MAX_NUM_SAMPLES];
  uint16_t nb_samples;

  /* Wait for new measure done */
  if (ICU.data_ready()) {
    /* Get raw data from the sensor */
    ICU.get_iq_data(raw_data, nb_samples);
    Serial.println("ICU Raw Data (clear)");
    for (int count = 0; count < nb_samples; count++) {
      /* output one I/Q pair per line */
      Serial.print("q=");
      Serial.print(raw_data[count].q);
      Serial.print(",i=");
      Serial.println(raw_data[count].i);
    }
  }
}
