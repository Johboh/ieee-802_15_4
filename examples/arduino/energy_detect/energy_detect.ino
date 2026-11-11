// Copyright (c) Johan Böhlin
// SPDX-License-Identifier: GPL-2.0-or-later OR GPL-3.0-or-later

#include <Arduino.h>
#include <Ieee802154.h>

Ieee802154 _ieee802154({.channel = 15, .pan_id = 0x9191});

void setup() {
  _ieee802154.initialize();
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "This device IEEE802.15.4 MAC: 0x%llx",
           (unsigned long long)_ieee802154.deviceMacAddress());
  Serial.println(buffer);
}

void loop() {
  Serial.println("--- Scanning channels 11-26 ---");
  for (uint8_t ch = 11; ch <= 26; ++ch) {
    _ieee802154.setChannel(ch);
    int8_t power = 0;
    if (_ieee802154.energyDetect(power)) {
      char buf[32];
      snprintf(buf, sizeof(buf), "Channel %2u: %4d dBm", ch, power);
      Serial.println(buf);
    } else {
      Serial.println("Channel " + String(ch) + ": measurement failed");
    }
  }
  delay(2000);
}
