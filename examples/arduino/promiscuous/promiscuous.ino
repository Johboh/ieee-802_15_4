// Copyright (c) Johan Böhlin
// SPDX-License-Identifier: GPL-2.0-or-later OR GPL-3.0-or-later

#include <Arduino.h>
#include <Ieee802154.h>

Ieee802154 _ieee802154({.channel = 15, .pan_id = 0x9191, .promiscuous_mode = true});

void setup() {
  _ieee802154.receivePromiscuous([](const Ieee802154::PromiscuousFrame &frame) {
    const char *frame_type_names[] = {"Beacon", "Data", "Ack", "Command"};
    const char *type_str = (frame.frame_type < 4) ? frame_type_names[frame.frame_type] : "Unknown";

    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Frame [%s] seq=%u ch=%u rssi=%d lqi=%u", type_str, frame.sequence_number,
             frame.channel, frame.rssi, frame.lqi);
    Serial.println(buffer);
    snprintf(buffer, sizeof(buffer),
             " -- PAN: dst=0x%04x, src=0x%04x | Addr: src=0x%llx, dst=0x%llx | payload size: %u", frame.dst_pan,
             frame.src_pan, (unsigned long long)frame.src_addr, (unsigned long long)frame.dst_addr, frame.payload_size);
    Serial.println(buffer);
  });

  _ieee802154.initialize();
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "This device IEEE802.15.4 MAC: 0x%llx",
           (unsigned long long)_ieee802154.deviceMacAddress());
  Serial.println(buffer);
  Serial.println("Promiscuous sniffer started on channel " + String(_ieee802154.channel()));
}

void loop() {}
