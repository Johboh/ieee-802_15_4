// Copyright (c) Johan Böhlin
// SPDX-License-Identifier: GPL-2.0-or-later OR GPL-3.0-or-later

#include <Arduino.h>
#include <Ieee802154.h>

// Shared
struct __attribute__((packed)) ApplicationMessage {
  double temperature;
};

Ieee802154 _ieee802154({.channel = 15, .pan_id = 0x9191});

void setup() {
  _ieee802154.initialize();
  char buffer[64];
  sprintf(buffer, "This device IEEE802.15.4 MAC: 0x%llx", _ieee802154.deviceMacAddress());
  Serial.println(buffer);
}

bool data_frame = false;
void loop() {
  // YOU MUST UPDATE THIS TO THE MAC ADDRESS OF YOUR HOST!
  // See printout in your host console for MAC addresses.
  uint64_t destination_address = 0xe4b323fffe926d10;

  if (data_frame) {
    ApplicationMessage message = {
        .temperature = 22.5,
    };
    auto result = _ieee802154.transmit(destination_address, (uint8_t *)&message, sizeof(ApplicationMessage));
    switch (result) {
    case Ieee802154::TransmitResult::Success:
      Serial.println("Transmit OK (attempts: " + String(result.attempts) + ", ACK RSSI: " + String(result.ack_rssi) +
                     ", LQI: " + String(result.ack_lqi) + ")");
      break;
    case Ieee802154::TransmitResult::SuccessWithPending:
      Serial.println("Transmit OK with pending data (attempts: " + String(result.attempts) + ")");
      break;
    case Ieee802154::TransmitResult::Failure:
      Serial.println("Transmit error (attempts: " + String(result.attempts) + ", errors: " + String(result.errors) +
                     ", last_error: " + String(result.last_error) + ")");
      break;
    }
  } else {
    auto result = _ieee802154.dataRequest(destination_address);
    switch (result) {
    case Ieee802154::DataRequestResult::NoDataAvailable:
      Serial.println("Data Request: No data available (attempts: " + String(result.attempts) + ")");
      break;
    case Ieee802154::DataRequestResult::DataAvailable:
      Serial.println("Data Request: Data available (attempts: " + String(result.attempts) + ")");
      break;
    case Ieee802154::DataRequestResult::Failure:
      Serial.println("Data Request: Failure (attempts: " + String(result.attempts) +
                     ", errors: " + String(result.errors) + ", last_error: " + String(result.last_error) + ")");
      break;
    default:
      Serial.println("Data Request: Unknown outcome");
      break;
    }
  }

  delay(2000);
}
