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
    bool result = _ieee802154.transmit(destination_address, (uint8_t *)&message, sizeof(ApplicationMessage));
    if (!result) {
      Serial.println("Transmit error");
    } else {
      Serial.println("Transmit OK");
    }
  } else {
    auto result = _ieee802154.dataRequest(destination_address);
    switch (result) {
    case Ieee802154::DataRequestResult::NoDataAvailable:
      Serial.println("Data Request: No data available");
      break;
    case Ieee802154::DataRequestResult::DataAvailble:
      Serial.println("Data Request: Data available");
      break;
    case Ieee802154::DataRequestResult::Failure:
      Serial.println("Data Request: Failure");
      break;
    default:
      Serial.println("Data Request: Unknown outcome");
      break;
    }
  }

  delay(2000);
}
