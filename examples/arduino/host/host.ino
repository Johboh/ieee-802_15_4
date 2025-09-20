#include <Arduino.h>
#include <Ieee802154.h>

// Shared
struct __attribute__((packed)) ApplicationMessage {
  double temperature;
};

char buffer[64];

Ieee802154 _ieee802154({.channel = 15, .pan_id = 0x9191}, [](Ieee802154::Message message) {
  Serial.println("Got Message");
  sprintf(buffer, "source MAC: 0x%llx", message.source_address);
  Serial.println(buffer);
  sprintf(buffer, "destination MAC: 0x%llx", message.destination_address);
  Serial.println(" -- payload size: " + String(message.payload_size));
  ApplicationMessage *app = reinterpret_cast<ApplicationMessage *>(message.payload);
  Serial.println(" -- Application.temperature: " + String(app->temperature));
});

void setup() {
  _ieee802154.initialize();
  sprintf(buffer, "This device IEEE802.15.4 MAC: 0x%llx", _ieee802154.deviceMacAddress());
  Serial.println(buffer);
}

void loop() {}
