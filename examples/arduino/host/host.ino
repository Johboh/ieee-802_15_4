#include <Arduino.h>
#include <Ieee802154.h>

// Shared
struct __attribute__((packed)) ApplicationMessage {
  double temperature;
};

Ieee802154 _ieee802154({.handle_broadcasts = false}, [](Ieee802154::Message message) {
  Serial.println("Got Message");
  Serial.println(" -- source MAC: 0x%llx", message.source_address);
  Serial.println(" -- destination MAC: 0x%llx", message.destination_address);
  Serial.println(" -- payload size: %d", message.payload_size);
  ApplicationMessage *app = reinterpret_cast<ApplicationMessage *>(message.payload);
  Serial.println(" -- Application.temperature: " + String(app->temperature));
});

void setup() {
  _ieee802154.initialize();
  char buffer[64];
  sprintf(buffer, "This device IEEE802.15.4 MAC: 0x%llx", _ieee802154.deviceMacAddress());
  Serial.println(buffer);

  _ieee802154.setPending(0x543204fffe017694);
}

void loop() {}
