#include <Ieee802154.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LOG_TAG "c6-host"

// Shared
struct __attribute__((packed)) ApplicationMessage {
  double temperature;
};

Ieee802154 _ieee802154({.channel = 15, .pan_id = 0x9191, .handle_broadcasts = false}, [](Ieee802154::Message message) {
  ESP_LOGI(LOG_TAG, "Got Message");
  ESP_LOGI(LOG_TAG, " -- source MAC: 0x%llx", message.source_address);
  ESP_LOGI(LOG_TAG, " -- destination MAC: 0x%llx", message.destination_address);
  ESP_LOGI(LOG_TAG, " -- payload size: %d", message.payload_size);
  ApplicationMessage *app = reinterpret_cast<ApplicationMessage *>(message.payload);
  ESP_LOGI(LOG_TAG, " -- Application.temperature: %f", app->temperature);
});

extern "C" {
void app_main();
}

void app_main(void) {
  _ieee802154.initialize();
  ESP_LOGI(LOG_TAG, "This device IEEE802.15.4 MAC: 0x%llx", _ieee802154.deviceMacAddress());
  _ieee802154.setPending(0x543204fffe017694);

  while (1) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    fflush(stdout);
  }
}
