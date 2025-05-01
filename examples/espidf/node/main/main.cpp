#include <Ieee802154.h>
#include <cstring>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LOG_TAG "c6-node"

// Shared
struct __attribute__((packed)) ApplicationMessage {
  double temperature;
};

Ieee802154 _ieee802154({.channel = 15, .pan_id = 0x9191});

void transmitTask(void *pvParameters) {
  bool data_frame = false;
  while (1) {
    uint64_t destination_address = 0xe4b323fffe926d10;

    if (data_frame) {
      ApplicationMessage message = {
          .temperature = 22.5,
      };
      bool result = _ieee802154.transmit(destination_address, (uint8_t *)&message, sizeof(ApplicationMessage));
      if (!result) {
        ESP_LOGE(LOG_TAG, "Transmit error");
      } else {
        ESP_LOGI(LOG_TAG, "Transmit OK");
      }
    } else {
      auto result = _ieee802154.dataRequest(destination_address);
      switch (result) {
      case Ieee802154::DataRequestResult::NoDataAvailable:
        ESP_LOGI(LOG_TAG, "Data Request: No data available");
        break;
      case Ieee802154::DataRequestResult::DataAvailble:
        ESP_LOGI(LOG_TAG, "Data Request: Data available");
        break;
      case Ieee802154::DataRequestResult::Failure:
        ESP_LOGE(LOG_TAG, "Data Request: Failure");
        break;
      default:
        ESP_LOGE(LOG_TAG, "Data Request: Unknown outcome");
        break;
      }
    }

    data_frame = !data_frame;
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

extern "C" {
void app_main();
}

void app_main(void) {
  _ieee802154.initialize();
  ESP_LOGI(LOG_TAG, "This device IEEE802.15.4 MAC: 0x%llx", _ieee802154.deviceMacAddress());

  xTaskCreate(transmitTask, "transmitTask", 8192, NULL, 15, NULL);

  while (1) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    fflush(stdout);
  }
}
