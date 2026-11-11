// Copyright (c) Johan Böhlin
// SPDX-License-Identifier: GPL-2.0-or-later OR GPL-3.0-or-later

#include <Ieee802154.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LOG_TAG "c6-energy-detect"

Ieee802154 _ieee802154({.channel = 15, .pan_id = 0x9191});

void scanTask(void *pvParameters) {
  while (1) {
    ESP_LOGI(LOG_TAG, "--- Scanning channels 11-26 ---");
    for (uint8_t ch = 11; ch <= 26; ++ch) {
      _ieee802154.setChannel(ch);
      int8_t power = 0;
      if (_ieee802154.energyDetect(power)) {
        ESP_LOGI(LOG_TAG, "Channel %2u: %4d dBm", ch, power);
      } else {
        ESP_LOGW(LOG_TAG, "Channel %2u: measurement failed", ch);
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

extern "C" {
void app_main();
}

void app_main(void) {
  _ieee802154.initialize();
  ESP_LOGI(LOG_TAG, "This device IEEE802.15.4 MAC: 0x%llx", (unsigned long long)_ieee802154.deviceMacAddress());

  xTaskCreate(scanTask, "scanTask", 4096, NULL, 5, NULL);

  while (1) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    fflush(stdout);
  }
}
