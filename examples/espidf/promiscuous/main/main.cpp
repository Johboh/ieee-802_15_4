// Copyright (c) Johan Böhlin
// SPDX-License-Identifier: GPL-2.0-or-later OR GPL-3.0-or-later

#include <Ieee802154.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LOG_TAG "c6-promiscuous"

Ieee802154 _ieee802154({.channel = 15, .pan_id = 0x9191, .promiscuous_mode = true});

extern "C" {
void app_main();
}

void app_main(void) {
  _ieee802154.receivePromiscuous([](const Ieee802154::PromiscuousFrame &frame) {
    const char *frame_type_names[] = {"Beacon", "Data", "Ack", "Command"};
    const char *type_str = (frame.frame_type < 4) ? frame_type_names[frame.frame_type] : "Unknown";

    ESP_LOGI(LOG_TAG, "Frame [%s] seq=%u ch=%u rssi=%d lqi=%u", type_str, frame.sequence_number, frame.channel,
             frame.rssi, frame.lqi);
    ESP_LOGI(LOG_TAG, " -- PAN: dst=0x%04x, src=0x%04x | Addr: src=0x%llx, dst=0x%llx | payload size: %u",
             frame.dst_pan, frame.src_pan, (unsigned long long)frame.src_addr, (unsigned long long)frame.dst_addr,
             frame.payload_size);
  });

  _ieee802154.initialize();
  ESP_LOGI(LOG_TAG, "Promiscuous sniffer started on channel %u", _ieee802154.channel());
  ESP_LOGI(LOG_TAG, "This device IEEE802.15.4 MAC: 0x%llx", (unsigned long long)_ieee802154.deviceMacAddress());

  while (1) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    fflush(stdout);
  }
}
