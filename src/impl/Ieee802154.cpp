#include "Ieee802154.h"
#include <algorithm>
#include <cstring>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <nvs.h>
#include <nvs_flash.h>

// Callback available from 5.5+, 5.4.1+, 5.3.3+, 5.2.4+, 5.1.6+
#if ESP_IDF_VERSION_MAJOR == 5 and                                                                                     \
    ((ESP_IDF_VERSION_MINOR == 1 and ESP_IDF_VERSION_PATCH >= 6) or                                                    \
     (ESP_IDF_VERSION_MINOR == 2 and ESP_IDF_VERSION_PATCH >= 4) or                                                    \
     (ESP_IDF_VERSION_MINOR == 3 and ESP_IDF_VERSION_PATCH >= 3) or                                                    \
     (ESP_IDF_VERSION_MINOR == 4 and ESP_IDF_VERSION_PATCH >= 1) or ESP_IDF_VERSION_MINOR >= 5)
#define USE_CALLBACKS 1
#endif

using namespace Ieee802154Internal;

static QueueHandle_t __ieee802154_receive_queue = xQueueCreate(10, sizeof(ReceivedFrame));
static QueueHandle_t __ieee802154_transmit_queue = xQueueCreate(10, sizeof(TransmitResult));

#ifdef USE_CALLBACKS
void esp_ieee802154_transmit_done(const uint8_t *frame, const uint8_t *ack,
                                  esp_ieee802154_frame_info_t *ack_frame_info) {
  Ieee802154::ieee802154_transmit_done_cb(frame, ack, ack_frame_info);
}

void esp_ieee802154_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info) {
  Ieee802154::ieee802154_receive_done_cb(frame, frame_info);
}

void esp_ieee802154_transmit_failed(const uint8_t *frame, esp_ieee802154_tx_error_t error) {
  Ieee802154::ieee802154_transmit_failed_cb(frame, error);
}
#endif

void IRAM_ATTR Ieee802154::ieee802154_receive_done_cb(uint8_t *raw_frame, esp_ieee802154_frame_info_t *frame_info) {
  ReceivedFrame received_frame;
  received_frame.frame_info = *frame_info;

  if (frame_info->process) {
    parseFrame(raw_frame, &received_frame);
  }

  esp_ieee802154_receive_handle_done(raw_frame);

  auto xHigherPriorityTaskWoken = pdFALSE;
  auto result = xQueueSendFromISR(__ieee802154_receive_queue, &received_frame, &xHigherPriorityTaskWoken);
  if (result != pdFAIL && xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR Ieee802154::ieee802154_receive_sfd_done_cb(void) {}

void IRAM_ATTR Ieee802154::ieee802154_transmit_done_cb(const uint8_t *frame, const uint8_t *ack,
                                                       esp_ieee802154_frame_info_t *ack_frame_info) {
  TransmitResult transmit_result;
  transmit_result.pending = false;
  transmit_result.error = ESP_IEEE802154_TX_ERR_NONE;

  // For some reason, we cannot trust ack_frame_info when trying to figure out if pending bit is set,
  // as it will never be set in ack_frame_info.
  // Instead, we need to parse the frame and look at frame_pending bit in the fcf.
  if (ack != nullptr) {
    ReceivedFrame received_frame;
    parseFrame((uint8_t *)ack, &received_frame);
    transmit_result.pending = received_frame.fcf.frame_pending;
    esp_ieee802154_receive_handle_done(ack);
  }

  auto xHigherPriorityTaskWoken = pdFALSE;
  auto result = xQueueSendFromISR(__ieee802154_transmit_queue, &transmit_result, &xHigherPriorityTaskWoken);
  if (result != pdFAIL && xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR Ieee802154::ieee802154_transmit_failed_cb(const uint8_t *frame, esp_ieee802154_tx_error_t error) {
  TransmitResult transmit_result;
  transmit_result.error = error;

  auto xHigherPriorityTaskWoken = pdFALSE;
  auto result = xQueueSendFromISR(__ieee802154_transmit_queue, &transmit_result, &xHigherPriorityTaskWoken);
  if (result != pdFAIL && xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR Ieee802154::ieee802154_transmit_sfd_done_cb(uint8_t *frame) {}

void IRAM_ATTR Ieee802154::ieee802154_energy_detect_done_cb(int8_t power) {}

esp_err_t IRAM_ATTR Ieee802154::ieee802154_enh_ack_generator_cb(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info,
                                                                uint8_t *enhack_frame) {
  // TODO(johboh): Figure out how to use this one to send pending ACKs without needing the additional data request.
  return ESP_OK;
}

Ieee802154::Ieee802154(Configuration configuration, OnMessage on_message, OnDataRequest on_data_request)
    : _on_message(on_message), _configuration(configuration), _sequence_number(configuration.initial_sequence_number),
      _on_data_request(on_data_request) {}

void Ieee802154::cbTask(void *pvParameters) {
  Ieee802154 *_this = (Ieee802154 *)pvParameters;

  while (1) {
    ReceivedFrame received_message;
    auto rx_result = xQueueReceive(__ieee802154_receive_queue, &received_message, portMAX_DELAY);
    if (rx_result == pdPASS) {
      // Accept frames for us specifically, or for short broadcast.
      // This is also done in hardware, but this is future preperation for promiscuous mode.
      auto destination_address = Ieee802154::macToMacLE(received_message.dst_addr);
      auto destination_address_short = Ieee802154::macToShort(received_message.dst_addr);
      auto is_extended_origin = received_message.fcf.src_addr_mode == 0x03;
      auto is_extended_unicast =
          received_message.fcf.dest_addr_mode == 0x3 && destination_address == _this->deviceMacAddress();
      auto is_short_broadcast =
          received_message.fcf.dest_addr_mode == 0x2 && destination_address_short == __UINT16_MAX__;

      if (received_message.frame_info.process && is_extended_origin && (is_extended_unicast || is_short_broadcast)) {
        auto source_address = Ieee802154::macToMac(received_message.src_addr);
        if (received_message.fcf.frame_type == 0x03 && received_message.data[0] == 0x04) {
          // We got a MAC command frame with a "Data Request".
          // Was expecting ESP-IDF to set process to false in this case.
          if (_this->_on_data_request) {
            DataRequest request = {
                .ieee802154 = *_this,
                .source_address = source_address,
            };
            _this->_on_data_request(request);
          }
        } else {
          auto last_processed = _this->_last_processed_sequence_number.find(source_address);
          if (last_processed == _this->_last_processed_sequence_number.end() ||
              last_processed->second != received_message.sequence_number) {
            if (_this->_on_message) {
              Ieee802154::Message message = {
                  .ieee802154 = *_this,
                  .source_address = source_address,
                  .destination_address = destination_address,
                  .payload_size = std::min(received_message.data_length, (uint8_t)sizeof(message.payload)),
              };
              memcpy(message.payload, received_message.data, message.payload_size);
              _this->_on_message(message);
            }
          } else {
            ESP_LOGW(Ieee802154Log::TAG, "Already processed this sequence number %d", received_message.sequence_number);
          }
          _this->_last_processed_sequence_number[source_address] = received_message.sequence_number;
        }
      }
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void Ieee802154::printState() {
  auto state = esp_ieee802154_get_state();
  switch (state) {
  case ESP_IEEE802154_RADIO_DISABLE:
    ESP_LOGI(Ieee802154Log::TAG, "802.15.4 radio state: disabled");
    break;
  case ESP_IEEE802154_RADIO_IDLE:
    ESP_LOGI(Ieee802154Log::TAG, "802.15.4 radio state: idle");
    break;
  case ESP_IEEE802154_RADIO_SLEEP:
    ESP_LOGI(Ieee802154Log::TAG, "802.15.4 radio state: sleep");
    break;
  case ESP_IEEE802154_RADIO_RECEIVE:
    ESP_LOGI(Ieee802154Log::TAG, "802.15.4 radio state: receive");
    break;
  case ESP_IEEE802154_RADIO_TRANSMIT:
    ESP_LOGI(Ieee802154Log::TAG, "802.15.4 radio state: transmit");
    break;
  }
}

void Ieee802154::initializeNvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

void Ieee802154::initialize(bool initialize_nvs) {
  if (_initialized) {
    ESP_LOGE(Ieee802154Log::TAG, "Already initialized.");
    return;
  }

  if (initialize_nvs) {
    initializeNvs();
  }

#ifdef USE_CALLBACKS
  ESP_ERROR_CHECK(esp_ieee802154_event_callback_list_register({
      .rx_done_cb = ieee802154_receive_done_cb,
      .rx_sfd_done_cb = ieee802154_receive_sfd_done_cb,
      .tx_done_cb = ieee802154_transmit_done_cb,
      .tx_failed_cb = ieee802154_transmit_failed_cb,
      .tx_sfd_done_cb = ieee802154_transmit_sfd_done_cb,
      .ed_done_cb = ieee802154_energy_detect_done_cb,
      .enh_ack_generator_cb = ieee802154_enh_ack_generator_cb,
  }));
#endif

  ESP_ERROR_CHECK(esp_ieee802154_enable());

  setChannel(_configuration.channel);

  ESP_ERROR_CHECK(esp_ieee802154_set_panid(_configuration.pan_id));

  ESP_ERROR_CHECK(esp_ieee802154_set_pending_mode(ESP_IEEE802154_AUTO_PENDING_ENABLE));

  ESP_ERROR_CHECK(esp_ieee802154_set_txpower(_configuration.tx_power));

  uint8_t mac[8];
  esp_read_mac(mac, ESP_MAC_IEEE802154);

  for (int i = 0; i < 4; i++) {
    uint8_t temp = mac[i];
    mac[i] = mac[7 - i];
    mac[7 - i] = temp;
  }
  ESP_ERROR_CHECK(esp_ieee802154_set_extended_address(mac));
  // TODO(johboh): Potentially support these promiscuous and coordinator.
  ESP_ERROR_CHECK(esp_ieee802154_set_promiscuous(false));
  ESP_ERROR_CHECK(esp_ieee802154_set_coordinator(false));

  if (_on_message) {
    ESP_ERROR_CHECK(esp_ieee802154_set_rx_when_idle(true));
    ESP_ERROR_CHECK(esp_ieee802154_receive());
  }

  xTaskCreate(cbTask, "cbTask", 8192, this, 20, &cbTaskHandle);
  printState();
  _initialized = true;
}

void Ieee802154::teardown() {
  if (!_initialized) {
    return;
  }

  ESP_ERROR_CHECK(esp_ieee802154_set_rx_when_idle(false));
  ESP_ERROR_CHECK(esp_ieee802154_sleep());
  ESP_ERROR_CHECK(esp_ieee802154_disable());

#ifdef USE_CALLBACKS
  ESP_ERROR_CHECK(esp_ieee802154_event_callback_list_unregister());
#endif

  if (cbTaskHandle != nullptr) {
    vTaskDelete(cbTaskHandle);
    cbTaskHandle = nullptr;
  }

  _initialized = false;
}

bool Ieee802154::broadcast(uint8_t *payload, uint8_t payload_size) {
  if (!_initialized) {
    return false;
  }

  auto total_length = calculateFrameLength(payload_size);
  uint8_t frame_buffer[total_length];
  buildBroadcastFrame(frame_buffer, 0x01 /* frame type = Data Frame */, payload, payload_size);

  bool cca = true;
  uint16_t attempt = 0;
  auto result = transmitInternal(frame_buffer, cca);
  while (attempt++ < _configuration.data_frame_retries &&
         (result != InternalTransmitResult::Success && result != InternalTransmitResult::SuccessWithPending)) {
    vTaskDelay((10 + attempt) / portTICK_PERIOD_MS);
    result = transmitInternal(frame_buffer, cca);
  }

  return result == InternalTransmitResult::Success || result == InternalTransmitResult::SuccessWithPending;
}

bool Ieee802154::transmit(uint64_t destination_mac, uint8_t *payload, uint8_t payload_size) {
  if (!_initialized) {
    return false;
  }

  auto total_length = calculateFrameLength(payload_size);
  uint8_t frame_buffer[total_length];
  buildFrame(frame_buffer, 0x01 /* frame type = Data Frame */, destination_mac, payload, payload_size);

  bool cca = true;
  uint16_t attempt = 0;
  auto result = transmitInternal(frame_buffer, cca);
  while (attempt++ < _configuration.data_frame_retries &&
         (result != InternalTransmitResult::Success && result != InternalTransmitResult::SuccessWithPending)) {
    vTaskDelay((10 + attempt) / portTICK_PERIOD_MS);
    result = transmitInternal(frame_buffer, cca);
  }

  return result == InternalTransmitResult::Success || result == InternalTransmitResult::SuccessWithPending;
}

Ieee802154::DataRequestResult Ieee802154::dataRequest(uint64_t destination_mac) {
  if (!_initialized) {
    return DataRequestResult::Failure;
  }

  auto total_length = calculateFrameLength(1);
  uint8_t frame_buffer[total_length];
  uint8_t payload[1] = {0x04}; // data request
  buildFrame(frame_buffer, 0x03 /* frame type = MAC Command Frame */, destination_mac, payload, 1);

  bool cca = true;
  uint16_t attempt = 0;
  auto result = transmitInternal(frame_buffer, cca);
  while (attempt++ < _configuration.data_request_frame_retries &&
         (result != InternalTransmitResult::Success && result != InternalTransmitResult::SuccessWithPending)) {
    vTaskDelay((10 + attempt) / portTICK_PERIOD_MS);
    result = transmitInternal(frame_buffer, cca);
  }

  switch (result) {
  case InternalTransmitResult::SuccessWithPending:
    return DataRequestResult::DataAvailble;
  case InternalTransmitResult::Success:
    return DataRequestResult::NoDataAvailable;
  default:
    break;
  }
  return DataRequestResult::Failure;
}

uint8_t Ieee802154::calculateFrameLength(uint8_t payload_size) {
  uint8_t mac_hdr_length = sizeof(Ieee802154Specification::Frame);
  uint8_t total_length = mac_hdr_length + payload_size + 2; // +2 for FCS
  return total_length;
}

void Ieee802154::buildFrame(uint8_t *frame_buffer, uint8_t frame_type, uint64_t destination_mac, uint8_t *payload,
                            uint8_t payload_size) {
  auto *frame = reinterpret_cast<Ieee802154Specification::Frame *>(frame_buffer);

  // Populate the frame fields
  auto total_length = calculateFrameLength(payload_size);
  frame->length = total_length - 1; // Exclude the length field itself
  frame->fcf = (Ieee802154Specification::FrameControlField){
      .frame_type = frame_type,
      .security_enabled = 0,
      .frame_pending = 0,
      .ack_request = (uint16_t)(destination_mac != UINT64_MAX ? 1 : 0), // Turned off ACK for broadcast
      .pan_id_compression = 1,
      .reserved = 0,
      .sequence_number_suppression = 0,
      .ie_list_present = 0,
      .dest_addr_mode = 0x3, // 64-bit
      .frame_version = 0x1,  // IEEE 802.15.4-2006
      .src_addr_mode = 0x3,  // 64-bit
  };
  frame->sequence_number = ++_sequence_number;
  // We stay within same PAN
  frame->destination_pan_id = _configuration.pan_id;
  frame->destination_mac = destination_mac;

  uint8_t mac[8];
  ESP_ERROR_CHECK(esp_ieee802154_get_extended_address(mac));
  frame->source_mac = macToMac(mac);

  // Copy the payload
  memcpy(frame->payload, payload, payload_size);
}

void Ieee802154::buildBroadcastFrame(uint8_t *frame_buffer, uint8_t frame_type, uint8_t *payload,
                                     uint8_t payload_size) {
  auto *frame = reinterpret_cast<Ieee802154Specification::BroadcastFrame *>(frame_buffer);

  // Populate the frame fields
  auto total_length = calculateFrameLength(payload_size);
  frame->length = total_length - 1; // Exclude the length field itself
  frame->fcf = (Ieee802154Specification::FrameControlField){
      .frame_type = frame_type,
      .security_enabled = 0,
      .frame_pending = 0,
      .ack_request = 0, // Broadcast cannot be acked
      .pan_id_compression = 1,
      .reserved = 0,
      .sequence_number_suppression = 0,
      .ie_list_present = 0,
      .dest_addr_mode = 0x2, // 16-bit
      .frame_version = 0x1,  // IEEE 802.15.4-2006
      .src_addr_mode = 0x3,  // 64-bit
  };
  frame->sequence_number = ++_sequence_number;
  frame->destination_pan_id = 0xFFFF; // Broadcast
  frame->destination_mac = 0xFFFF;    // Broadcast

  uint8_t mac[8];
  ESP_ERROR_CHECK(esp_ieee802154_get_extended_address(mac));
  frame->source_mac = macToMac(mac);

  // Copy the payload
  memcpy(frame->payload, payload, payload_size);
}

Ieee802154::InternalTransmitResult Ieee802154::transmitInternal(uint8_t *frame, bool cca) {
  std::scoped_lock lock(_send_mutex);
  ESP_ERROR_CHECK(esp_ieee802154_transmit(frame, cca));

  TransmitResult transmit_result;
  // TODO(johboh): move timeout to config
  auto tx_result = xQueueReceive(__ieee802154_transmit_queue, &transmit_result, 1000 / portTICK_PERIOD_MS);
  if (tx_result == pdPASS) {
    switch (transmit_result.error) {
    case ESP_IEEE802154_TX_ERR_NONE:
      return transmit_result.pending ? InternalTransmitResult::SuccessWithPending : InternalTransmitResult::Success;
      break;
    case ESP_IEEE802154_TX_ERR_NO_ACK:
      ESP_LOGE(Ieee802154Log::TAG, "Transmit Result: no ack");
      return InternalTransmitResult::AckNotReceived;
      break;
    case ESP_IEEE802154_TX_ERR_INVALID_ACK:
      ESP_LOGE(Ieee802154Log::TAG, "Transmit Result: invalid ack");
      return InternalTransmitResult::AckNotReceived;
      break;
    case ESP_IEEE802154_TX_ERR_CCA_BUSY:
      ESP_LOGE(Ieee802154Log::TAG, "Transmit Result: channel is busy");
      return InternalTransmitResult::ChannelIsBusy;
      break;
    case ESP_IEEE802154_TX_ERR_ABORT:
      ESP_LOGE(Ieee802154Log::TAG, "Transmit Result: abort");
      return InternalTransmitResult::ChannelIsBusy;
      break;
    default:
      ESP_LOGE(Ieee802154Log::TAG, "Transmit Result: %d", transmit_result.error);
      break;
    }
  } else {
    ESP_LOGE(Ieee802154Log::TAG, "Transmit Result: timeout");
    return InternalTransmitResult::Timeout; // Timeout.
  }

  return InternalTransmitResult::Unknown;
}

void Ieee802154::setPending(uint64_t mac) {
  // TODO(johboh): Endianess?
  uint8_t mac_array[8];
  for (int i = 0; i < 8; ++i) {
    mac_array[i] = (mac >> (56 - i * 8)) & 0xFF;
  }
  ESP_ERROR_CHECK(esp_ieee802154_add_pending_addr(mac_array, false));
}

void Ieee802154::clearPending(uint64_t mac) {
  // TODO(johboh): Endianess?
  uint8_t mac_array[8];
  for (int i = 0; i < 8; ++i) {
    mac_array[i] = (mac >> (56 - i * 8)) & 0xFF;
  }
  esp_ieee802154_clear_pending_addr(mac_array, false);
}

void Ieee802154::receive(OnMessage on_message) {
  if (on_message && !_on_message) {
    ESP_ERROR_CHECK(esp_ieee802154_set_rx_when_idle(true));
    ESP_ERROR_CHECK(esp_ieee802154_receive());
  } else if (!on_message && _on_message) {
    ESP_ERROR_CHECK(esp_ieee802154_set_rx_when_idle(false));
    ESP_ERROR_CHECK(esp_ieee802154_sleep());
  }
  _on_message = on_message;
}

void Ieee802154::setChannel(uint8_t channel) {
  _configuration.channel = std::clamp(channel, (uint8_t)11, (uint8_t)26);
  ESP_ERROR_CHECK(esp_ieee802154_set_channel(_configuration.channel));
}

void Ieee802154::setNumberOfDataFramesRetries(uint8_t number_of_retries) {
  _configuration.data_frame_retries = number_of_retries;
}

uint64_t Ieee802154::deviceMacAddress() {
  uint8_t mac[8];
  esp_read_mac(mac, ESP_MAC_IEEE802154);
  return Ieee802154::macToMac(mac);
}

// 8 bytes mac addresses (not 6 as normally)
uint64_t Ieee802154::macToMac(uint8_t *mac_addr) {
  return ((uint64_t)mac_addr[0] << 56) + ((uint64_t)mac_addr[1] << 48) + ((uint64_t)mac_addr[2] << 40) +
         ((uint64_t)mac_addr[3] << 32) + ((uint64_t)mac_addr[4] << 24) + ((uint64_t)mac_addr[5] << 16) +
         ((uint64_t)mac_addr[6] << 8) + ((uint64_t)mac_addr[7]);
}

// 8 bytes mac addresses (not 6 as normally), endianess change.
uint64_t Ieee802154::macToMacLE(uint8_t *mac_addr) {
  return ((uint64_t)mac_addr[7] << 56) + ((uint64_t)mac_addr[6] << 48) + ((uint64_t)mac_addr[5] << 40) +
         ((uint64_t)mac_addr[4] << 32) + ((uint64_t)mac_addr[3] << 24) + ((uint64_t)mac_addr[2] << 16) +
         ((uint64_t)mac_addr[1] << 8) + ((uint64_t)mac_addr[0]);
}

// 8 bytes mac addresses array into short 16-bit address.
uint16_t Ieee802154::macToShort(uint8_t *mac_addr) { return ((uint16_t)mac_addr[1] << 8) + ((uint16_t)mac_addr[0]); }

bool IRAM_ATTR Ieee802154::parseFrame(uint8_t *raw_frame, ReceivedFrame *received_frame) {
  if (raw_frame != nullptr) {
    auto length = raw_frame[0];
    received_frame->frame_length = length;

    // Dynamically calculate MAC header length based on FCF
    Ieee802154Specification::FrameControlField *fcf =
        reinterpret_cast<Ieee802154Specification::FrameControlField *>(raw_frame + 1);

    uint8_t mac_hdr_length = 3;        // Base header size (FCF + Sequence Number)
    uint8_t *addr_ptr = raw_frame + 3; // after length(1 byte) and FCF (2 bytes)

    received_frame->sequence_number = *addr_ptr;
    addr_ptr += 1;

    received_frame->fcf = *fcf;

    if (fcf->dest_addr_mode != 0) {
      memcpy(&received_frame->dst_pan, addr_ptr, 2);
      addr_ptr += 2;
      mac_hdr_length += 2;

      if (fcf->dest_addr_mode == 0x2) {
        memcpy(received_frame->dst_addr, addr_ptr, 2);
        addr_ptr += 2;
        mac_hdr_length += 2;
      } else if (fcf->dest_addr_mode == 0x3) {
        memcpy(received_frame->dst_addr, addr_ptr, 8);
        addr_ptr += 8;
        mac_hdr_length += 8;
      }
    } else {
      // no dest address
    }

    if (!fcf->pan_id_compression) {
      received_frame->with_src_pan = true;
      memcpy(&received_frame->src_pan, addr_ptr, 2);
      addr_ptr += 2;
      mac_hdr_length += 2;
    }

    if (fcf->src_addr_mode != 0) {
      if (fcf->src_addr_mode == 0x2) {
        memcpy(received_frame->src_addr, addr_ptr, 2);
        addr_ptr += 2;
        mac_hdr_length += 2;
      } else if (fcf->src_addr_mode == 0x3) {
        memcpy(received_frame->src_addr, addr_ptr, 8);
        addr_ptr += 8;
        mac_hdr_length += 8;
      }
    }

    // Calculate payload length
    uint8_t payload_length = length - mac_hdr_length - 2; // Also remove FCS
    if (payload_length > 0 && payload_length <= sizeof(received_frame->data)) {
      memcpy(received_frame->data, addr_ptr, payload_length);
      received_frame->data_length = payload_length;
    }
    return true;
  }
  return false;
}

void Ieee802154::printFrameInfo(esp_ieee802154_frame_info_t &frame_info) {
  ESP_LOGI(Ieee802154Log::TAG, "-- Frame Info --");
  ESP_LOGI(Ieee802154Log::TAG, "  - Pending: %s", frame_info.pending ? "true" : "false");
  ESP_LOGI(Ieee802154Log::TAG, "  - Process: %s", frame_info.process ? "true" : "false");
  ESP_LOGI(Ieee802154Log::TAG, "  - Channel: %u", frame_info.channel);
  ESP_LOGI(Ieee802154Log::TAG, "  - RSSI: %d", frame_info.rssi);
  ESP_LOGI(Ieee802154Log::TAG, "  - LQI: %u", frame_info.lqi);
  ESP_LOGI(Ieee802154Log::TAG, "  - Timestamp: %llu", frame_info.timestamp);
}

void Ieee802154::printFrameControlField(const Ieee802154Specification::FrameControlField &fcf) {
  ESP_LOGI(Ieee802154Log::TAG, "-- Frame Control Field --");
  ESP_LOGI(Ieee802154Log::TAG, "  - Frame Type: %u", fcf.frame_type);
  ESP_LOGI(Ieee802154Log::TAG, "  - Security Enabled: %s", fcf.security_enabled ? "true" : "false");
  ESP_LOGI(Ieee802154Log::TAG, "  - Frame Pending: %s", fcf.frame_pending ? "true" : "false");
  ESP_LOGI(Ieee802154Log::TAG, "  - ACK Request: %s", fcf.ack_request ? "true" : "false");
  ESP_LOGI(Ieee802154Log::TAG, "  - PAN ID Compression: %s", fcf.pan_id_compression ? "true" : "false");
  ESP_LOGI(Ieee802154Log::TAG, "  - Destination Address Mode: %u", fcf.dest_addr_mode);
  ESP_LOGI(Ieee802154Log::TAG, "  - Frame Version: %u", fcf.frame_version);
  ESP_LOGI(Ieee802154Log::TAG, "  - Source Address Mode: %u", fcf.src_addr_mode);
}

void Ieee802154::printMessageAddresses(const ReceivedFrame &message) {
  // Print Destination Address
  if (message.fcf.dest_addr_mode == 0x2) { // 16-bit address
    ESP_LOGI(Ieee802154Log::TAG, "  - Destination Address: %02X%02X, PAN: 0x%04X", message.dst_addr[0],
             message.dst_addr[1], message.dst_pan);
  } else if (message.fcf.dest_addr_mode == 0x3) { // 64-bit address
    ESP_LOGI(Ieee802154Log::TAG, "  - Destination Address: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X, PAN: 0x%04X",
             message.dst_addr[0], message.dst_addr[1], message.dst_addr[2], message.dst_addr[3], message.dst_addr[4],
             message.dst_addr[5], message.dst_addr[6], message.dst_addr[7], message.dst_pan);
  }

  // Print Source Address
  if (message.fcf.src_addr_mode == 0x2) { // 16-bit address
    ESP_LOGI(Ieee802154Log::TAG, "  - Source Address: %02X%02X", message.src_addr[0], message.src_addr[1]);
  } else if (message.fcf.src_addr_mode == 0x3) { // 64-bit address
    ESP_LOGI(Ieee802154Log::TAG, "  - Source Address: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", message.src_addr[0],
             message.src_addr[1], message.src_addr[2], message.src_addr[3], message.src_addr[4], message.src_addr[5],
             message.src_addr[6], message.src_addr[7]);
  }
  if (message.with_src_pan) {
    ESP_LOGI(Ieee802154Log::TAG, "  - source PAN: 0x%04X", message.src_pan);
  } else {
    ESP_LOGI(Ieee802154Log::TAG, "  - No source PAN");
  }
}

void Ieee802154::printFrame(const ReceivedFrame &frame) {
  ESP_LOGI(Ieee802154Log::TAG, "-- Frame --");
  ESP_LOGI(Ieee802154Log::TAG, "  - frame length: %u", frame.frame_length);
  ESP_LOGI(Ieee802154Log::TAG, "  - payload length: %u", frame.data_length);
  ESP_LOGI(Ieee802154Log::TAG, "  - sequence number: %u", frame.sequence_number);
  printFrameControlField(frame.fcf);
  ESP_LOGI(Ieee802154Log::TAG, "-- Addresses & PAN --");
  printMessageAddresses(frame);
}
