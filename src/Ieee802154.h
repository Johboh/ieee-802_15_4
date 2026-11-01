#pragma once

#include "Ieee802154Types.h"
#include <atomic>
#include <cstdint>
#include <esp_idf_version.h>
#include <esp_ieee802154.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace Ieee802154Log {
const char TAG[] = "802.15.4";
} // namespace Ieee802154Log

// Callback available from 5.5+, 5.4.1+, 5.3.3+, 5.2.4+, 5.1.6+
#if ESP_IDF_VERSION_MAJOR == 5 and                                                                                     \
    ((ESP_IDF_VERSION_MINOR == 1 and ESP_IDF_VERSION_PATCH >= 6) or                                                    \
     (ESP_IDF_VERSION_MINOR == 2 and ESP_IDF_VERSION_PATCH >= 4) or                                                    \
     (ESP_IDF_VERSION_MINOR == 3 and ESP_IDF_VERSION_PATCH >= 3) or                                                    \
     (ESP_IDF_VERSION_MINOR == 4 and ESP_IDF_VERSION_PATCH >= 1) or ESP_IDF_VERSION_MINOR >= 5)
#define IEEE802154_USE_CALLBACKS 1
#endif

/**
 * Class around ESP-IDF 802.15.4 layer.
 * Requires: 5.4.1+, 5.3.3+, 5.2.4+, 5.1.6+
 *
 * Allow for sending and receiving payload.
 *
 * Technical Notes:
 * - Security: This library transmits frames without link-layer encryption (security_enabled=0).
 *   For confidential or authenticated communications, use application-layer encryption (e.g. mbedTLS AES-GCM
 *   or ChaCha20-Poly1305) to encrypt payloads before transmit() and decrypt inside receive callbacks. The ESP32
 *   802.15.4 radio hardware only supports hardware-accelerated encryption on transmit; reception must be decrypted
 *   in software regardless.
 * - Frame Version: Uses IEEE 802.15.4-2006 (frame_version=0b01).
 * - Auto Pending: Configured with ESP_IEEE802154_AUTO_PENDING_ENHANCED so hardware auto-ACK sets the frame_pending
 *   bit in all ACK frames (including Data frame ACKs) when the peer MAC matches an entry in the pending table.
 *   This enables sleeping nodes to conditionally poll via dataRequest() only when pending data is waiting.
 * - Bitrate: The 2.4 GHz IEEE 802.15.4 PHY is standardized strictly at 250 kbps (DSSS O-QPSK at 2 Mchip/s).
 * - TX Power: Expressed in dBm. Typical range is -20 to +20 dBm (ESP32-C6) and -24 to +20 dBm (ESP32-H2).
 */
class Ieee802154 {
public:
  struct Message {
    Ieee802154 &ieee802154;
    uint64_t source_address = 0;
    // Either the address for this device or an broadcast address.
    uint64_t destination_address = 0;
    uint8_t payload[127] = {0};
    uint8_t payload_size = 0;
    int8_t rssi = 0;
  };

  typedef std::function<void(Ieee802154::Message message)> OnMessage;

  struct DataRequest {
    Ieee802154 &ieee802154;
    uint64_t source_address = 0;
  };

  typedef std::function<void(Ieee802154::DataRequest request)> OnDataRequest;

  struct Configuration {
    /**
     * 802.15.4 channel to use. Value between 11 and 26, and its recommended to pick a channel that is in between the
     * common WiFi channels. 15 is the most safe best. Other good options are 20. 25-26 are good too, but risky
     * compatibiltiy wise.
     */
    uint8_t channel = 15;
    /**
     * Private Area Network Identifier. Should be same between host and node.
     */
    uint16_t pan_id = 0x4242;
    /**
     * If the device is going to deep sleep between transmissions, keep the sequence number in an RTC variable and set
     * it in the Configuration upon waking up.
     * Use [nextSequenceNumber] to retreive the next sequence number (to persist upon sleep).
     */
    uint8_t initial_sequence_number = 0;
    /**
     * How many times to try resending a data frame on failure (no ack, busy cca etc)
     */
    uint8_t data_frame_retries = 30;
    /**
     * How many times to try resending a data request frame on failure (no ack, busy cca etc)
     */
    uint8_t data_request_frame_retries = 30;
    /**
     * @brief Transmit power in dBm.
     * Supported range: -20 to +20 dBm (ESP32-C6) / -24 to +20 dBm (ESP32-H2).
     */
    int8_t tx_power = 20;
  };

  /**
   * @brief Create a new Ieee802154 object.
   *
   * @param configuration see Configuration.
   * @param on_message if set, radio will go into receive mode and allow for receiving messages targeted to
   * this device, or if promiscuous_mode is set to true in the Configuration, also broadcast messages. Can also be set
   * later using receive() call.
   * @param on_data_request callback for when a data request was received. See set/clearPending(). This callback will be
   * called regardless if the sender has any pending data.
   */
  Ieee802154(Configuration configuration, OnMessage on_message = {}, OnDataRequest on_data_request = {});
  ~Ieee802154();

  Ieee802154(const Ieee802154 &) = delete;
  Ieee802154 &operator=(const Ieee802154 &) = delete;

public:
  /**
   * Call to initialize. Must be called before any transmit call.
   * Use teardown() to tear down again.
   *
   * @param initialize_nvs NVS is needed for radio calibration. Set to true and this function will initialize NVS. False
   * if this is taken care of already.
   * @return true if initialization succeeded, false otherwise.
   */
  bool initialize(bool initialize_nvs = true);
  void teardown();

  /**
   * @brief Check whether the radio and dispatcher task are initialized and running.
   */
  bool isInitialized() const { return _initialized.load(std::memory_order_acquire); }

  uint16_t panId() const { return _configuration.pan_id; }
  uint8_t channel() const { return _configuration.channel; }
  /**
   * Sequence number to use in the next outgoing message.
   */
  uint8_t nextSequenceNumber() const;
  /**
   * Clear the persisted/last known sequence number for a given device/MAC.
   * Useful in case you know that the device is going to forget its sequence number, for example on firmware update.
   */
  void clearLastKnownSequenceNumberFor(uint64_t mac);

  /**
   * @brief Set/change the channel.
   *
   * @param channel channel number between 11 and 26.
   */
  void setChannel(uint8_t channel);

  /**
   * Set how many times to try resending a data frame on failure (no ack, busy cca etc)
   */
  void setNumberOfDataFramesRetries(uint8_t number_of_retries);

  /**
   * @brief Get the device MAC address for this device in canonical 64-bit integer format
   * (e.g. 0xe4b323fffe926d10ULL where MSB is the first octet/OUI).
   * Matches Message.source_address / Message.destination_address.
   */
  uint64_t deviceMacAddress() const;

  /**
   * @brief Send broadcast frame.
   *
   * @param payload the payload to send.
   * @param payload_size the payload size. Maximum payload size 104 bytes.
   */
  bool broadcast(const uint8_t *payload, uint8_t payload_size);
  /**
   * Result of a unicast transmit() operation.
   */
  enum class TransmitResult {
    Success,
    SuccessWithPending,
    Failure,
  };

  /**
   * @brief Send frame to destination.
   *
   * @param destination_mac Destination address in canonical 64-bit integer format (e.g. 0xe4b323fffe926d10ULL).
   * Automatically serialized to IEEE 802.15.4 little-endian wire order.
   * @param payload the payload to send.
   * @param payload_size the payload size. Maximum payload size 104 bytes.
   * @return TransmitResult indicating Success, SuccessWithPending (receiver has queued data for this node), or Failure.
   */
  TransmitResult transmit(uint64_t destination_mac, const uint8_t *payload, uint8_t payload_size);

  /**
   * Result of a dataRequest() poll operation.
   */
  enum class DataRequestResult {
    NoDataAvailable,
    DataAvailable,
    Failure,
  };

  /**
   * @brief Send a MAC Command frame with a Data Request to poll a receiver/coordinator.
   *
   * The receiver's hardware automatically responds with an ACK containing the frame_pending bit,
   * controlled on the receiver side via setPending() / clearPending().
   *
   * When to use:
   * - Use dataRequest() when this device has NO payload to send, but wants to poll the coordinator
   *   to ask if any messages/commands are queued for it.
   *
   * When NOT needed:
   * - If this device is already sending a data message via transmit(), you do NOT need dataRequest().
   *   transmit() directly returns TransmitResult::SuccessWithPending if the receiver has data queued for this node,
   *   saving a redundant round-trip.
   *
   * @param destination_mac Destination address in canonical 64-bit integer format (e.g. 0xe4b323fffe926d10ULL).
   * @return DataRequestResult indicating whether data is available, no data is available, or failure (e.g. no
   * ACK/timeout).
   */
  DataRequestResult dataRequest(uint64_t destination_mac);

  /**
   * Start receiving and set callback for received messages. If callback already set in constructor, no need to call
   * again. Call with empty callback to clear/stop receiving frames.
   */
  void receive(OnMessage on_message);

  /**
   * @brief Let transmitter know that there is pending data when transmitter sends a data request.
   * Must be in a receiving mode for this to work.
   *
   * @param mac Canonical 64-bit MAC address (e.g. 0xe4b323fffe926d10ULL, as in Message.source_address or
   * deviceMacAddress()). If you only have the raw 8-byte buffer from esp_read_mac(), convert it first using
   * macToMac(buffer). Converted internally to the little-endian wire format expected by
   * esp_ieee802154_add_pending_addr().
   */
  void setPending(uint64_t mac);
  /**
   * Clear pending bit for a given device MAC address.
   *
   * @param mac Canonical 64-bit MAC address (same format as setPending()).
   */
  void clearPending(uint64_t mac);

  /**
   * Convert an 8-byte MAC address array in big-endian (canonical/network order, e.g. from esp_read_mac)
   * into a canonical 64-bit integer MAC address (MSB is mac_addr[0]).
   */
  static uint64_t macToMac(const uint8_t *mac_addr);

  /**
   * Debug printout of current state.
   */
  void printState();

private:
  void initializeNvs();
  // Including FCS length (2 bytes)
  uint8_t calculateFrameLength(uint8_t payload_size);
  uint8_t calculateBroadcastFrameLength(uint8_t payload_size);
  void buildBroadcastFrame(uint8_t *frame, uint8_t frame_type, const uint8_t *payload, uint8_t payload_size);
  void buildFrame(uint8_t *frame, uint8_t frame_type, uint64_t destination_mac, const uint8_t *payload,
                  uint8_t payload_size);

  enum class InternalTransmitResult {
    Timeout,
    AckNotReceived,
    ChannelIsBusy,
    Unknown,
    Success,
    SuccessWithPending,
  };

  InternalTransmitResult transmitInternal(uint8_t *frame, bool cca);

private: // static helpers
  static uint64_t macToMacLE(const uint8_t *mac_addr);
  static uint16_t macToShort(const uint8_t *mac_addr);
  static bool parseFrame(const uint8_t *raw_frame, Ieee802154Internal::ReceivedFrame *received_frame);
  static void printFrameInfo(esp_ieee802154_frame_info_t &frame_info);
  static void printFrameControlField(const Ieee802154Specification::FrameControlField &fcf);
  static void printMessageAddresses(const Ieee802154Internal::ReceivedFrame &message);
  static void printFrame(const Ieee802154Internal::ReceivedFrame &frame);

private: // static callbacks
  static void cbTask(void *pvParameters);

// Private callbacks if we have callbacks support, otherwise public.
// See initialize()
#ifdef IEEE802154_USE_CALLBACKS
private:
#else
public:
#endif
  static void ieee802154_receive_done_cb(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info);
  static void ieee802154_receive_sfd_done_cb(void);
  static void ieee802154_transmit_done_cb(const uint8_t *frame, const uint8_t *ack,
                                          esp_ieee802154_frame_info_t *ack_frame_info);
  static void ieee802154_transmit_failed_cb(const uint8_t *frame, esp_ieee802154_tx_error_t error);
  static void ieee802154_transmit_sfd_done_cb(uint8_t *frame);
  static void ieee802154_energy_detect_done_cb(int8_t power);
  static esp_err_t ieee802154_enh_ack_generator_cb(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info,
                                                   uint8_t *enhack_frame);

private:
  OnMessage _on_message;
  mutable std::mutex _send_mutex;
  mutable std::mutex _rx_mutex;
  std::atomic<bool> _initialized{false};
  std::atomic<bool> _running{false};
  std::atomic<bool> _task_running{false};
  TaskHandle_t cbTaskHandle = nullptr;
  Configuration _configuration;
  uint8_t _sequence_number = 0;
  OnDataRequest _on_data_request;
  std::map<uint64_t, uint8_t> _last_processed_sequence_number;
  uint64_t _device_mac = 0;
  uint8_t _device_mac_le[8] = {0};
};
