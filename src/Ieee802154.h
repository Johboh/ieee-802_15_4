#pragma once

#include "Ieee802154Types.h"
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

/**
 * Class around ESP-IDF 802.15.4 layer.
 * Requires: 5.4.1+, 5.3.3+, 5.2.4+, 5.1.6+
 *
 * Allow for sending and reciving payload.
 *
 * Still to do:
 * - Figure out how security works. Can only find a way to transmit encrypted frames, but not decrypt them on the
 * reciver side. Currently, the payload has to be decrypted by the user of this library.
 * - Support frame version 802.15.5-2015 (frame_version=0b10). Currently, setting frame version to be 2015 make frames
 * not be acked by reciver, and only way to recive the is to use promiscuous mode.
 * - If we can get frame version 2015 to work, we could use Enhanched/Enh-Ack, which ESP-IDF has a callback for to
 * generate such but is never called, we could deliver payload like firmware updates and such in the initial message
 * recived from a node, instead of needing the data request frame (where we first send a data frame followed by a data
 * request frame).
 * - Not figured out how to set the bitrate and also by lowering it, allowing for longer range.
 * - TX power can be set to -127 to +127, which doesn't make sense. Not sure what the actual range is.
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
  };

  typedef std::function<void(Ieee802154::Message message)> OnMessage;

  struct DataRequest {
    Ieee802154 &ieee802154;
    uint64_t source_address = 0;
  };

  typedef std::function<void(Ieee802154::DataRequest request)> OnDataRequest;

  struct Configuration {
    /**
     * 802.15.4 channel to use. Value between 11 and 16, and its recommended to pick a channel that is in between the
     * common WiFi channels. 15 is a good number.
     */
    uint8_t channel;
    /**
     * Private Area Network Identifier. Should be same between host and node.
     */
    uint16_t pan_id;
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
     * @brief Transmit power in dB.
     * Unknown allowed range.
     */
    int8_t tx_power = 20;
  };

  /**
   * @brief Create a new Ieee802154 object.
   *
   * @param configuration see Configuration.
   * @param on_message if set, radio will go into receive mode and allow for reciving messages targeted to
   * this device, or if promiscuous_mode is set to true in the Configuration, also broadcast messages. Can also be set
   * later using receive() call.
   * @param on_data_request callback for when a data request was received. See set/clearPending(). This callback will be
   * called regardless if the sender has any pending data.
   */
  Ieee802154(Configuration configuration, OnMessage on_message = {}, OnDataRequest on_data_request = {});

public:
  /**
   * Call to initialize. Must be called before any transmit call.
   * Use teardown() to tear down again.
   *
   * @param initialize_nvs NVS is needed for radio calibration. Set to true and this function will initialize NVS. False
   * if this is taken care of already.
   */
  void initialize(bool initialize_nvs = true);
  void teardown();

  uint16_t panId() { return _configuration.pan_id; }
  uint16_t channel() { return _configuration.channel; }
  /**
   * Sequence number to use in the next outgoing message.
   */
  uint8_t nextSequenceNumber() { return _sequence_number; }
  /**
   * Clear the persisted/last known sequence number for a given device/MAC.
   * Useful in case you know that the device is going to forget its sequence number, for example on firmware update.
   */
  void clearLastKnownSequenceNumberFor(uint64_t mac) { _last_processed_sequence_number.erase(mac); }

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
   * @brief Get the device mac address for this device. This would be the source address in the 802.15.4 frame (or the
   * destination address for a sender).
   */
  uint64_t deviceMacAddress();

  /**
   * @brief Send broadcast frame.
   *
   * @param payload the payload to send.
   * @param payload_size the payload size. Maximum payload size 104 bytes without security enabled.
   */
  bool broadcast(uint8_t *payload, uint8_t payload_size);
  /**
   * @brief Send frame to destination.
   *
   * @param destination_mac big endian
   * @param payload the payload to send.
   * @param payload_size the payload size. Maximum payload size 104 bytes without security enabled.
   */
  bool transmit(uint64_t destination_mac, uint8_t *payload, uint8_t message_size);

  enum class DataRequestResult {
    NoDataAvailable,
    DataAvailble,
    Failure,
  };

  /**
   * Send a MAC command with a data request to the reciver, where the reciver will ack with pending or not pending. The
   * reciver uses setPending()/clearPending() to control the pending reply for an data request.
   *
   * @param destination_mac big endian
   */
  DataRequestResult dataRequest(uint64_t destination_mac);

  /**
   * Start receiving and set callback for received messages. If callback already set in constructor, no need to call
   * again. Call with empty callback to clear/stop reciving frames.
   */
  void receive(OnMessage on_message);

  /**
   * @brief Let transmitter know that there is pending data when transmitter sends a data request.
   * Must be in a receiving mode for this to work.
   */
  void setPending(uint64_t mac);
  /**
   * Clear pending bit.
   */
  void clearPending(uint64_t mac);

  // 8 bytes mac addresses (not 6 as normally)
  // input and output in big endian.
  static uint64_t macToMac(uint8_t *mac_addr);

  /**
   * Debug printout of current state.
   */
  void printState();

private:
  void initializeNvs();
  // Including FCS length (2 bytes)
  uint8_t calculateFrameLength(uint8_t payload_size);
  void buildBroadcastFrame(uint8_t *frame, uint8_t frame_type, uint8_t *payload, uint8_t payload_size);
  void buildFrame(uint8_t *frame, uint8_t frame_type, uint64_t destination_mac, uint8_t *payload, uint8_t payload_size);

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
  static uint64_t macToMacLE(uint8_t *mac_addr);
  static uint16_t macToShort(uint8_t *mac_addr);
  static bool parseFrame(uint8_t *raw_frame, Ieee802154Internal::ReceivedFrame *received_frame);
  static void printFrameInfo(esp_ieee802154_frame_info_t &frame_info);
  static void printFrameControlField(const Ieee802154Specification::FrameControlField &fcf);
  static void printMessageAddresses(const Ieee802154Internal::ReceivedFrame &message);
  static void printFrame(const Ieee802154Internal::ReceivedFrame &frame);
  static uint64_t toLittleEndian(uint64_t value);

private: // static callbacks
  static void cbTask(void *pvParameters);

// Callback available from 5.4.1, 5.3.3, 5.2.4, 5.1.6
// Private callbacks if we have callbacks support, otherwise public.
// See initialize()
#if ESP_IDF_VERSION_MAJOR == 5 and ((ESP_IDF_VERSION_MINOR == 1 and ESP_IDF_VERSION_PATCH >= 6) or                     \
                                    (ESP_IDF_VERSION_MINOR == 2 and ESP_IDF_VERSION_PATCH >= 4) or                     \
                                    (ESP_IDF_VERSION_MINOR == 3 and ESP_IDF_VERSION_PATCH >= 3) or                     \
                                    (ESP_IDF_VERSION_MINOR == 4 and ESP_IDF_VERSION_PATCH >= 1))
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
  std::mutex _send_mutex;
  bool _initialized = false;
  TaskHandle_t cbTaskHandle;
  Configuration _configuration;
  uint8_t _sequence_number = 0;
  OnDataRequest _on_data_request;
  std::map<uint64_t, uint8_t> _last_processed_sequence_number;
};
