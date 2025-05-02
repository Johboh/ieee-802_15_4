#pragma once

#include <cstdint>
#include <esp_ieee802154.h>

// Structures sent on "the wire", according to 802.15.4 specification.
namespace Ieee802154Specification {

struct __attribute__((packed)) FrameControlField {
  uint16_t frame_type : 3;
  uint16_t security_enabled : 1;
  uint16_t frame_pending : 1;
  uint16_t ack_request : 1;
  uint16_t pan_id_compression : 1;
  uint16_t reserved : 1;
  uint16_t sequence_number_suppression : 1;
  uint16_t ie_list_present : 1;
  uint16_t dest_addr_mode : 2;
  uint16_t frame_version : 2;
  uint16_t src_addr_mode : 2;
};

// Frame for 64bit src/dst addresses, and PAN ID compression.
// This is the frame we send on the wire.
struct __attribute__((packed)) Frame {
  uint8_t length; // Total frame length, including FCS
  FrameControlField fcf;
  uint8_t sequence_number;
  uint16_t destination_pan_id;
  uint64_t destination_mac;
  uint64_t source_mac;
  uint8_t payload[];
};

// Frame for 64bit src address, 16bit dst address, and PAN ID compression.
// Used for broadcasting.
// This is the frame we send on the wire.
struct __attribute__((packed)) BroadcastFrame {
  uint8_t length; // Total frame length, including FCS
  FrameControlField fcf;
  uint8_t sequence_number;
  uint16_t destination_pan_id = 0xFFFF;
  uint16_t destination_mac = 0xFFFF;
  uint64_t source_mac;
  uint8_t payload[];
};

}; // namespace Ieee802154Specification

// Stuctures related to ESP-IDF 802.15.4 internals, upon receiving a frame or a result after trasmitting a frame.
// These are not going on "the wire".
namespace Ieee802154Internal {

struct ReceivedFrame {
  esp_ieee802154_frame_info_t frame_info;
  Ieee802154Specification::FrameControlField fcf;
  uint8_t frame_length;
  uint8_t sequence_number = 0;
  uint8_t src_addr[8] = {0};
  uint8_t dst_addr[8] = {0};
  bool with_src_pan = false;
  uint16_t src_pan = 0x0000;
  uint16_t dst_pan = 0x0000;
  uint8_t data[127] = {0};
  uint8_t data_length = 0;
};

struct TransmitResult {
  bool pending = false;
  esp_ieee802154_tx_error_t error;
};

}; // namespace Ieee802154Internal