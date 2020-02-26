#include "packet_serializer.h"

namespace srrg2_core {
  PacketSerializer::PacketSerializer() {
  }

  PacketSerializer::~PacketSerializer() {
  }

  void PacketSerializer::putPacket(const uint8_t packet_type_, const PacketBase* packet_) {
    assert(packet_);

    if (!_buffer || !_buffer->size)
      throw std::runtime_error("[PacketSerializer] invalid buffer, please set the buffer");

    if (packet_type_ == PACKET_TYPE_INVALID)
      throw std::runtime_error("[PacketSerializer] invalid packet type");

    // ia we put first the packet type
    _buffer->data = putInBuffer(_buffer->data, packet_type_);

    // ia finally the payload
    _buffer->data = packet_->serialize(_buffer->data);

    // ia increment the packet counter
    ++_buffer->num_packets;

    // ia delete the packet - now it doesn't exist anymore in this context
    delete packet_;
  }
} // namespace srrg2_core
