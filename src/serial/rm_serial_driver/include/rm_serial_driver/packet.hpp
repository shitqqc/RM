// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
enum class SentryDecision_e : uint8_t { IDLE = 0, SPIN, NAVI, AIMING }; // 1: spin, 2: navigate, 3: aiming

struct VisionReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t localColor : 1;
  uint8_t taskMode : 2;
  bool resetTracker : 1;
  uint8_t isplay : 1;
  bool targetChange : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw;
  float aimX;
  float aimY;
  float aimZ;
  uint16_t gameTime;
  uint32_t rTimestamp;
  uint16_t checkSum;
} __attribute__((packed));

struct NaviReceivePacket
{
  uint8_t header = 0x6A;
  float pitch;
  float yaw;
  uint16_t game_time;
  uint8_t game_progress;
  uint8_t navi_hp_state;
  uint16_t crcSum;
} __attribute__((packed));

struct ReceivePacket
{
  uint8_t header = 0x5A;
  NaviReceivePacket navi;
  VisionReceivePacket vision;
  uint16_t checksum = 0;
} __attribute__((packed));

struct Navi_send_packet
{
  uint8_t header = 0xA6;
  // uint8_t navi_for_robot_status; // 0: spin, 1: navigate, 2: aiming
  // uint32_t sentry_devision_making; // Default: 0
  SentryDecision_e decision;
  float Vx;
  float Vy;
  float Wz;
  uint16_t crcSum;
} __attribute__((packed));

struct Vision_send_packet
{
  uint8_t header = 0xA5;
  uint8_t target : 2;
  uint8_t idNum : 3;
  uint8_t armorNum :3;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float vYaw;
  float r1;
  float r2;
  float dz;
  uint32_t sTimestamp;
  uint16_t firstPhase;
  uint16_t checkSum;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  Navi_send_packet navi;
  Vision_send_packet vision;
  uint16_t checksum = 0;
} __attribute__((packed));

template <typename PacketType>
inline PacketType fromVector(const std::vector<uint8_t> & data)
{
  PacketType packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename PacketType>
inline std::vector<uint8_t> toVector(const PacketType & data)
{
  std::vector<uint8_t> packet(sizeof(PacketType));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(PacketType), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
