#pragma once
// Protocol v1 for experiment-only PX4/MAVLink TUNNEL 42001.
// Identical implementation must be compiled by ROS and PX4; DO NOT use
// for real motion before PX4 firmware validation.
#include <cstdint>
#include <cmath>
#include <cstring>

namespace avoidance {
constexpr uint16_t kTunnelPayloadType=42001; // local experimental (>32767)
constexpr uint32_t kMagic=0x31445641U; // "AVD1" little endian
constexpr uint8_t kVersion=1;
enum Flags : uint8_t { VALID=1, OBSTACLE=2, STOP=4 };
#pragma pack(push,1)
struct AdvisoryWire {
  uint32_t magic;
  uint8_t version;
  uint8_t flags;
  uint16_t sequence;
  uint32_t session;
  float vx_limit_mps;
  float yaw_correction_radps;
  float front_clearance_m;
  float left_clearance_m;
  float right_clearance_m;
  uint32_t crc;
};
#pragma pack(pop)
static_assert(sizeof(AdvisoryWire)==36,"MAVLink tunnel packet size changed");
inline uint32_t crc32(const void* data, size_t len) {
  auto p=static_cast<const uint8_t*>(data);
  uint32_t crc=0xFFFFFFFFU;
  for(size_t i=0;i<len;++i) {
    crc^=p[i];
    for(int j=0;j<8;++j)
      crc=(crc>>1)^(0xEDB88320U & (0U-(crc&1U)));
  }
  return crc^0xFFFFFFFFU;
}
inline void seal(AdvisoryWire &p) {
  p.magic=kMagic;p.version=kVersion;
  p.crc=crc32(&p,sizeof(p)-sizeof(p.crc));
}
inline bool decode(const uint8_t* buf,size_t n,AdvisoryWire &out) {
  if(n!=sizeof(out)||!buf)return false;
  std::memcpy(&out,buf,n);
  if(out.magic!=kMagic||out.version!=kVersion||
     !(out.flags&VALID)|| (out.flags&~(VALID|OBSTACLE|STOP))!=0 ||
     out.session==0) return false;
  if(out.crc!=crc32(buf,n-sizeof(out.crc)))return false;
  if(!std::isfinite(out.vx_limit_mps)||out.vx_limit_mps<0.0f||out.vx_limit_mps>5.0f ||
     !std::isfinite(out.yaw_correction_radps)||std::abs(out.yaw_correction_radps)>3.0f ||
     !std::isfinite(out.front_clearance_m)||out.front_clearance_m<0.0f||
     !std::isfinite(out.left_clearance_m)||out.left_clearance_m<0.0f||
     !std::isfinite(out.right_clearance_m)||out.right_clearance_m<0.0f)return false;
  return true;
}
} // namespace avoidance
