#pragma once
#include <cstdint>
#include <cstring>

struct __attribute__((packed)) BleAdvRecord {
  uint32_t timestampMs;
  uint8_t  mac[6];
  int8_t   rssi;
  uint8_t  advType;
  uint8_t  payloadLen;
  uint32_t payloadHash;
  int8_t   txPower;
  uint16_t mfgId;
  uint8_t  appleType;
  uint8_t  extraFlags;
};
static_assert(sizeof(BleAdvRecord) == 22, "BleAdvRecord must be 22 bytes");

struct BleAdParsed {
  uint16_t appearance;
  uint16_t svcUuid;
  uint16_t svcUuid2;
  uint16_t mfgId;
  uint8_t  adFlags;
  uint8_t  appleType;
  uint8_t  appleDevType;
  uint8_t  appleStatus;
  uint8_t  appleInfo;
  uint8_t  airpodsLeft;
  uint8_t  airpodsRight;
  uint8_t  airpodsCase;
  uint8_t  hotspotBat;
};

struct BleDeviceSummary {
  uint8_t  mac[6];
  uint8_t  addrType;
  int8_t   rssiLast;
  int8_t   rssiMin;
  int8_t   rssiMax;
  uint8_t  advType;
  uint32_t payloadHash;
  uint8_t  payloadLen;
  int8_t   txPower;
  uint32_t firstSeenMs;
  uint32_t lastSeenMs;
  uint16_t hitCount;
  char     name[24];
  bool     connectable;
  uint16_t appearance;
  uint16_t primarySvcUuid;
  uint16_t secondarySvcUuid;
  uint16_t mfgCompanyId;
  uint8_t  appleMessageType;
  uint8_t  appleDeviceType;
  uint8_t  appleStatusByte;
  uint8_t  appleInfoByte;
  uint8_t  airpodsBattLeft;
  uint8_t  airpodsBattRight;
  uint8_t  airpodsBattCase;
  uint8_t  hotspotBattery;
  uint8_t  adFlags;
};
