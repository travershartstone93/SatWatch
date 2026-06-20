// BLE Network Analyzer — passive scanning + portal + SD binary logging

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <esp_crc.h>
#include "ble_types.h"

static const uint32_t BLE_RING_SIZE = 32768;
static const int      BLE_MAX_DEVICES = 256;
#define BLE_SCAN_LOG_PATH "/ble_scan.bin"

// ── State ───────────────────────────────────────────────────────────────────

static uint8_t*          s_bleRingBuf = nullptr;
static volatile uint32_t s_bleRingHead = 0;
static volatile uint32_t s_bleRingTail = 0;

static BleDeviceSummary* s_bleDevices = nullptr;
static int               s_bleDeviceCount = 0;
static SemaphoreHandle_t s_bleDeviceMutex = nullptr;

static BLEScan*          s_bleScan = nullptr;
static TaskHandle_t      s_bleScanTaskHandle = nullptr;
static volatile bool     s_bleScanRunning = false;
static volatile bool     s_bleScanShouldRun = false;
static uint32_t          s_bleTotalRecords = 0;
static uint32_t          s_bleScanStartMs = 0;
static bool              s_bleInitDone = false;

// ── Helpers ─────────────────────────────────────────────────────────────────

static bool bleIsPluggedIn() {
  int cs = readAxp2101ChargeState();
  if (cs < 0) return false;
  uint8_t phase = (cs >> 1) & 0x07;
  return (cs & 0x01) || phase <= 0x03;
}

static int bleBatPct() {
  return readAxp2101BatPct();
}

static void bleRingPush(const void* data, size_t len) {
  uint32_t head = s_bleRingHead;
  uint32_t next = (head + len) % BLE_RING_SIZE;
  if (next == s_bleRingTail) return;  // full — drop
  uint32_t tailRoom = BLE_RING_SIZE - head;
  if (tailRoom >= len) {
    memcpy(s_bleRingBuf + head, data, len);
  } else {
    memcpy(s_bleRingBuf + head, data, tailRoom);
    memcpy(s_bleRingBuf, (const uint8_t*)data + tailRoom, len - tailRoom);
  }
  s_bleRingHead = next;
}

static int bleDeviceFindByMac(const uint8_t* mac) {
  for (int i = 0; i < s_bleDeviceCount; i++) {
    if (memcmp(s_bleDevices[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

static int bleDeviceEvictLru() {
  if (s_bleDeviceCount == 0) return -1;
  int oldest = 0;
  for (int i = 1; i < s_bleDeviceCount; i++) {
    if (s_bleDevices[i].lastSeenMs < s_bleDevices[oldest].lastSeenMs) oldest = i;
  }
  if (oldest < s_bleDeviceCount - 1)
    s_bleDevices[oldest] = s_bleDevices[s_bleDeviceCount - 1];
  s_bleDeviceCount--;
  return s_bleDeviceCount;
}

static void bleDeviceUpdate(const uint8_t* mac, uint8_t addrType, int8_t rssi,
                            uint8_t advType, uint8_t payloadLen, uint32_t payloadHash,
                            int8_t txPower, const char* name,
                            const BleAdParsed& p) {
  if (!s_bleDeviceMutex || xSemaphoreTake(s_bleDeviceMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  uint32_t now = millis();
  int idx = bleDeviceFindByMac(mac);
  if (idx >= 0) {
    BleDeviceSummary& d = s_bleDevices[idx];
    d.rssiLast = rssi;
    if (rssi < d.rssiMin) d.rssiMin = rssi;
    if (rssi > d.rssiMax) d.rssiMax = rssi;
    d.advType = advType;
    d.lastSeenMs = now;
    d.hitCount++;
    if (name[0] && !d.name[0]) {
      strncpy(d.name, name, sizeof(d.name) - 1);
      d.name[sizeof(d.name) - 1] = '\0';
    }
    if (p.appearance && !d.appearance) d.appearance = p.appearance;
    if (p.svcUuid && !d.primarySvcUuid) d.primarySvcUuid = p.svcUuid;
    if (p.svcUuid2 && !d.secondarySvcUuid) d.secondarySvcUuid = p.svcUuid2;
    if (p.mfgId && !d.mfgCompanyId) d.mfgCompanyId = p.mfgId;
    if (p.adFlags) d.adFlags = p.adFlags;
    // Apple fields: always update (transient state)
    if (p.appleType) {
      d.appleMessageType = p.appleType;
      if (p.appleDevType) d.appleDeviceType = p.appleDevType;
      d.appleStatusByte = p.appleStatus;
      d.appleInfoByte = p.appleInfo;
    }
    if (p.airpodsLeft != 0xFF) d.airpodsBattLeft = p.airpodsLeft;
    if (p.airpodsRight != 0xFF) d.airpodsBattRight = p.airpodsRight;
    if (p.airpodsCase != 0xFF) d.airpodsBattCase = p.airpodsCase;
    if (p.hotspotBat != 0xFF) d.hotspotBattery = p.hotspotBat;
  } else {
    if (s_bleDeviceCount >= BLE_MAX_DEVICES) bleDeviceEvictLru();
    idx = s_bleDeviceCount++;
    BleDeviceSummary& d = s_bleDevices[idx];
    memset(&d, 0, sizeof(d));
    memcpy(d.mac, mac, 6);
    d.addrType = addrType;
    d.rssiLast = d.rssiMin = d.rssiMax = rssi;
    d.advType = advType;
    d.payloadHash = payloadHash;
    d.payloadLen = payloadLen;
    d.txPower = txPower;
    d.firstSeenMs = now;
    d.lastSeenMs = now;
    d.hitCount = 1;
    d.connectable = (advType == 0);
    d.appearance = p.appearance;
    d.primarySvcUuid = p.svcUuid;
    d.secondarySvcUuid = p.svcUuid2;
    d.mfgCompanyId = p.mfgId;
    d.adFlags = p.adFlags;
    d.appleMessageType = p.appleType;
    d.appleDeviceType = p.appleDevType;
    d.appleStatusByte = p.appleStatus;
    d.appleInfoByte = p.appleInfo;
    d.airpodsBattLeft = p.airpodsLeft;
    d.airpodsBattRight = p.airpodsRight;
    d.airpodsBattCase = p.airpodsCase;
    d.hotspotBattery = p.hotspotBat;
    strncpy(d.name, name, sizeof(d.name) - 1);
    d.name[sizeof(d.name) - 1] = '\0';
  }
  xSemaphoreGive(s_bleDeviceMutex);
}

// ── Scan callback ───────────────────────────────────────────────────────────

static void bleParseAppleContinuity(const uint8_t* data, uint8_t dataLen,
                                     BleAdParsed* out) {
  uint8_t pos = 0;
  while (pos + 2 <= dataLen) {
    uint8_t msgType = data[pos];
    uint8_t msgLen  = data[pos + 1];
    if (pos + 2 + msgLen > dataLen) break;
    const uint8_t* md = data + pos + 2;

    out->appleType = msgType;

    switch (msgType) {
      case 0x10: // Nearby Info
        if (msgLen >= 2) {
          out->appleStatus = md[0];
          out->appleInfo   = md[1];
          out->appleDevType = (md[1] >> 4) & 0x0F;
        }
        break;
      case 0x07: // Proximity Pairing (AirPods)
        if (msgLen >= 7) {
          uint8_t rawLeft  = (md[5] >> 4) & 0x0F;
          uint8_t rawRight = md[5] & 0x0F;
          uint8_t rawCase  = (md[6] >> 4) & 0x0F;
          out->airpodsLeft  = (rawLeft  <= 10) ? rawLeft  * 10 : 0xFF;
          out->airpodsRight = (rawRight <= 10) ? rawRight * 10 : 0xFF;
          out->airpodsCase  = (rawCase  <= 10) ? rawCase  * 10 : 0xFF;
          out->appleDevType = 6; // AirPods
        }
        break;
      case 0x0D: // Instant Hotspot
        if (msgLen >= 5) {
          out->hotspotBat = md[4];
        }
        break;
      default:
        break;
    }
    pos += 2 + msgLen;
  }
}

static void bleParseAdPayload(const uint8_t* payload, size_t len,
                               BleAdParsed* out) {
  memset(out, 0, sizeof(*out));
  out->airpodsLeft = out->airpodsRight = out->airpodsCase = 0xFF;
  out->hotspotBat = 0xFF;

  size_t pos = 0;
  int svcUuidCount = 0;
  while (pos < len) {
    uint8_t fieldLen = payload[pos];
    if (fieldLen == 0 || pos + 1 + fieldLen > len) break;
    uint8_t adType = payload[pos + 1];
    const uint8_t* fieldData = payload + pos + 2;
    uint8_t dataLen = fieldLen - 1;

    switch (adType) {
      case 0x01: // Flags
        if (dataLen >= 1) out->adFlags = fieldData[0];
        break;
      case 0x19: // Appearance
        if (dataLen >= 2) out->appearance = fieldData[0] | (fieldData[1] << 8);
        break;
      case 0x02: // Incomplete 16-bit service UUIDs
      case 0x03: { // Complete 16-bit service UUIDs
        for (uint8_t i = 0; i + 1 < dataLen && svcUuidCount < 2; i += 2) {
          uint16_t uuid = fieldData[i] | (fieldData[i + 1] << 8);
          if (svcUuidCount == 0) out->svcUuid = uuid;
          else out->svcUuid2 = uuid;
          svcUuidCount++;
        }
        break;
      }
      case 0xFF: // Manufacturer Specific Data
        if (dataLen >= 2) {
          out->mfgId = fieldData[0] | (fieldData[1] << 8);
          if (out->mfgId == 0x004C && dataLen > 2) {
            bleParseAppleContinuity(fieldData + 2, dataLen - 2, out);
          }
        }
        break;
    }
    pos += 1 + fieldLen;
  }
}

class BleScanCb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (!s_bleRingBuf) return;

    BleAdvRecord rec;
    rec.timestampMs = millis();

    BLEAddress addr = advertisedDevice.getAddress();
    memcpy(rec.mac, addr.getNative(), 6);

    rec.rssi = (int8_t)advertisedDevice.getRSSI();
    rec.advType = advertisedDevice.getAdvType();
    rec.payloadLen = (uint8_t)advertisedDevice.getPayloadLength();
    rec.payloadHash = esp_crc32_le(0, advertisedDevice.getPayload(), advertisedDevice.getPayloadLength());
    rec.txPower = advertisedDevice.haveTXPower() ? (int8_t)advertisedDevice.getTXPower() : 0x7F;

    BleAdParsed parsed;
    bleParseAdPayload(advertisedDevice.getPayload(), advertisedDevice.getPayloadLength(), &parsed);

    rec.mfgId = parsed.mfgId;
    rec.appleType = parsed.appleType;

    // Pack extraFlags
    rec.extraFlags = 0;
    rec.extraFlags |= (parsed.appleDevType & 0x0F) << 4;
    if (advertisedDevice.getAdvType() == 0) rec.extraFlags |= 0x08; // connectable
    if (advertisedDevice.haveName()) rec.extraFlags |= 0x04;
    if (parsed.adFlags & 0x04) rec.extraFlags |= 0x01;       // LE-only
    else if (parsed.adFlags & 0x02) rec.extraFlags |= 0x02;  // BR/EDR+LE

    bleRingPush(&rec, sizeof(rec));

    char nameBuf[24] = {0};
    if (advertisedDevice.haveName()) {
      String n = advertisedDevice.getName().c_str();
      int j = 0;
      for (int i = 0; i < (int)n.length() && j < 23; i++) {
        char c = n.charAt(i);
        if (c >= 0x20 && c < 0x7F) nameBuf[j++] = c;
      }
      nameBuf[j] = 0;
    }
    bleDeviceUpdate(rec.mac, advertisedDevice.getAddressType(), rec.rssi,
                    rec.advType, rec.payloadLen, rec.payloadHash, rec.txPower,
                    nameBuf, parsed);
  }
};
static BleScanCb s_bleScanCb;

// ── SD flush ────────────────────────────────────────────────────────────────

static void bleFlushRingToSd() {
  uint32_t head = s_bleRingHead;
  uint32_t tail = s_bleRingTail;
  if (head == tail) return;

  uint32_t avail = (head >= tail) ? (head - tail) : (BLE_RING_SIZE - tail + head);
  if (avail < 512) return;

  File f = SD.open(BLE_SCAN_LOG_PATH, FILE_APPEND);
  if (!f) return;

  if (f.size() == 0) {
    uint8_t hdr[16] = {0};
    memcpy(hdr, "BLSC", 4);
    uint16_t ver = 3;
    uint16_t recSz = sizeof(BleAdvRecord);
    uint32_t boot = s_diagBootNum;
    memcpy(hdr + 4, &ver, 2);
    memcpy(hdr + 6, &recSz, 2);
    memcpy(hdr + 8, &boot, 4);
    f.write(hdr, 16);
  }

  uint8_t buf[660]; // 30 records * 22 bytes
  while (s_bleRingTail != s_bleRingHead) {
    uint32_t t = s_bleRingTail;
    uint32_t h = s_bleRingHead;
    uint32_t chunk;
    if (h > t) {
      chunk = h - t;
    } else {
      chunk = BLE_RING_SIZE - t;
    }
    if (chunk > sizeof(buf)) chunk = sizeof(buf);
    memcpy(buf, s_bleRingBuf + t, chunk);
    s_bleRingTail = (t + chunk) % BLE_RING_SIZE;
    f.write(buf, chunk);
    s_bleTotalRecords += chunk / sizeof(BleAdvRecord);
  }
  f.close();
}

// ── Scanner RTOS task ───────────────────────────────────────────────────────

static void bleScanTask(void* param) {
  (void)param;

  if (!s_bleInitDone) {
    BLEDevice::init("");
    s_bleInitDone = true;
    uint32_t freeInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    appendDiagLog("BLE", "msg=init heap_int=%u\n", (unsigned)freeInternal);
    if (freeInternal < 20000) {
      appendDiagLog("BLE", "msg=abort_low_heap heap_int=%u\n", (unsigned)freeInternal);
      BLEDevice::deinit(true);
      s_bleInitDone = false;
      s_bleScanRunning = false;
      s_bleScanTaskHandle = nullptr;
      vTaskDelete(nullptr);
      return;
    }
  }

  s_bleScan = BLEDevice::getScan();
  s_bleScan->setAdvertisedDeviceCallbacks(&s_bleScanCb, true);
  s_bleScan->setActiveScan(true);

  s_bleScanRunning = true;
  s_bleScanStartMs = millis();
  appendDiagLog("BLE", "msg=scan_start\n");

  while (s_bleScanShouldRun) {
    bool plugged = bleIsPluggedIn();
    int bat = bleBatPct();

    if (!plugged && bat >= 0 && bat < 15) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    if (plugged) {
      s_bleScan->setInterval(100);
      s_bleScan->setWindow(100);
    } else {
      s_bleScan->setInterval(10000);
      s_bleScan->setWindow(3000);
    }

    s_bleScan->start(5, false);
    s_bleScan->clearResults();

    bleFlushRingToSd();

    vTaskDelay(pdMS_TO_TICKS(200));
  }

  s_bleScan->stop();
  bleFlushRingToSd();
  s_bleScanRunning = false;
  appendDiagLog("BLE", "msg=scan_stop records=%u devices=%d\n",
                (unsigned)s_bleTotalRecords, s_bleDeviceCount);

  s_bleScanTaskHandle = nullptr;
  vTaskDelete(nullptr);
}

// ── Public start/stop ───────────────────────────────────────────────────────

static void startBleScanning() {
  if (s_bleScanTaskHandle) return;

  if (!s_bleRingBuf) {
    s_bleRingBuf = (uint8_t*)heap_caps_malloc(BLE_RING_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_bleRingBuf) {
      appendDiagLog("BLE", "msg=ring_alloc_fail\n");
      return;
    }
  }
  if (!s_bleDevices) {
    s_bleDevices = (BleDeviceSummary*)heap_caps_calloc(BLE_MAX_DEVICES, sizeof(BleDeviceSummary), MALLOC_CAP_SPIRAM);
    if (!s_bleDevices) {
      appendDiagLog("BLE", "msg=dev_alloc_fail\n");
      return;
    }
  }
  if (!s_bleDeviceMutex) {
    s_bleDeviceMutex = xSemaphoreCreateMutex();
  }

  s_bleRingHead = 0;
  s_bleRingTail = 0;
  s_bleScanShouldRun = true;

  xTaskCreatePinnedToCore(bleScanTask, "ble_scan", 8192, nullptr, 1, &s_bleScanTaskHandle, 0);
}

static void stopBleScanningImpl(bool deinit) {
  if (!s_bleScanRunning && !s_bleScanTaskHandle && !deinit) return;
  s_bleScanShouldRun = false;
  if (s_bleScan) s_bleScan->stop();
  for (int i = 0; i < 80 && s_bleScanRunning; i++) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (deinit && s_bleInitDone) {
    for (int i = 0; i < 20 && s_bleScanTaskHandle; i++) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    BLEDevice::deinit(true);
    s_bleInitDone = false;
  }
}

static void stopBleScanning() { stopBleScanningImpl(false); }
static void stopBleScanningFull() { stopBleScanningImpl(true); }

// ── Portal: JSON endpoint ───────────────────────────────────────────────────

static void handleBleJson() {
  String json;
  json.reserve(6144);
  json = "{\"scanning\":";
  json += s_bleScanRunning ? "true" : "false";
  json += ",\"profile\":\"";
  if (!s_bleScanRunning) json += "off";
  else if (bleIsPluggedIn()) json += "plugged";
  else json += "battery";
  json += "\",\"devCount\":";
  json += s_bleDeviceCount;
  json += ",\"totalRecords\":";
  json += s_bleTotalRecords;
  json += ",\"uptimeS\":";
  json += s_bleScanRunning ? (millis() - s_bleScanStartMs) / 1000 : 0;
  json += ",\"devices\":[";

  if (s_bleDeviceMutex && xSemaphoreTake(s_bleDeviceMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (int i = 0; i < s_bleDeviceCount; i++) {
      if (i > 0) json += ',';
      BleDeviceSummary& d = s_bleDevices[i];
      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
               d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5]);
      json += "{\"mac\":\"";
      json += macStr;
      json += "\",\"name\":\"";
      for (int c = 0; d.name[c] && c < 23; c++) {
        char ch = d.name[c];
        if (ch < 0x20 || ch >= 0x7F) continue;
        if (ch == '"') json += "\\\"";
        else if (ch == '\\') json += "\\\\";
        else json += ch;
      }
      json += "\",\"rssi\":";
      json += (int)d.rssiLast;
      json += ",\"rssiMin\":";
      json += (int)d.rssiMin;
      json += ",\"rssiMax\":";
      json += (int)d.rssiMax;
      json += ",\"type\":";
      json += (int)d.advType;
      json += ",\"hits\":";
      json += d.hitCount;
      json += ",\"age\":";
      json += (millis() - d.lastSeenMs) / 1000;
      json += ",\"txPower\":";
      json += (int)d.txPower;
      json += ",\"payloadHash\":\"0x";
      char hashStr[9];
      snprintf(hashStr, sizeof(hashStr), "%08X", d.payloadHash);
      json += hashStr;
      json += "\",\"payloadLen\":";
      json += d.payloadLen;
      json += ",\"connectable\":";
      json += d.connectable ? "true" : "false";
      json += ",\"appearance\":";
      json += d.appearance;
      json += ",\"svcUuid\":";
      json += d.primarySvcUuid;
      json += ",\"mfgId\":";
      json += d.mfgCompanyId;
      json += ",\"appleType\":";
      json += d.appleMessageType;
      json += ",\"appleDevType\":";
      json += d.appleDeviceType;
      json += ",\"appleStatus\":";
      json += d.appleStatusByte;
      json += ",\"adFlags\":";
      json += d.adFlags;
      json += ",\"airpodsBatL\":";
      json += d.airpodsBattLeft;
      json += ",\"airpodsBatR\":";
      json += d.airpodsBattRight;
      json += ",\"airpodsBatC\":";
      json += d.airpodsBattCase;
      json += ",\"hotspotBat\":";
      json += d.hotspotBattery;
      json += '}';
    }
    xSemaphoreGive(s_bleDeviceMutex);
  }
  json += "]}";
  s_wifiPortalServer.send(200, "application/json", json);
}

// ── Portal: download / clear endpoints ──────────────────────────────────────

static void handleBleDownload() {
  File f = SD.open(BLE_SCAN_LOG_PATH, FILE_READ);
  if (!f) {
    s_wifiPortalServer.send(404, "text/plain", "no scan log");
    return;
  }
  size_t sz = f.size();
  s_wifiPortalServer.sendHeader("Content-Disposition", "attachment; filename=\"ble_scan.bin\"");
  s_wifiPortalServer.setContentLength(sz);
  s_wifiPortalServer.send(200, "application/octet-stream", "");
  uint8_t buf[1024];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    if (n == 0) break;
    s_wifiPortalServer.client().write(buf, n);
  }
  f.close();
}

static void handleBleClearLogs() {
  if (!portalTokenValid()) return;
  SD.remove(BLE_SCAN_LOG_PATH);
  File root = SD.open("/sd");
  if (root && root.isDirectory()) {
    File f = root.openNextFile();
    while (f) {
      String fname = f.name();
      f.close();
      if (fname.startsWith("ble_gatt_") && fname.endsWith(".log")) {
        SD.remove(String("/sd/") + fname);
      }
      f = root.openNextFile();
    }
    root.close();
  }
  s_bleTotalRecords = 0;
  s_wifiPortalServer.send(200, "text/plain", "BLE logs cleared");
}

// ── Portal: HTML page ───────────────────────────────────────────────────────

static const char kBleAnalyzerHtml_1[] PROGMEM =
  "<!DOCTYPE html><html><head>"
  "<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>BLE Analyzer</title><style>"
  "*{margin:0;padding:0;box-sizing:border-box}"
  "body{font-family:-apple-system,sans-serif;background:#0f172a;color:#e2e8f0;padding:1em;max-width:900px;margin:0 auto}"
  "h2{color:#38bdf8;text-align:center;margin-bottom:.3em}"
  ".status{text-align:center;color:#94a3b8;font-size:.85em;margin-bottom:.8em}"
  ".status .on{color:#4ade80}.status .off{color:#f87171}"
  "table{width:100%;border-collapse:collapse;font-size:.82em}"
  "thead{position:sticky;top:0;background:#1e293b}"
  "th{padding:6px 4px;text-align:left;color:#38bdf8;border-bottom:2px solid #334155;cursor:pointer;white-space:nowrap;user-select:none}"
  "th:hover{color:#7dd3fc}"
  "td{padding:5px 4px;border-bottom:1px solid #1e293b;white-space:nowrap}"
  "tr:hover{background:#1e293b}"
  ".rssi-bar{display:inline-block;height:10px;border-radius:2px;min-width:3px}"
  ".type-c{color:#4ade80}.type-n{color:#94a3b8}.type-s{color:#fbbf24}"
  ".apple{color:#a78bfa;font-size:.8em}"
  ".dl-link{color:#38bdf8;text-decoration:none;font-size:.85em}.dl-link:hover{text-decoration:underline}"
  ".actions{margin-top:1em;display:flex;gap:10px;justify-content:center;flex-wrap:wrap}"
  ".btn{padding:8px 16px;border-radius:6px;border:none;cursor:pointer;font-size:.85em}"
  ".btn-dl{background:#1e40af;color:#fff}.btn-dl:hover{background:#2563eb}"
  ".btn-clr{background:#7f1d1d;color:#fff}.btn-clr:hover{background:#991b1b}"
  ".btn-toggle{background:#065f46;color:#fff}.btn-toggle:hover{background:#047857}"
  ".expand{color:#64748b;font-size:.75em;font-family:monospace;display:none;padding:4px 4px 4px 20px}"
  "tr.open+tr.expand{display:table-row}"
  "</style></head><body>"
  "<h2>&#128225; BLE Analyzer</h2>"
  "<div class='status' id='status'>Loading...</div>"
  "<div style='overflow-x:auto;max-height:70vh;overflow-y:auto'>"
  "<table><thead><tr>"
  "<th onclick='sortBy(\"name\")'>Name</th>"
  "<th onclick='sortBy(\"rssi\")'>RSSI</th>"
  "<th onclick='sortBy(\"type\")'>Type</th>"
  "<th>Device</th>"
  "<th onclick='sortBy(\"hits\")'>Hits</th>"
  "<th onclick='sortBy(\"age\")'>Age</th>"
  "</tr></thead><tbody id='tbody'></tbody></table></div>"
  "<div class='actions'>"
  "<button class='btn btn-toggle' id='toggleBtn' onclick='toggleScan()'>Start Scan</button>"
  "<a class='btn btn-dl' href='/bledl'>Download Log</a>"
  "<button class='btn btn-clr' onclick='clearLogs()'>Clear Logs</button>"
  "</div>"
  "<script>";

static const char kBleAnalyzerHtml_2[] PROGMEM =
  "var K=new URLSearchParams(location.search).get('k')||'';"
  "var sortKey='rssi',sortAsc=false,devs=[];"
  "var sortBy=function(k){if(sortKey===k)sortAsc=!sortAsc;else{sortKey=k;sortAsc=k==='name';}render();};"
  "var rssiColor=function(r){if(r>-60)return'#4ade80';if(r>-75)return'#fbbf24';if(r>-85)return'#fb923c';return'#f87171';};"
  "var rssiWidth=function(r){var p=Math.max(0,Math.min(100,(r+100)*1.5));return Math.max(3,p);};"
  "var typeLabel=function(t){if(t===0)return'<span class=\"type-c\">C</span>';if(t===4)return'<span class=\"type-s\">S</span>';return'<span class=\"type-n\">N</span>';};"
  "var fmtAge=function(s){if(s<60)return s+'s';if(s<3600)return Math.floor(s/60)+'m';return Math.floor(s/3600)+'h';};"
  "var ADN={1:'iPhone',2:'iPad',3:'Mac',4:'Watch',6:'AirPods',9:'AppleTV',10:'HomePod',14:'HomePod'};"
  "var AMT={2:'iBeacon',5:'AirDrop',6:'HomeKit',7:'AirPods',8:'Siri',9:'AirPlay',12:'Handoff',13:'Hotspot',14:'WiFi',15:'Nearby',16:'NearbyInfo',18:'FindMy'};"
  "var AAC={0:'Idle',1:'Audio',2:'Call',3:'Driving'};"
  "var appleInfo=function(d){"
  "if(!d.appleType)return'';"
  "var s=ADN[d.appleDevType]||'';"
  "if(d.appleType===16){var a=AAC[d.appleStatus&0x0F];if(a)s+=' ('+a+')';}"
  "else if(d.appleType===7){s='AirPods';if(d.airpodsBatL<255)s+=' L:'+d.airpodsBatL+'%';if(d.airpodsBatR<255)s+=' R:'+d.airpodsBatR+'%';if(d.airpodsBatC<255)s+=' C:'+d.airpodsBatC+'%';}"
  "else if(d.appleType===13&&d.hotspotBat<255){s+=' Bat:'+d.hotspotBat+'%';}"
  "else{var mt=AMT[d.appleType];if(mt)s+=s?' '+mt:mt;}"
  "return s?'<span class=\"apple\">'+s+'</span>':'';};"
  "var render=function(){"
  "var s=devs.slice();"
  "s.sort(function(a,b){"
  "var av=a[sortKey],bv=b[sortKey];"
  "if(typeof av==='string')av=av.toLowerCase();"
  "if(typeof bv==='string')bv=bv.toLowerCase();"
  "if(av<bv)return sortAsc?-1:1;if(av>bv)return sortAsc?1:-1;return 0;"
  "});"
  "var h='';"
  "for(var i=0;i<s.length;i++){"
  "var d=s[i];"
  "var nm=d.name||d.mac;"
  "h+='<tr onclick=\"this.classList.toggle(\\'open\\')\">';"
  "h+='<td>'+nm+'</td>';"
  "h+='<td><span class=\"rssi-bar\" style=\"background:'+rssiColor(d.rssi)+';width:'+rssiWidth(d.rssi)+'px\"></span> '+d.rssi+'</td>';"
  "h+='<td>'+typeLabel(d.type)+'</td>';"
  "h+='<td>'+appleInfo(d)+'</td>';"
  "h+='<td>'+d.hits+'</td>';"
  "h+='<td>'+fmtAge(d.age)+'</td>';"
  "h+='</tr>';"
  "h+='<tr class=\"expand\"><td colspan=6>';"
  "h+='MAC: '+d.mac+' | Hash: '+d.payloadHash+' | Len: '+d.payloadLen;"
  "h+=' | TX: '+(d.txPower===127?'N/A':d.txPower+' dBm');"
  "h+=' | RSSI: '+d.rssiMax+'/'+d.rssiMin;"
  "if(d.mfgId)h+=' | MfgID: 0x'+d.mfgId.toString(16).toUpperCase();"
  "if(d.appleType)h+=' | Apple: '+(AMT[d.appleType]||'0x'+d.appleType.toString(16));"
  "if(d.adFlags)h+=' | Flags: 0x'+d.adFlags.toString(16);"
  "h+='</td></tr>';}"
  "document.getElementById('tbody').innerHTML=h;"
  "};"
  "var poll=function(){"
  "var x=new XMLHttpRequest();"
  "x.open('GET','/blejson');x.timeout=3000;"
  "x.onload=function(){"
  "var j=JSON.parse(x.responseText);"
  "devs=j.devices;"
  "var st=j.scanning?'<span class=\"on\">Scanning</span> ('+j.profile+')':'<span class=\"off\">Stopped</span>';"
  "st+=' &mdash; '+j.devCount+' devices, '+j.totalRecords+' records';"
  "if(j.uptimeS>0)st+=', '+fmtAge(j.uptimeS)+' uptime';"
  "document.getElementById('status').innerHTML=st;"
  "document.getElementById('toggleBtn').textContent=j.scanning?'Stop Scan':'Start Scan';"
  "render();};"
  "x.onerror=function(){document.getElementById('status').innerHTML='<span class=\"off\">Offline</span>';};"
  "x.send();};"
  "var toggleScan=function(){"
  "var x=new XMLHttpRequest();x.open('GET','/bletoggle?k='+K);"
  "x.onload=function(){poll();};x.send();};"
  "var clearLogs=function(){"
  "if(!confirm('Clear all BLE logs?'))return;"
  "var x=new XMLHttpRequest();x.open('GET','/bleclear?k='+K);"
  "x.onload=function(){poll();};x.send();};"
  "setInterval(poll,2000);poll();"
  "</script></body></html>";

static void handleBleAnalyzerPage() {
  String page;
  page.reserve(strlen_P(kBleAnalyzerHtml_1) + strlen_P(kBleAnalyzerHtml_2) + 8);
  page += FPSTR(kBleAnalyzerHtml_1);
  page += FPSTR(kBleAnalyzerHtml_2);
  s_wifiPortalServer.send(200, "text/html", page);
}

// ── Handler registration (called from main .ino) ────────────────────────────

static void handleBleToggle() {
  if (!portalTokenValid()) return;
  if (s_bleScanRunning) {
    stopBleScanning();
    s_wifiPortalServer.send(200, "text/plain", "BLE scanning stopped");
  } else {
    startBleScanning();
    s_wifiPortalServer.send(200, "text/plain", "BLE scanning started");
  }
}

// ── GATT Probe: connect to a device and read name + device info ────────────

static void handleBleProbe() {
  if (!portalTokenValid()) return;

  String macArg = s_wifiPortalServer.arg("mac");
  if (macArg.length() < 17) {
    s_wifiPortalServer.send(400, "application/json", "{\"error\":\"missing mac param\"}");
    return;
  }

  bool wasScanning = s_bleScanRunning;
  if (wasScanning) stopBleScanningFull();
  else if (s_bleInitDone) { BLEDevice::deinit(true); s_bleInitDone = false; }
  vTaskDelay(pdMS_TO_TICKS(300));

  BLEDevice::init("SatWatch");
  s_bleInitDone = true;
  BLEClient* client = BLEDevice::createClient();

  String json = "{\"mac\":\"" + macArg + "\"";
  BLEAddress addr(macArg.c_str());

  // Try random first (most BLE devices use random), then public
  bool connected = client->connect(addr, BLE_ADDR_RANDOM, 5000);
  if (!connected) connected = client->connect(addr, BLE_ADDR_PUBLIC, 5000);

  if (!connected) {
    json += ",\"connected\":false}";
    delete client;
    if (wasScanning) startBleScanning();
    s_wifiPortalServer.send(200, "application/json", json);
    return;
  }

  json += ",\"connected\":true";

  // GAP Service (0x1800) — Device Name (0x2A00)
  BLERemoteService* gapSvc = client->getService(BLEUUID((uint16_t)0x1800));
  if (gapSvc) {
    BLERemoteCharacteristic* nameChr = gapSvc->getCharacteristic(BLEUUID((uint16_t)0x2A00));
    if (nameChr && nameChr->canRead()) {
      String val = nameChr->readValue().c_str();
      val.replace("\"", "\\\"");
      json += ",\"gapName\":\"" + val + "\"";
    }
    BLERemoteCharacteristic* appearChr = gapSvc->getCharacteristic(BLEUUID((uint16_t)0x2A01));
    if (appearChr && appearChr->canRead()) {
      uint16_t app = appearChr->readUInt16();
      json += ",\"appearance\":" + String(app);
    }
  }

  // Device Information Service (0x180A)
  BLERemoteService* disSvc = client->getService(BLEUUID((uint16_t)0x180A));
  if (disSvc) {
    struct { uint16_t uuid; const char* key; } chars[] = {
      {0x2A24, "modelNumber"},
      {0x2A25, "serialNumber"},
      {0x2A26, "fwRevision"},
      {0x2A27, "hwRevision"},
      {0x2A28, "swRevision"},
      {0x2A29, "manufacturer"},
    };
    for (auto& c : chars) {
      BLERemoteCharacteristic* chr = disSvc->getCharacteristic(BLEUUID(c.uuid));
      if (chr && chr->canRead()) {
        String val = chr->readValue().c_str();
        val.replace("\"", "\\\"");
        if (val.length() > 0) {
          json += ",\"";
          json += c.key;
          json += "\":\"";
          json += val;
          json += "\"";
        }
      }
    }
  }

  // List all services for discovery
  auto* svcMap = client->getServices();
  if (svcMap && !svcMap->empty()) {
    json += ",\"services\":[";
    bool first = true;
    for (auto& kv : *svcMap) {
      if (!first) json += ",";
      json += "\"" + String(kv.second->getUUID().toString().c_str()) + "\"";
      first = false;
    }
    json += "]";
  }

  client->disconnect();
  delete client;

  json += "}";

  // Update device name in our summary if we got one
  int gapNameIdx = json.indexOf("\"gapName\":\"");
  if (gapNameIdx > 0) {
    int nameStart = gapNameIdx + 11;
    int nameEnd = json.indexOf("\"", nameStart);
    if (nameEnd > nameStart && nameEnd - nameStart < 24) {
      String gapName = json.substring(nameStart, nameEnd);
      if (s_bleDeviceMutex && xSemaphoreTake(s_bleDeviceMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < s_bleDeviceCount; i++) {
          char dm[18];
          snprintf(dm, sizeof(dm), "%02X:%02X:%02X:%02X:%02X:%02X",
                   s_bleDevices[i].mac[0], s_bleDevices[i].mac[1], s_bleDevices[i].mac[2],
                   s_bleDevices[i].mac[3], s_bleDevices[i].mac[4], s_bleDevices[i].mac[5]);
          if (macArg.equalsIgnoreCase(dm)) {
            gapName.toCharArray(s_bleDevices[i].name, sizeof(s_bleDevices[i].name));
            break;
          }
        }
        xSemaphoreGive(s_bleDeviceMutex);
      }
    }
  }

  if (wasScanning) startBleScanning();
  s_wifiPortalServer.send(200, "application/json", json);
}

// Probe all unnamed devices
static void handleBleProbeAll() {
  if (!portalTokenValid()) return;

  bool wasScanning = s_bleScanRunning;
  if (wasScanning) stopBleScanningFull();
  else if (s_bleInitDone) { BLEDevice::deinit(true); s_bleInitDone = false; }
  vTaskDelay(pdMS_TO_TICKS(300));

  BLEDevice::init("SatWatch");
  s_bleInitDone = true;

  String json = "{\"probed\":[";
  int probed = 0;

  if (s_bleDeviceMutex && xSemaphoreTake(s_bleDeviceMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    int devCount = s_bleDeviceCount;
    struct { uint8_t mac[6]; uint8_t addrType; } targets[64];
    int targetCount = 0;
    for (int i = 0; i < devCount && targetCount < 64; i++) {
      if (s_bleDevices[i].name[0] == 0) {
        memcpy(targets[targetCount].mac, s_bleDevices[i].mac, 6);
        targets[targetCount].addrType = s_bleDevices[i].addrType;
        targetCount++;
      }
    }
    xSemaphoreGive(s_bleDeviceMutex);

    for (int t = 0; t < targetCount; t++) {
      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
               targets[t].mac[0], targets[t].mac[1], targets[t].mac[2],
               targets[t].mac[3], targets[t].mac[4], targets[t].mac[5]);

      BLEClient* client = BLEDevice::createClient();
      BLEAddress addr(macStr);

      uint8_t atype = (targets[t].addrType == 1) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
      bool connected = client->connect(addr, atype, 3000);

      if (connected) {
        if (probed > 0) json += ",";
        json += "{\"mac\":\"";
        json += macStr;
        json += "\"";

        BLERemoteService* gapSvc = client->getService(BLEUUID((uint16_t)0x1800));
        if (gapSvc) {
          BLERemoteCharacteristic* nameChr = gapSvc->getCharacteristic(BLEUUID((uint16_t)0x2A00));
          if (nameChr && nameChr->canRead()) {
            String val = nameChr->readValue().c_str();
            val.replace("\"", "\\\"");
            json += ",\"gapName\":\"" + val + "\"";
            if (val.length() > 0 && val.length() < 24) {
              if (s_bleDeviceMutex && xSemaphoreTake(s_bleDeviceMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (int i = 0; i < s_bleDeviceCount; i++) {
                  if (memcmp(s_bleDevices[i].mac, targets[t].mac, 6) == 0) {
                    val.toCharArray(s_bleDevices[i].name, sizeof(s_bleDevices[i].name));
                    break;
                  }
                }
                xSemaphoreGive(s_bleDeviceMutex);
              }
            }
          }
        }

        BLERemoteService* disSvc = client->getService(BLEUUID((uint16_t)0x180A));
        if (disSvc) {
          BLERemoteCharacteristic* modelChr = disSvc->getCharacteristic(BLEUUID((uint16_t)0x2A24));
          if (modelChr && modelChr->canRead()) {
            String val = modelChr->readValue().c_str();
            val.replace("\"", "\\\"");
            json += ",\"model\":\"" + val + "\"";
          }
          BLERemoteCharacteristic* mfgChr = disSvc->getCharacteristic(BLEUUID((uint16_t)0x2A29));
          if (mfgChr && mfgChr->canRead()) {
            String val = mfgChr->readValue().c_str();
            val.replace("\"", "\\\"");
            json += ",\"manufacturer\":\"" + val + "\"";
          }
        }

        json += "}";
        client->disconnect();
        probed++;
      }
      delete client;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  json += "],\"total\":" + String(probed) + "}";

  if (wasScanning) startBleScanning();
  s_wifiPortalServer.send(200, "application/json", json);
}

static void registerBleAnalyzerHandlers() {
  s_wifiPortalServer.on("/bleanalyzer", HTTP_GET, handleBleAnalyzerPage);
  s_wifiPortalServer.on("/blejson", HTTP_GET, handleBleJson);
  s_wifiPortalServer.on("/bledl", HTTP_GET, handleBleDownload);
  s_wifiPortalServer.on("/bleclear", HTTP_GET, handleBleClearLogs);
  s_wifiPortalServer.on("/bletoggle", HTTP_GET, handleBleToggle);
  s_wifiPortalServer.on("/bleprobe", HTTP_GET, handleBleProbe);
  s_wifiPortalServer.on("/bleprobeall", HTTP_GET, handleBleProbeAll);
}
