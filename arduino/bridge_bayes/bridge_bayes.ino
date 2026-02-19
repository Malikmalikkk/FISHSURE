#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>

// ================== CONFIG ==================
#define WIFI_CH 1
// REPLACE WITH YOUR RECEIVER MAC ADDRESS
uint8_t receiverMac[] = {0xCC, 0xDB, 0xA7, 0x92, 0x41, 0x14}; 

#define BAUD_RATE 921600
#define MAX_PAYLOAD 240
#define MAX_JPEG 40000

// ================== PACKETS ==================
enum PktType { PKT_START = 1, PKT_DATA = 2, PKT_CONFIG = 3 };

#pragma pack(push, 1)
struct StartPacket {
  uint8_t type; // PKT_START
  uint16_t frameId;
  uint32_t jpegSize;
  uint16_t numChunks;
  uint8_t fishCount; // New field
};

struct ConfigPacket {
  uint8_t type; // PKT_CONFIG
  uint8_t mode; // 0=Auto, 1=Static
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t freq; // New field
};
#pragma pack(pop)

volatile bool sendCbFired = true;
volatile bool lastSendOk = true;

void onSend(const wifi_tx_info_t*, esp_now_send_status_t status) {
  sendCbFired = true;
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

// Receive callback (Config from Receiver)
void onReceive(const esp_now_recv_info*, const uint8_t *data, int len) {
  if (len != sizeof(ConfigPacket)) return;
  if (data[0] != PKT_CONFIG) return;

  ConfigPacket cp;
  memcpy(&cp, data, sizeof(cp));

  // Forward to Pi via Serial
  if (cp.mode == 0) {
    Serial.println("CMD:MODE:AUTO");
  } else {
    Serial.println("CMD:MODE:STATIC");
    Serial.printf("CMD:COLOR:%d,%d,%d\n", cp.r, cp.g, cp.b);
    Serial.printf("CMD:FREQ:%d\n", cp.freq);
  }
}

bool sendNow(const uint8_t *buf, size_t len) {
  sendCbFired = false;
  lastSendOk = true;

  esp_err_t e = esp_now_send(receiverMac, buf, len);
  if (e != ESP_OK) {
    Serial.printf("[TX4] esp_now_send ERR=%d len=%u\n", (int)e, (unsigned)len);
    sendCbFired = true;
    return false;
  }

  unsigned long t0 = millis();
  while (!sendCbFired) {
    if (millis() - t0 > 500) { // prevent deadlock
      Serial.println("[TX4] send_cb TIMEOUT");
      sendCbFired = true;
      return false;
    }
    delay(1);
    yield();
  }

  if (!lastSendOk) {
    Serial.println("[TX4] send_cb FAIL");
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(8000);

  // Wi-Fi init (manual, stable)
  nvs_flash_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[TX4] esp_now_init FAILED");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive); // Register receive cb

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = WIFI_CH;
  peer.encrypt = false;

  esp_err_t pe = esp_now_add_peer(&peer);
  if (pe != ESP_OK) {
    Serial.printf("[TX4] add_peer FAILED err=%d\n", (int)pe);
    while (true) delay(1000);
  }

  Serial.println("[TX4] Sender ready. Waiting for Pi JPEG (AA 66)...");
}

void loop() {
  // Wait for Pi header: 0xAA 0x66 + uint32 size + uint8 fishCount (7 bytes)
  if (Serial.available() < 7) return;

  uint8_t h1 = Serial.read();
  uint8_t h2 = Serial.read();
  if (h1 != 0xAA || h2 != 0x66) return;

  uint32_t jpegSize = 0;
  if (Serial.readBytes((uint8_t*)&jpegSize, 4) != 4) return;

  uint8_t fishCount = Serial.read();

  if (jpegSize < 200 || jpegSize > MAX_JPEG) {
    Serial.printf("[TX4] invalid jpegSize=%lu\n", (unsigned long)jpegSize);
    return;
  }

  uint8_t* jpegBuf = (uint8_t*)malloc(jpegSize);
  if (!jpegBuf) {
    Serial.println("[TX4] malloc fail");
    return;
  }

  size_t readN = Serial.readBytes(jpegBuf, jpegSize);
  if (readN != jpegSize) {
    Serial.printf("[TX4] serial underflow %lu/%lu\n",
                  (unsigned long)readN, (unsigned long)jpegSize);
    free(jpegBuf);
    return;
  }

  static uint16_t frameId = 0;
  frameId++;

  uint16_t numChunks = (jpegSize + MAX_PAYLOAD - 1) / MAX_PAYLOAD;
  Serial.printf("[TX4] FRAME %u jpeg=%lu chunks=%u\n",
                frameId, (unsigned long)jpegSize, numChunks);

  // START
  StartPacket sp{PKT_START, frameId, jpegSize, numChunks, fishCount};
  if (!sendNow((uint8_t*)&sp, sizeof(sp))) {
    Serial.println("[TX4] START send failed");
    free(jpegBuf);
    delay(200);
    return;
  }

  // DATA packet buffer: header(7) + payload
  uint8_t pkt[7 + MAX_PAYLOAD];
  pkt[0] = PKT_DATA;

  uint32_t offset = 0;
  for (uint16_t i = 0; i < numChunks; i++) {
    uint16_t chunkSize = (jpegSize - offset > MAX_PAYLOAD) ? MAX_PAYLOAD : (uint16_t)(jpegSize - offset);

    *(uint16_t*)(pkt + 1) = frameId;
    *(uint16_t*)(pkt + 3) = i;
    *(uint16_t*)(pkt + 5) = chunkSize;
    memcpy(pkt + 7, jpegBuf + offset, chunkSize);

    if (!sendNow(pkt, 7 + chunkSize)) {
      Serial.printf("[TX4] DATA send failed at chunk %u\n", i);
      break;
    }

    offset += chunkSize;

    // ✅ pacing (prevents the "START ok, 0 chunks later" failure)
    delay(10);
    yield();

    if (i % 10 == 0 || i == numChunks - 1) {
      Serial.printf("[TX4] sent %u/%u\n", i + 1, numChunks);
    }
  }

  Serial.println("[TX4] Done");

  free(jpegBuf);

  // ✅ recovery delay between frames (clears internal buffers)
  delay(1000);
  yield();
}
