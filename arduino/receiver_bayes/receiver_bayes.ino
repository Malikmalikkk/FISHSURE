#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>

// ================== CONFIG ==================
#define WIFI_CH 1
#define MAX_PAYLOAD 240
#define MAX_JPEG 40000
#define SWAP_BYTES true

#define SCREEN_W 480
#define SCREEN_H 320
#define UI_BAR_H 35

// Buttons
#define BTN_SELECT 0  // Boot button
#define BTN_CHANGE 35 // Example pin

// ================== GLOBALS ==================
TFT_eSPI tft = TFT_eSPI();

uint8_t senderMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Capture automatically

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

uint8_t currFishCount = 0; // State to hold last received fish count

struct ConfigPacket {
  uint8_t type; // PKT_CONFIG
  uint8_t mode; // 0=Auto, 1=Static
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t freq; // New field
};
#pragma pack(pop)

uint8_t *jpegBuf = nullptr;
uint8_t *chunkMap = nullptr;
uint32_t jpegSize = 0;
uint16_t numChunks = 0;
uint16_t frameId = 0;
uint16_t receivedChunks = 0;
bool active = false;
bool senderPeerAdded = false;

unsigned long lastDataMs = 0;

// UI State
int catchCount = 0;
enum Mode { MODE_AUTO = 0, MODE_STATIC = 1 };
Mode currentMode = MODE_AUTO;
// Static Color cycle: Red, Green, Blue, Yellow, Cyan, Magenta, White
uint8_t staticColorIdx = 6; // Default White
uint8_t colors[][3] = {
  {255, 0, 0},   {0, 255, 0},   {0, 0, 255},
  {255, 255, 0}, {0, 255, 255}, {255, 0, 255},
  {255, 255, 255}
};

// Static Freq cycle
uint16_t staticFreqIdx = 2; // Default 100Hz
uint16_t frequencies[] = {20, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1500};

enum ValidMenu { MENU_NONE, MENU_CATCH, MENU_MODE, MENU_COLOR, MENU_FREQ };
ValidMenu currentMenu = MENU_NONE;

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (y >= tft.height()) return false;
  tft.pushImage(x, y, w, h, bitmap);
  return true;
}

void resetFrame() {
  if (jpegBuf) { free(jpegBuf); jpegBuf = nullptr; }
  if (chunkMap) { free(chunkMap); chunkMap = nullptr; }
  jpegSize = 0; numChunks = 0; frameId = 0; receivedChunks = 0; active = false;
}

void drawUI() {
  // Premium dark bar with a subtle border
  tft.fillRect(0, 0, SCREEN_W, UI_BAR_H, 0x1084); // Dark grey/navy
  tft.drawFastHLine(0, UI_BAR_H - 1, SCREEN_W, TFT_DARKGREY);
  
  tft.setTextColor(TFT_WHITE, 0x1084);
  tft.setTextSize(2);
  
  // Catch Count (Left)
  tft.setCursor(10, 8);
  tft.printf("FISH: %d", catchCount);

  // Mode (Center-ish)
  tft.setCursor(130, 8);
  if (currentMode == MODE_AUTO) {
    tft.setTextColor(TFT_CYAN, 0x1084);
    tft.printf("AUTO-OPT");
  } else {
    tft.setTextColor(TFT_ORANGE, 0x1084);
    tft.printf("STATIC");
  }
  tft.setTextColor(TFT_WHITE, 0x1084);

  // Color & Freq (Right aligned)
  if (currentMode == MODE_STATIC) {
    tft.setTextSize(1);
    tft.setCursor(260, 6);
    tft.printf("COLOR: #%d", staticColorIdx);
    tft.setCursor(260, 18);
    tft.printf("FREQ: %dHz", frequencies[staticFreqIdx]);
    tft.setTextSize(2);
  } else {
    tft.setCursor(350, 8);
    tft.setTextColor(TFT_GREEN, 0x1084);
    tft.print("LINK OK");
    tft.setTextColor(TFT_WHITE, 0x1084);
  }
  
  // Highlight active menu with a glow effect
  if (currentMenu != MENU_NONE) {
    int x_underline = 0;
    int w_underline = 0;
    
    if (currentMenu == MENU_CATCH) { x_underline = 6; w_underline = 100; }
    else if (currentMenu == MENU_MODE) { x_underline = 126; w_underline = 110; }
    else if (currentMenu == MENU_COLOR) { x_underline = 256; w_underline = 80; }
    else if (currentMenu == MENU_FREQ) { x_underline = 256; w_underline = 80; }
    
    tft.drawRoundRect(x_underline, 2, w_underline, UI_BAR_H - 6, 4, TFT_YELLOW);
  }
}

void startFrame(const StartPacket &sp) {
  resetFrame();
  if (sp.jpegSize < 200 || sp.jpegSize > MAX_JPEG || sp.numChunks == 0) {
    Serial.println("[RX4] Invalid START");
    return;
  }
  jpegSize = sp.jpegSize;
  numChunks = sp.numChunks;
  frameId = sp.frameId;
  currFishCount = sp.fishCount; // Capture the fish in this photo

  jpegBuf = (uint8_t*)malloc(jpegSize);
  chunkMap = (uint8_t*)calloc(numChunks, 1);

  if (!jpegBuf || !chunkMap) {
    Serial.println("[RX4] Memory alloc failed");
    resetFrame();
    return;
  }
  active = true;
  lastDataMs = millis();
}

void decodeAndDisplay() {
  Serial.println("[RX4] ✅ Frame complete -> decoding");
  
  catchCount += currFishCount; // Add the actual fish count, not just 1

  // Clear background (optional since JPEG usually covers it)
  // tft.fillScreen(TFT_BLACK); 

  // Center the image if possible, but keep bar visible
  // We assume the image might be smaller than 480x(320-UI_BAR_H)
  TJpgDec.drawJpg(0, UI_BAR_H, jpegBuf, jpegSize);
  
  drawUI();
  resetFrame();
}

void addSenderPeer() {
  if (senderPeerAdded) return;
  
  // Try to register peer if MAC is valid (user must edit placeholder)
  if (senderMac[0] == 0xFF) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, senderMac, 6);
  peer.channel = WIFI_CH;
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) == ESP_OK) {
    senderPeerAdded = true;
    Serial.println("[RX4] Peer added");
  }
}

void sendConfig() {
  addSenderPeer();
  if (!senderPeerAdded) {
    Serial.println("[RX4] Cannot send config - Peer not added (Check MAC)");
    return;
  }

  ConfigPacket cp;
  cp.type = PKT_CONFIG;
  cp.mode = (uint8_t)currentMode;
  cp.r = 0; cp.g = 0; cp.b = 0;
  cp.freq = 0;
  
  if (currentMode == MODE_STATIC) {
    cp.r = colors[staticColorIdx][0];
    cp.g = colors[staticColorIdx][1];
    cp.b = colors[staticColorIdx][2];
    cp.freq = frequencies[staticFreqIdx];
  }

  esp_err_t res = esp_now_send(senderMac, (uint8_t*)&cp, sizeof(cp));
  if (res == ESP_OK) {
    Serial.println("[RX4] Config Sent");
  } else {
    Serial.printf("[RX4] Config Send Failed: %d\n", res);
  }
}

void onReceive(const esp_now_recv_info* info, const uint8_t *data, int len) {
  if (len < 1) return;
  
  // Opportunistically capture Sender MAC if still default FF
  if (senderMac[0] == 0xFF && info->src_addr) {
    memcpy(senderMac, info->src_addr, 6);
    Serial.printf("[RX4] Captured Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);
    addSenderPeer();
  }

  uint8_t type = data[0];

  if (type == PKT_START) {
    if (len != (int)sizeof(StartPacket)) return;
    StartPacket sp;
    memcpy(&sp, data, sizeof(sp));
    startFrame(sp);
    return;
  }

  if (type == PKT_DATA && active) {
    if (len < 7) return;

    uint16_t rxFrameId = *(uint16_t*)(data + 1);
    uint16_t rxChunk   = *(uint16_t*)(data + 3);
    uint16_t rxSize    = *(uint16_t*)(data + 5);

    if (rxFrameId != frameId) return;
    if (rxChunk >= numChunks) return;
    if (rxSize > MAX_PAYLOAD) return;
    if (len != (int)(7 + rxSize)) return;

    if (chunkMap[rxChunk]) return;

    uint32_t offset = (uint32_t)rxChunk * MAX_PAYLOAD;
    if (offset + rxSize > jpegSize) return;

    memcpy(jpegBuf + offset, data + 7, rxSize);
    chunkMap[rxChunk] = 1;
    receivedChunks++;
    lastDataMs = millis();

    if (receivedChunks == numChunks) {
      decodeAndDisplay();
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_CHANGE, INPUT_PULLUP);

  delay(800);

  // TFT init
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  drawUI();

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(SWAP_BYTES);
  TJpgDec.setCallback(tft_output);

  // Wi-Fi init
  nvs_flash_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[RX4] esp_now_init FAILED");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println("[RX4] Ready.");
}

void loop() {
  static unsigned long lastBtn = 0;
  if (millis() - lastBtn > 200) {
    if (digitalRead(BTN_SELECT) == LOW) {
      // Cycle Menu: None -> Catch -> Mode -> Color -> Freq -> None
      currentMenu = (ValidMenu)((int)currentMenu + 1);
      if (currentMenu > MENU_FREQ) currentMenu = MENU_NONE;
      
      // If Auto mode, skip Color/Freq menus?
      if ((currentMenu == MENU_COLOR || currentMenu == MENU_FREQ) && currentMode == MODE_AUTO) {
        currentMenu = MENU_NONE; // Wrap back
      }

      drawUI();
      lastBtn = millis();
    }
    
    if (digitalRead(BTN_CHANGE) == LOW) {
        bool changed = false;
        if (currentMenu == MENU_CATCH) {
            catchCount = 0; // Reset
            changed = true;
        } else if (currentMenu == MENU_MODE) {
            currentMode = (currentMode == MODE_AUTO) ? MODE_STATIC : MODE_AUTO;
            changed = true;
        } else if (currentMenu == MENU_COLOR) {
            staticColorIdx++;
            if (staticColorIdx >= 7) staticColorIdx = 0;
            changed = true;
        } else if (currentMenu == MENU_FREQ) {
            staticFreqIdx++;
            if (staticFreqIdx >= 5) staticFreqIdx = 0;
            changed = true;
        }

        if (changed) {
            drawUI();
            if (currentMenu == MENU_MODE || currentMenu == MENU_COLOR || currentMenu == MENU_FREQ) {
                sendConfig();
            }
        }
        lastBtn = millis();
    }
  }

  // Timeout logic
  if (active && (millis() - lastDataMs > 6000)) {
    Serial.printf("[RX4] ❌ Timeout. Got %u/%u\n", receivedChunks, numChunks);
    resetFrame();
  }
}
