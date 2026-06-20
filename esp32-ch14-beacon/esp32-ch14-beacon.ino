#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino_GFX_Library.h>

static Arduino_DataBus* bus =
  new Arduino_ESP32QSPI(AMOLED_CS, AMOLED_SCK, AMOLED_D0, AMOLED_D1, AMOLED_D2, AMOLED_D3);
static Arduino_CO5300* gfx =
  new Arduino_CO5300(bus, AMOLED_RESET, 0, AMOLED_WIDTH, AMOLED_HEIGHT, 22, 0, 0, 0);

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AMOLED_PWR_EN, OUTPUT);
  digitalWrite(AMOLED_PWR_EN, HIGH);
  delay(50);

  gfx->begin();
  gfx->displayOn();
  gfx->setBrightness(255);
  gfx->fillScreen(0x0000);

  // Start AP on channel 14
  WiFi.mode(WIFI_AP);
  wifi_country_t wc = { .cc = "JP", .schan = 1, .nchan = 14,
                         .max_tx_power = 80, .policy = WIFI_COUNTRY_POLICY_MANUAL };
  esp_wifi_set_country(&wc);
  WiFi.softAP("CH13_TEST_BEACON", "12345678", 13, 0, 1);

  // Draw devil face
  int cx = gfx->width() / 2;
  int cy = gfx->height() / 2 - 30;
  int r = 60;

  gfx->fillCircle(cx, cy, r, 0xF800);
  gfx->fillTriangle(cx - 50, cy - 45, cx - 35, cy - 75, cx - 20, cy - 45, 0xF800);
  gfx->fillTriangle(cx + 50, cy - 45, cx + 35, cy - 75, cx + 20, cy - 45, 0xF800);
  gfx->fillRect(cx - 30, cy - 15, 20, 8, 0x0000);
  gfx->fillRect(cx + 10, cy - 15, 20, 8, 0x0000);
  gfx->fillRect(cx - 35, cy + 15, 70, 4, 0x0000);
  gfx->fillTriangle(cx - 20, cy + 15, cx - 15, cy + 28, cx - 10, cy + 15, 0xFFFF);
  gfx->fillTriangle(cx + 10, cy + 15, cx + 15, cy + 28, cx + 20, cy + 15, 0xFFFF);

  // Read back actual channel after AP settles
  delay(1000);
  uint8_t actualCh = 0;
  wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&actualCh, &sec);

  int textY = cy + r + 20;
  gfx->setTextColor(0xF800);
  gfx->setTextSize(2);
  gfx->setCursor(10, textY);
  gfx->print("CH14 BEACON");
  gfx->setCursor(10, textY + 25);
  gfx->print("SSID: CH14_TEST_BEACON");
  gfx->setCursor(10, textY + 50);
  gfx->setTextColor(actualCh == 14 ? 0x07E0 : 0xF800);
  gfx->printf("Actual channel: %d", actualCh);
  gfx->setCursor(10, textY + 75);
  gfx->setTextColor(0x07E0);
  gfx->printf("IP: %s", WiFi.softAPIP().toString().c_str());

  Serial.printf("AP started: CH14_TEST_BEACON actual_ch=%d\n", actualCh);
}

void loop() {
  delay(5000);
}
