//======================================
//Transmitter MAC Address: CC:DB:A7:98:13:D0
//======================================
//======================================
//Transmitter MAC Address will print at startup
//======================================

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== OLED =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== Pin Mapping (Updated) =====
#define SENSOR_VP   36   // Throttle ADC input
#define SENSOR_VN   39   // Direction ADC input
#define VBAT_PIN    34   // Battery voltage sense via 10k:20k divider
#define LEFT_BTN    25   // Left button (unused for now)
#define RIGHT_BTN   26   // Right button (unused for now)

// ===== WiFi / ESP-NOW =====
// Boat STA MAC (your boat)
const uint8_t RECEIVER_MAC[6] = {0x34, 0xAB, 0x95, 0x42, 0x47, 0x74};
const uint8_t WIFI_CHANNEL = 1;

// ===== Battery divider constants =====
const float ADC_REF_V = 3.3f;
const float DIV_GAIN  = 3.0f; // 10k:20k divider ratio

// ===== Telemetry from boat =====
volatile int lastBoatRSSI = 127;        // 127 = unknown
volatile uint16_t lastBoatBatt_mV = 0;

// ===== Helper: Read battery voltage =====
uint16_t readBattery_mV() {
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
  int raw = analogRead(VBAT_PIN);
  float vpin = (raw / 4095.0f) * ADC_REF_V;
  float vbat = vpin * DIV_GAIN;
  return (uint16_t)lroundf(vbat * 1000.0f);
}

// ===== Convert RSSI (dBm) to a label =====
static const char* rssiLabel(int rssi) {
  if (rssi >= -50)      return "Excellent";
  else if (rssi >= -60) return "Strong";
  else if (rssi >= -70) return "Moderate";
  else if (rssi >= -80) return "Weak";
  else                  return "Very Weak";
}

// ===== RX callback: boat sends CSV "rssi,boatBatt_mV" =====
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  static char buf[32];
  if (len <= 0) return;
  if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, data, len);
  buf[len] = '\0';

  Serial.printf("Boat->TX CSV: '%s' from %02X:%02X:%02X:%02X:%02X:%02X\n",
                buf, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  int rssi = 127;
  unsigned battmV = 0;
  if (sscanf(buf, "%d,%u", &rssi, &battmV) == 2) {
    lastBoatRSSI = rssi;
    lastBoatBatt_mV = (uint16_t)battmV;
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(500);

  // Print MAC address at startup
  String mac = WiFi.macAddress();
  Serial.println("======================================");
  Serial.print("ESP32 MAC Address: ");
  Serial.println(mac);
  Serial.println("======================================");

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    while (1) {}
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();

  // WiFi / ESP-NOW setup
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1) {}
  }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  pinMode(LEFT_BTN, INPUT_PULLUP);
  pinMode(RIGHT_BTN, INPUT_PULLUP);
}

// ===== LOOP =====
void loop() {
  // === Read throttle (SENSOR_VP) and direction (SENSOR_VN) ===
  uint16_t throttleRaw = analogRead(SENSOR_VP);
  uint16_t directionRaw = analogRead(SENSOR_VN);

  // === Map throttle 0–4095 → 0–255 ===
int throttle = map(throttleRaw, 0, 4095, 255, 0);

  // === Normalize direction to -1.0 .. +1.0 ===
  float centerVN = 2048.0f;
  float turn = (directionRaw - centerVN) / centerVN;
  if (turn > 1.0f) turn = 1.0f;
  if (turn < -1.0f) turn = -1.0f;

  // === Compute proportional left/right outputs ===
  float leftMotor  = throttle * (1.0f - turn);
  float rightMotor = throttle * (1.0f + turn);

  // Clamp values
  leftMotor  = constrain(leftMotor,  0, 255);
  rightMotor = constrain(rightMotor, 0, 255);

  int cmdL = (int)leftMotor;
  int cmdR = (int)rightMotor;

  // === Send CSV "cmdL,cmdR" ===
  char csv[24];
  snprintf(csv, sizeof(csv), "%d,%d", cmdL, cmdR);
  esp_now_send(RECEIVER_MAC, (uint8_t*)csv, strlen(csv));

  // === Read TX battery voltage ===
  uint16_t txBatt_mV = readBattery_mV();

  // === Update OLED display ===
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.printf("TX:   %.2f V\n", txBatt_mV / 1000.0f);

  display.setCursor(0, 12);
  if (lastBoatBatt_mV > 0)
    display.printf("Boat: %.2f V\n", lastBoatBatt_mV / 1000.0f);
  else
    display.printf("Boat: --.-- V\n");

  display.setCursor(0, 24);
  if (lastBoatRSSI != 127) {
    display.printf("RSSI: %d dBm\n", lastBoatRSSI);
    display.setCursor(0, 36);
    display.printf("Link: %s\n", rssiLabel(lastBoatRSSI));
  } else {
    display.printf("RSSI: -- dBm\n");
    display.setCursor(0, 36);
    display.printf("Link: --\n");
  }

  display.setCursor(0, 50);
  display.printf("L:%4d  R:%4d", cmdL, cmdR);
  display.display();

  delay(50); // small delay for stability
}
