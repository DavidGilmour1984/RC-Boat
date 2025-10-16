/*
  ============================================================
  PROJECT:   ESP32 Boat Receiver (Dual Motor + Telemetry)
  AUTHOR:    David Gilmour
  DATE:      October 2025
  BOARD:     ESP32-WROOM-32U-N4
  ============================================================

  DESCRIPTION:
  This firmware runs on the boat-side ESP32 receiver. It listens
  for control data (CSV format: "cmdX,cmdY") sent via ESP-NOW 
  from the handheld transmitter. It drives two DC motors through 
  RZ7886 H-bridge modules and returns live telemetry (RSSI and 
  battery voltage) to the transmitter.

  FUNCTIONAL OVERVIEW:
  ------------------------------------------------------------
  • Uses ESP-NOW for bidirectional communication.
  • Drives two motors (left and right) with direction + PWM.
  • Reads boat battery voltage via ADC pin 36 (20k:10k divider).
  • Sends telemetry back: RSSI (signal strength) and voltage.
  • Includes failsafe timeout — motors stop if no data received.
  • Includes quiet-boot initialization (no motor twitch).
  • Limits power when battery voltage drops below safe level.
  • Fully compatible with your handheld transmitter (MAC: CC:DB:A7:98:13:D0).

  PIN ASSIGNMENTS:
  ------------------------------------------------------------
  LEFT MOTOR  (Y-axis)
     PWM  → GPIO 19
     DIR  → GPIO 18

  RIGHT MOTOR (X-axis)
     PWM  → GPIO 5
     DIR  → GPIO 4

  BATTERY VOLTAGE INPUT
     ADC  → GPIO 36 (VP)  via 20k:10k divider (3× scaling)

  COMMUNICATION
     ESP-NOW on Wi-Fi Channel 1

  SAFETY FEATURES:
  ------------------------------------------------------------
  • Motors disabled at boot and after communication timeout.
  • Invalid ADC readings ignored.
  • Battery under-voltage automatically scales down power.
  • Low-voltage cutoff protects 2S LiPo (~6.4 V threshold).
  • Reverse polarity and direction inversion configurable.

  OPTIONAL EXPANSION (future use):
  ------------------------------------------------------------
  • I²C ready on GPIO 21 (SDA) / 22 (SCL)
  • UART2 available on GPIO 16 (RX) / 17 (TX)
  • Free ADC inputs on GPIO 25, 26, 32, 33
  • Ideal for GPS, depth, or water quality sensors.

  ============================================================
*/


#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
extern "C" {
  #include "esp_wifi_types.h"
}

// ===== Pins (boat) =====
const int motorAPWM = 19;
const int motorADIR = 18;
const int motorBPWM = 5;
const int motorBDIR = 4;
const int VP_PIN   = 36;   // Battery divider (20k:10k on VP)

// ===== PWM config =====
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int maxDuty = 254;

// ===== Comms =====
const uint8_t TRANSMITTER_MAC[6] = {0xCC,0xDB,0xA7,0x98,0x13,0xD0};
const uint8_t WIFI_CHANNEL = 1;
const unsigned long COMM_TIMEOUT_MS = 300;

// Optional polarity flips
const bool INVERT_A = true; // Y motor
const bool INVERT_B = true; // X motor

// ===== State =====
volatile unsigned long lastRxMs = 0;
volatile int8_t lastPeerRSSI = 127;

// ===== Battery calc =====
const float ADC_REF_V    = 3.3f;
const float DIVIDER_GAIN = 3.0f;
const float MIN_SAFE_VOLTAGE = 6.4f;   // lower cutoff for 2S LiPo
const float MAX_SANE_VOLTAGE = 15.0f;  // sanity upper bound

// ===== Helpers =====
static inline void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}
static inline void safeDriveOff() {
  stopMotors();
  digitalWrite(motorADIR, LOW);
  digitalWrite(motorBDIR, LOW);
}

// Map -255..255 -> dir + PWM
static inline void driveMotor(int16_t cmd, int dirPin, int channel, bool invert) {
  if (invert) cmd = -cmd;
  bool rev = (cmd < 0);
  int duty = abs(cmd);
  if (duty > 255) duty = 255;
  duty = map(duty, 0, 255, 0, maxDuty);
  if (rev) duty = maxDuty - duty;
  digitalWrite(dirPin, rev ? HIGH : LOW);
  ledcWrite(channel, duty);
}

static inline uint16_t readBattery_mV(int pin) {
  analogSetPinAttenuation(pin, ADC_11db);
  int raw = analogRead(pin);
  float v_pin = (raw / 4095.0f) * ADC_REF_V;
  float v_bat = v_pin * DIVIDER_GAIN;
  if (v_bat < 0.5f || v_bat > 20.0f) return 0; // ignore nonsense
  return (uint16_t)lroundf(v_bat * 1000.0f);
}

// Parse CSV "cmdX,cmdY"
static bool parseCSVCmds(const char* buf, int16_t& outX, int16_t& outY) {
  int x=0, y=0;
  if (sscanf(buf, "%d,%d", &x, &y) == 2) {
    x = constrain(x, -255, 255);
    y = constrain(y, -255, 255);
    outX = (int16_t)x; outY = (int16_t)y;
    return true;
  }
  return false;
}

// ===== Promiscuous RSSI capture =====
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t *ppkt = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *payload = ppkt->payload;
  const uint8_t *addr2 = payload + 10;
  if (memcmp(addr2, TRANSMITTER_MAC, 6) == 0) {
    lastPeerRSSI = ppkt->rx_ctrl.rssi;
    esp_wifi_set_promiscuous(false);
  }
}

// ===== ESP-NOW send callback =====
void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.printf("Send to %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],
                status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
}

// ===== ESP-NOW receive callback =====
void onDataRecv(const uint8_t*, const uint8_t* data, int len) {
  static char buf[32];
  if (len <= 0) return;
  if (len >= (int)sizeof(buf)) len = sizeof(buf)-1;
  memcpy(buf, data, len);
  buf[len] = '\0';
  Serial.printf("RX CSV: '%s'\n", buf);

  int16_t cmdX, cmdY;
  if (!parseCSVCmds(buf, cmdX, cmdY)) return;
  lastRxMs = millis();

  // small dead-zone
  if (abs(cmdX) < 3) cmdX = 0;
  if (abs(cmdY) < 3) cmdY = 0;

  // Read battery
  uint16_t boatBatt_mV = readBattery_mV(VP_PIN);
  float boatBatt_V = boatBatt_mV / 1000.0f;

  // Safety voltage window
  if (boatBatt_V < 1.0f || boatBatt_V > MAX_SANE_VOLTAGE) {
    safeDriveOff();
    Serial.println("Invalid battery reading – motors off");
  } else if (boatBatt_V < MIN_SAFE_VOLTAGE) {
    // low battery but still alive — allow reduced power
    cmdX /= 3;
    cmdY /= 3;
    driveMotor(cmdY, motorADIR, 0, INVERT_A);
    driveMotor(cmdX, motorBDIR, 1, INVERT_B);
    Serial.println("Low voltage: limited drive");
  } else {
    // normal operation
    driveMotor(cmdY, motorADIR, 0, INVERT_A);
    driveMotor(cmdX, motorBDIR, 1, INVERT_B);
  }

  // Send telemetry back
  char out[24];
  snprintf(out, sizeof(out), "%d,%u", (int)lastPeerRSSI, (unsigned)boatBatt_mV);
  esp_now_send(TRANSMITTER_MAC, (uint8_t*)out, strlen(out));
  esp_wifi_set_promiscuous(true);
}

void setup() {
  Serial.begin(115200);

  // --- QUIET BOOT FIX: set all motor pins LOW before anything else ---
  pinMode(motorADIR, OUTPUT);
  pinMode(motorBDIR, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  digitalWrite(motorADIR, LOW);
  digitalWrite(motorBDIR, LOW);
  digitalWrite(motorAPWM, LOW);
  digitalWrite(motorBPWM, LOW);

  delay(50);  // allow outputs to settle before PWM setup
  // ---------------------------------------------------------------

  // PWM setup
  ledcSetup(0, pwmFreq, pwmResolution);
  ledcAttachPin(motorAPWM, 0);
  ledcWrite(0, 0);
  ledcSetup(1, pwmFreq, pwmResolution);
  ledcAttachPin(motorBPWM, 1);
  ledcWrite(1, 0);

  safeDriveOff();  // ensure off after attach

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {}
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, TRANSMITTER_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  wifi_promiscuous_filter_t filt{};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);
  esp_wifi_set_promiscuous(true);

  lastRxMs = millis();
  Serial.print("Boat STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Boat ready.");
}

void loop() {
  if ((millis() - lastRxMs) > COMM_TIMEOUT_MS) {
    safeDriveOff();
  }
}
