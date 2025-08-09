# RC-Boat
 ESP32 RC Boat Controller with Telemetry (ESP-NOW)
This project uses two ESP32 boards to create a long-range, low-latency, bi-directional RC control system for a boat.
The handheld transmitter controls the boat’s motors via a joystick and displays telemetry (battery voltage + signal strength) on an OLED.
The boat receiver drives the motors and sends back telemetry using ESP-NOW.

📦 Features
Handheld Transmitter (TX)
Reads 2-axis joystick values for forward/backward (Y-axis) and left/right (X-axis) control.

Displays:

TX battery voltage (from onboard battery)

Boat battery voltage (received via ESP-NOW)

Signal strength (RSSI) in dBm with quality label (Excellent, Strong, Moderate, Weak, Very Weak)

Current joystick commands sent to the boat

Automatically centers joystick on startup for calibration.

Sends control packets continuously for smooth motor response.

Boat Receiver (RX)
Controls two DC motors using PWM + direction pins:

Motor A (Y-axis): PWM on GPIO19, DIR on GPIO18

Motor B (X-axis): PWM on GPIO5, DIR on GPIO4

Measures boat battery voltage using a 20k:10k voltage divider on VP (GPIO36).

Sends telemetry back to TX:

scss
Copy
Edit
RSSI (dBm), Boat battery voltage (mV)
Failsafe: Stops motors if no control packet is received for 300 ms.

🔧 Hardware Requirements
Transmitter
ESP32 Dev Board

2-axis analog joystick (X on GPIO34, Y on GPIO32)

Voltage divider for TX battery monitoring on GPIO36 (20k:10k)

128x64 SSD1306 OLED display (I²C at 0x3C)

Battery power source

Boat Receiver
ESP32 Dev Board

Dual motor driver module (PWM + DIR inputs)

Two brushed DC motors

Voltage divider for boat battery monitoring on GPIO36 (20k:10k)

Battery power source

📡 Communication Flow
TX → RX

CSV: "cmdX,cmdY"

cmdX and cmdY range from -255 to +255 for proportional motor control.

RX → TX

CSV: "rssi,boatBatt_mV"

rssi is signal strength in dBm (negative value).

boatBatt_mV is battery voltage in millivolts.

🔋 Battery Voltage Calculation
Both TX and RX use:

cpp
Copy
Edit
ADC_REF_V = 3.3V
Voltage divider = 20k:10k (gain = 3.0)
Battery voltage (V) = (ADC reading / 4095) × ADC_REF_V × gain
📊 RSSI Strength Labels
RSSI (dBm)	Quality
≥ -50	Excellent
-60 to -51	Strong
-70 to -61	Moderate
-80 to -71	Weak
< -80	Very Weak

🚀 Setup Instructions
Find MAC Addresses
Upload a simple sketch to each ESP32 to print:

cpp
Copy
Edit
Serial.println(WiFi.macAddress());
Use the STA MAC for pairing in code.

Update MAC Addresses in Code

In TX code, set:

cpp
Copy
Edit
const uint8_t RECEIVER_MAC[6] = { ... }; // Boat MAC
In RX code, set:

cpp
Copy
Edit
const uint8_t TRANSMITTER_MAC[6] = { ... }; // Handheld MAC
Wire hardware as per the pin assignments above.

Flash TX and RX code to respective ESP32 boards.

Power both devices — joystick movement controls the boat, OLED displays telemetry.

📁 File Structure
bash
Copy
Edit
/ESP32-RC-Boat
│
├── transmitter/
│   └── transmitter.ino   # Handheld joystick + OLED display code
│
├── boat/
│   └── boat.ino           # Motor control + telemetry return code
│
└── README.md              # This documentation

