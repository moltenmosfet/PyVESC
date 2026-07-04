// Transparent TCP <-> UART bridge for driving a VESC over WiFi.
//
// LAPTOP -> WiFi -> router -> WiFi -> ESP32 -> UART -> VESC
//
// The bridge is stateless: bytes in one side come out the other, so the
// pyvesc driver talks to tcp://<esp32-ip>:65102 exactly as it would to a
// serial port. Power the ESP32 from the VESC's 5V supply — never from
// anything laptop-referenced, or the galvanic isolation this exists for
// is gone.

#include <WiFi.h>

// ---- configure these ----
const char *WIFI_SSID = "your-ssid";
const char *WIFI_PASS = "your-password";
const uint16_t TCP_PORT = 65102;    // VESC Tool / pyvesc default
const uint32_t UART_BAUD = 115200;  // must match the VESC app config
const int UART_RX_PIN = 16;         // ESP32 RX2  <- VESC TX
const int UART_TX_PIN = 17;         // ESP32 TX2  -> VESC RX
const int LED_PIN = 2;              // lit while a client is connected

WiFiServer server(TCP_PORT);
WiFiClient client;
uint8_t buf[1024];

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // modem power-save adds 100 ms+ latency spikes
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.printf("\nIP: %s  port: %u\n", WiFi.localIP().toString().c_str(), TCP_PORT);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);           // USB debug console only
  Serial2.setRxBufferSize(4096);  // GetValues bursts + firmware chunks
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  connectWiFi();
  server.begin();
  server.setNoDelay(true);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    client.stop();
    connectWiFi();
  }

  // a new connection replaces the old one, so a half-dead session
  // can never lock out the laptop
  if (server.hasClient()) {
    if (client) client.stop();
    client = server.accept();
    client.setNoDelay(true);
    Serial.printf("Client connected: %s\n", client.remoteIP().toString().c_str());
  }

  digitalWrite(LED_PIN, (client && client.connected()) ? HIGH : LOW);

  // TCP -> UART
  while (client && client.connected() && client.available()) {
    int n = client.read(buf, sizeof(buf));
    if (n > 0) Serial2.write(buf, n);
  }

  // UART -> TCP, batched into one TCP segment per pass
  int avail = Serial2.available();
  if (avail > 0) {
    int n = Serial2.read(buf, min(avail, (int)sizeof(buf)));
    if (n > 0) {
      if (client && client.connected()) {
        client.write(buf, n);
      }
      // no client: bytes are drained and dropped so the UART
      // buffer never fills with stale telemetry
    }
  }
}
