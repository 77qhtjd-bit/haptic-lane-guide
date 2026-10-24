// Lane-keeping aid actuator for Arduino UNO R4 WiFi.
//
// Receives "<seq>,<left>,<right>\n" (left/right 0..255) on UDP 4210, replies "ACK,<seq>\n"
// to the sender, drives two PWM pins (vibration motors) and shows the command on the
// built-in 12x8 LED matrix. If no valid command arrives for 500 ms, outputs go to 0 and the
// matrix shows the "no signal" screen (last octet of the IP + dotted bottom row).
// While there is no signal, "HELLO,<ip>\n" is broadcast to UDP 4211 every 2 s so the
// laptop can find the board.

#include "WiFiS3.h"
#include "Arduino_LED_Matrix.h"
#include "arduino_secrets.h"

// ---------- Settings ----------
const int LEFT_MOTOR_PIN = 9;     // PWM pin
const int RIGHT_MOTOR_PIN = 10;   // PWM pin
const unsigned int CMD_PORT = 4210;
const unsigned int HELLO_PORT = 4211;
const unsigned long SAFETY_TIMEOUT_MS = 500;
const unsigned long HELLO_INTERVAL_MS = 2000;
const unsigned long WIFI_RETRY_MS = 10000;
const unsigned long WIFI_CHECK_MS = 1000;   // WiFi.status() talks to the module, so poll it slowly

WiFiUDP udp;
ArduinoLEDMatrix matrix;
uint8_t frame[8][12];

unsigned long lastValidMs = 0;
unsigned long lastHelloMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastWifiCheckMs = 0;
bool wifiModule = true;
bool haveSignal = false;
bool sendHello = true;
bool udpStarted = false;
int curLeft = -1, curRight = -1;   // last values drawn (-1 = no-signal screen shown)
IPAddress myIp(0, 0, 0, 0);

// 3x5 digit font, one byte per row, low 3 bits = pixels (bit 2 = left column).
const uint8_t DIGITS[10][5] = {
  {7, 5, 5, 5, 7}, {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7}, {7, 1, 7, 1, 7}, {5, 5, 7, 1, 1},
  {7, 4, 7, 1, 7}, {7, 4, 7, 5, 7}, {7, 1, 1, 1, 1}, {7, 5, 7, 5, 7}, {7, 5, 7, 1, 7},
};
// Left-pointing arrow, 5x5, bit 4 = left column.
const uint8_t ARROW_LEFT[5] = {0b00100, 0b01000, 0b11111, 0b01000, 0b00100};

// ---------- Outputs ----------
void setMotors(int left, int right) {
  analogWrite(LEFT_MOTOR_PIN, left);
  analogWrite(RIGHT_MOTOR_PIN, right);
}

void clearFrame() { memset(frame, 0, sizeof(frame)); }

void drawDigit(int d, int col) {
  for (int r = 0; r < 5; r++)
    for (int c = 0; c < 3; c++)
      if (DIGITS[d][r] & (4 >> c)) frame[r][col + c] = 1;
}

// "No signal": last IP octet on rows 0-4, dotted line on row 7.
void showNoSignal() {
  clearFrame();
  int octet = myIp[3];
  int n = octet >= 100 ? 3 : (octet >= 10 ? 2 : 1);
  int col = (12 - (n * 4 - 1)) / 2;
  int div = n == 3 ? 100 : (n == 2 ? 10 : 1);
  for (; div > 0; div /= 10) {
    drawDigit((octet / div) % 10, col);
    col += 4;
  }
  for (int c = 0; c < 12; c += 2) frame[7][c] = 1;
  matrix.renderBitmap(frame, 8, 12);
  curLeft = curRight = -1;
}

// Bar length (0..6 columns) for intensity 0..255.
int barLen(int v) { return (v * 6 + 254) / 255; }

// Left arrow (cols 0-4) / right arrow (cols 7-11) when active; intensity bars on rows 6-7
// growing outward from the center. Both zero: small square in the center.
void showCommand(int left, int right) {
  clearFrame();
  for (int r = 0; r < 5; r++)
    for (int c = 0; c < 5; c++) {
      bool px = ARROW_LEFT[r] & (0b10000 >> c);
      if (left > 0 && px) frame[r][c] = 1;
      if (right > 0 && px) frame[r][11 - c] = 1;
    }
  for (int i = 0; i < barLen(left); i++) frame[6][5 - i] = frame[7][5 - i] = 1;
  for (int i = 0; i < barLen(right); i++) frame[6][6 + i] = frame[7][6 + i] = 1;
  if (left == 0 && right == 0) frame[2][5] = frame[2][6] = frame[3][5] = frame[3][6] = 1;
  matrix.renderBitmap(frame, 8, 12);
  curLeft = left;
  curRight = right;
}

// ---------- Parsing ----------
// Parses "<seq>,<left>,<right>" (optional trailing \r\n). Returns false on any malformed input.
bool parseCommand(const char *s, unsigned long &seq, int &left, int &right) {
  unsigned long vals[3];
  for (int i = 0; i < 3; i++) {
    if (*s < '0' || *s > '9') return false;
    unsigned long v = 0;
    int digits = 0;
    while (*s >= '0' && *s <= '9') {
      if (++digits > 9) return false;
      v = v * 10 + (*s++ - '0');
    }
    vals[i] = v;
    if (i < 2) {
      if (*s++ != ',') return false;
    }
  }
  while (*s == '\r' || *s == '\n') s++;
  if (*s != '\0') return false;
  if (vals[1] > 255 || vals[2] > 255) return false;
  seq = vals[0];
  left = vals[1];
  right = vals[2];
  return true;
}

// ---------- Wi-Fi ----------
bool ipValid(IPAddress ip) { return ip[0] | ip[1] | ip[2] | ip[3]; }

// Compares "a.b.c" version strings numerically. Returns <0, 0, >0.
int compareVersion(const char *a, const char *b) {
  for (int part = 0; part < 3; part++) {
    char *ea, *eb;
    long x = strtol(a, &ea, 10), y = strtol(b, &eb, 10);
    if (x != y) return x < y ? -1 : 1;
    if (*ea != '.' || *eb != '.') break;
    a = ea + 1;
    b = eb + 1;
  }
  return 0;
}

void connectWifi() {
  setMotors(0, 0);   // WiFi.begin() blocks for a few seconds
  Serial.print("Connecting to ");
  Serial.println(SECRET_SSID);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  lastWifiAttemptMs = millis();
}

void onWifiUp() {
  myIp = WiFi.localIP();
  Serial.print("Connected. IP: ");
  Serial.println(myIp);
  udp.begin(CMD_PORT);
  udpStarted = true;
  sendHello = true;
  lastHelloMs = 0;
  showNoSignal();
}

void broadcastHello() {
  IPAddress mask = WiFi.subnetMask();
  IPAddress bcast(myIp[0] | (~mask[0] & 0xFF), myIp[1] | (~mask[1] & 0xFF),
                  myIp[2] | (~mask[2] & 0xFF), myIp[3] | (~mask[3] & 0xFF));
  udp.beginPacket(bcast, HELLO_PORT);
  udp.print("HELLO,");
  udp.print(myIp);
  udp.print("\n");
  udp.endPacket();
}

// ---------- Main ----------
void setup() {
  Serial.begin(9600);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // don't block without USB serial

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  setMotors(0, 0);
  matrix.begin();
  clearFrame();
  for (int c = 0; c < 12; c += 2) frame[7][c] = 1;   // dotted row: booting / no signal
  matrix.renderBitmap(frame, 8, 12);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("ERROR: communication with WiFi module failed");
    wifiModule = false;
    return;   // loop() keeps running; outputs stay 0
  }
  String fv = WiFi.firmwareVersion();
  Serial.print("WiFi firmware: ");
  Serial.print(fv);
  Serial.print(" (latest known: ");
  Serial.print(WIFI_FIRMWARE_LATEST_VERSION);
  Serial.println(")");
  if (compareVersion(fv.c_str(), WIFI_FIRMWARE_LATEST_VERSION) < 0)
    Serial.println("Please update the firmware: Arduino IDE 2 -> Tools -> Firmware Updater");

  connectWifi();
}

void loop() {
  unsigned long now = millis();

  if (!wifiModule) return;

  // Wi-Fi state: wait for a non-zero IP after connecting; retry periodically when down.
  if (now - lastWifiCheckMs >= WIFI_CHECK_MS || !udpStarted) {
    lastWifiCheckMs = now;
    if (WiFi.status() == WL_CONNECTED) {
      if (!udpStarted && ipValid(WiFi.localIP())) onWifiUp();
    } else {
      if (udpStarted) {
        Serial.println("WiFi lost");
        udp.stop();
        udpStarted = false;
        haveSignal = false;
        setMotors(0, 0);
        showNoSignal();
      }
      if (now - lastWifiAttemptMs > WIFI_RETRY_MS) connectWifi();
    }
  }
  if (!udpStarted) return;

  // Drain all pending packets; act on the last valid one.
  int size;
  while ((size = udp.parsePacket()) > 0) {
    char buf[40];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0 || size >= (int)sizeof(buf)) continue;   // empty or oversized -> ignore
    buf[n] = '\0';
    unsigned long seq;
    int left, right;
    if (!parseCommand(buf, seq, left, right)) continue;
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("ACK,");
    udp.print(seq);
    udp.print("\n");
    udp.endPacket();
    lastValidMs = millis();
    haveSignal = true;
    sendHello = false;
    setMotors(left, right);
    if (left != curLeft || right != curRight) showCommand(left, right);
  }

  // Safety timer.
  if (haveSignal && millis() - lastValidMs > SAFETY_TIMEOUT_MS) {
    haveSignal = false;
    sendHello = true;
    setMotors(0, 0);
    showNoSignal();
    Serial.println("No signal: outputs off");
  }

  if (sendHello && now - lastHelloMs >= HELLO_INTERVAL_MS) {
    lastHelloMs = now;
    broadcastHello();
  }
}
