// Haptic belt actuator for Arduino UNO R4 WiFi, Bluetooth LE (team plan in HACK.md).
//
// Five motors around the waist, left to right: M1 big left turn, M2 small left turn,
// M3 straight "heartbeat", M4 small right turn, M5 big right turn.
// The phone (Bluefy, Web Bluetooth) writes a 7-byte command [seq lo, seq hi, L1..L5].
// Each level 0..255 sets that motor's pulse rate; the R4 generates the pulse timing:
//   0 = off, 255 = continuously on, 1..254 = PULSE_ON_MS pulses whose period shrinks from
//   PULSE_PERIOD_MAX_MS (level 1) to PULSE_PERIOD_MIN_MS (level 254).
// The board notifies the 2-byte seq back on the ACK characteristic (round-trip measurement).
// Watchdog: no valid command for 300 ms (or phone disconnected) -> all motors off.
//
// Matrix: dotted bottom row = advertising (not connected);
//         dotted top + bottom rows = connected, no signal;
//         one bar per motor (columns 2,4,6,8,10), lit while that motor's pulse is on.

#include <ArduinoBLE.h>
#include "Arduino_LED_Matrix.h"

// ---------- Settings ----------
const int NUM_MOTORS = 5;
const int MOTOR_PINS[NUM_MOTORS] = {3, 5, 6, 9, 10};   // PWM pins, M1..M5 (left to right)
const int MOTOR_DUTY = 255;                 // PWM duty while a pulse is on (lower if motors are too strong)
const unsigned long PULSE_ON_MS = 60;
const unsigned long PULSE_PERIOD_MAX_MS = 1000;   // level 1
const unsigned long PULSE_PERIOD_MIN_MS = 120;    // level 254
const unsigned long WATCHDOG_MS = 300;

#define SERVICE_UUID "06096ed2-b23b-4ba5-9737-27dba2053749"
#define CMD_UUID     "14dd8746-dd9e-4308-8f98-08701035fc46"
#define ACK_UUID     "535f34b1-32f0-4532-97d1-18f2fff4bfbe"
const int CMD_LEN = 2 + NUM_MOTORS;

BLEService laneService(SERVICE_UUID);
BLECharacteristic cmdChar(CMD_UUID, BLEWrite | BLEWriteWithoutResponse, CMD_LEN, true);
BLECharacteristic ackChar(ACK_UUID, BLENotify, 2, true);

ArduinoLEDMatrix matrix;
uint8_t frame[8][12];

uint8_t level[NUM_MOTORS] = {0};          // commanded pulse level per motor
bool pulseOn[NUM_MOTORS] = {false};       // motor output currently on
unsigned long pulseStart[NUM_MOTORS] = {0};
unsigned long lastValidMs = 0;
bool haveSignal = false;
bool connected = false;
bool bleOk = false;
bool matrixDirty = true;

// ---------- Outputs ----------
unsigned long periodFor(uint8_t lv) {
  return PULSE_PERIOD_MAX_MS - (unsigned long)(lv - 1) * (PULSE_PERIOD_MAX_MS - PULSE_PERIOD_MIN_MS) / 253;
}

void setMotor(int i, bool on) {
  if (pulseOn[i] == on) return;
  pulseOn[i] = on;
  analogWrite(MOTOR_PINS[i], on ? MOTOR_DUTY : 0);
  matrixDirty = true;
}

void allOff() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    level[i] = 0;
    setMotor(i, false);
  }
}

// Pulse timing for every motor, called from loop() (non-blocking).
void updatePulses() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_MOTORS; i++) {
    uint8_t lv = level[i];
    if (lv == 0) {
      setMotor(i, false);
    } else if (lv == 255) {
      setMotor(i, true);
    } else if (pulseOn[i]) {
      if (now - pulseStart[i] >= PULSE_ON_MS) setMotor(i, false);
    } else if (now - pulseStart[i] >= periodFor(lv)) {
      pulseStart[i] = now;
      setMotor(i, true);
    }
  }
}

void clearFrame() { memset(frame, 0, sizeof(frame)); }

void drawMatrix() {
  clearFrame();
  if (!haveSignal) {
    for (int c = 0; c < 12; c += 2) {
      frame[7][c] = 1;
      if (connected) frame[0][c] = 1;
    }
  } else {
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (!pulseOn[i]) continue;
      int h = 1 + level[i] * 7 / 255;            // bar height 1..8 shows the level
      for (int r = 0; r < h; r++) frame[7 - r][2 + 2 * i] = 1;
    }
  }
  matrix.renderBitmap(frame, 8, 12);
  matrixDirty = false;
}

void signalLost() {
  haveSignal = false;
  allOff();
  matrixDirty = true;
}

// ---------- BLE events ----------
void onConnect(BLEDevice central) {
  connected = true;
  Serial.print("Connected: ");
  Serial.println(central.address());
  signalLost();
}

void onDisconnect(BLEDevice central) {
  connected = false;
  Serial.println("Disconnected");
  signalLost();
  BLE.advertise();   // make sure the phone can reconnect
}

// ---------- Main ----------
void setup() {
  Serial.begin(9600);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // don't block without USB serial

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWrite(MOTOR_PINS[i], 0);
  }
  matrix.begin();
  drawMatrix();

  if (!BLE.begin()) {
    Serial.println("ERROR: BLE.begin() failed (update the WiFi/BLE module firmware?)");
    return;   // loop() keeps running; outputs stay 0
  }
  bleOk = true;
  BLE.setLocalName("LaneActuator");
  BLE.setDeviceName("LaneActuator");
  BLE.setAdvertisedService(laneService);
  laneService.addCharacteristic(cmdChar);
  laneService.addCharacteristic(ackChar);
  BLE.addService(laneService);
  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  BLE.advertise();
  Serial.println("Advertising as LaneActuator (5 motors)");
}

void loop() {
  if (!bleOk) return;
  BLE.poll();

  if (cmdChar.written()) {
    const uint8_t *v = cmdChar.value();
    if (cmdChar.valueLength() == CMD_LEN) {
      ackChar.writeValue(v, 2);   // ACK first: echo the seq
      lastValidMs = millis();
      if (!haveSignal) matrixDirty = true;
      haveSignal = true;
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (level[i] == 0 && v[2 + i] > 0) pulseStart[i] = millis() - PULSE_PERIOD_MAX_MS;   // start a pulse now
        level[i] = v[2 + i];
      }
    }
  }

  if (haveSignal && millis() - lastValidMs > WATCHDOG_MS) {
    signalLost();
    Serial.println("Watchdog: no command for 300 ms, motors off");
  }

  updatePulses();
  if (matrixDirty) drawMatrix();
}
