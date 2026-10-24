// Haptic belt firmware for the Arduino UNO R4 WiFi.
//
// The phone writes 8-byte commands over BLE (layout in web/control/protocol.js); this sketch turns
// them into vibration pulses on 5 motors, generating the rhythm locally so BLE jitter never shows
// up in the pulses. No packet for 300 ms -> stop pattern (motors 1 + 5), then silence after 5 s.
// Every received packet is printed as label:value pairs for the Arduino IDE Serial Plotter.

#include <ArduinoBLE.h>

const char* SERVICE_UUID = "6e1f0001-5a5d-4c3a-9b1e-7c2f8a0d4b10";
const char* COMMAND_UUID = "6e1f0002-5a5d-4c3a-9b1e-7c2f8a0d4b10";

// Motor n (1..5, left to right) is driven by MOTOR_PINS[n - 1] through a MOSFET/ULN2003; the LED
// next to each motor can share the same pin.
const uint8_t MOTOR_PINS[5] = {3, 5, 6, 9, 10};

const uint8_t PATTERN_OFF = 0;
const uint8_t PATTERN_STEER = 1;
const uint8_t PATTERN_HEARTBEAT = 2;
const uint8_t PATTERN_STOP = 3;

const unsigned long WATCHDOG_MS = 300;
const unsigned long WATCHDOG_SILENCE_MS = 5000;
const unsigned long MIN_ON_MS = 60;   // ERM motors need time to spin up
const unsigned long MAX_ON_MS = 150;

BLEService beltService(SERVICE_UUID);
BLECharacteristic commandChar(COMMAND_UUID, BLEWrite | BLEWriteWithoutResponse, 8);

struct Command {
  uint8_t pattern = PATTERN_OFF;
  uint8_t motors = 0;
  uint8_t level = 0;
  unsigned long periodMs = 0;
};

Command current;
unsigned long lastPacketMs = 0;
unsigned long cycleStartMs = 0;
bool everReceived = false;

void setMotors(uint8_t mask, uint8_t level) {
  for (uint8_t i = 0; i < 5; i++) {
    analogWrite(MOTOR_PINS[i], (mask & (1 << i)) ? level : 0);
  }
}

void onCommandWritten(BLEDevice, BLECharacteristic) {
  const uint8_t* p = commandChar.value();
  if (commandChar.valueLength() < 8 || p[0] != 1) return;

  Command next;
  next.pattern = p[1];
  next.motors = p[2] & 0x1F;
  next.level = p[3];
  next.periodMs = (unsigned long)p[4] * 10;
  // Restart the rhythm only when the cue changes, so a steady cue keeps an even beat.
  if (next.pattern != current.pattern || next.motors != current.motors) cycleStartMs = millis();
  current = next;
  lastPacketMs = millis();
  everReceived = true;

  Serial.print("eY_cm:");
  Serial.print((int8_t)p[5]);
  Serial.print(" ePsi_deg:");
  Serial.print((int8_t)p[6]);
  Serial.print(" motor:");
  Serial.print(motorNumber(next.motors));
  Serial.print(" level:");
  Serial.print(next.level);
  Serial.print(" period_ms:");
  Serial.print(next.periodMs);
  Serial.print(" status:");
  Serial.println(p[7]);
}

// For plotting: the single active motor (1..5), 6 for motors 1+5 (stop), 0 for none.
int motorNumber(uint8_t mask) {
  if (mask == 0x11) return 6;
  for (uint8_t i = 0; i < 5; i++) {
    if (mask == (1 << i)) return i + 1;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 5; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWrite(MOTOR_PINS[i], 0);
  }

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (true) {
      setMotors(0x11, 255);  // both outer motors on = hardware problem
      delay(200);
      setMotors(0, 0);
      delay(800);
    }
  }
  BLE.setLocalName("LaneBelt");
  BLE.setDeviceName("LaneBelt");
  BLE.setAdvertisedService(beltService);
  beltService.addCharacteristic(commandChar);
  BLE.addService(beltService);
  commandChar.setEventHandler(BLEWritten, onCommandWritten);
  BLE.advertise();
  Serial.println("advertising as LaneBelt");
}

void loop() {
  BLE.poll();
  const unsigned long now = millis();

  Command cmd = current;
  if (everReceived && now - lastPacketMs > WATCHDOG_MS) {
    // Phone stopped talking: tell the runner, then go quiet so a dead phone does not buzz forever.
    if (now - lastPacketMs > WATCHDOG_SILENCE_MS) {
      cmd.pattern = PATTERN_OFF;
    } else {
      cmd.pattern = PATTERN_STOP;
      cmd.motors = 0x11;
      cmd.level = 255;
      cmd.periodMs = 120;
    }
  }

  if (cmd.pattern == PATTERN_OFF || cmd.periodMs == 0) {
    setMotors(0, 0);
    return;
  }
  unsigned long onMs = constrain(cmd.periodMs / 2, MIN_ON_MS, MAX_ON_MS);
  if (cmd.pattern == PATTERN_STOP) onMs = cmd.periodMs / 2;
  const bool on = (now - cycleStartMs) % cmd.periodMs < onMs;
  setMotors(on ? cmd.motors : 0, cmd.level);
}
