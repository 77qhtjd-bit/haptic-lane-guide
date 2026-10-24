// BLE link to the UNO R4 WiFi belt (firmware/belt_r4). Must match the UUIDs and packet layout there.
//
// Packet, 8 bytes, written without response at ~20Hz:
//   [0] version (1)
//   [1] pattern: 0 off, 1 steer, 2 heartbeat, 3 stop
//   [2] motor mask: bit0 = motor 1 ... bit4 = motor 5
//   [3] level 0..255 (PWM)
//   [4] pulse period / 10 ms (0..255 -> 0..2550 ms)
//   [5] eY in cm, int8 (telemetry for the Serial Plotter)
//   [6] ePsi in degrees, int8 (telemetry)
//   [7] status code (telemetry): 0 unlocked, 1 inside, 2 outside-left, 3 outside-right, 4 lost
// The firmware generates the pulse rhythm itself, so BLE jitter never changes the rhythm, and it
// switches to the stop pattern if no packet arrives for 300 ms.

import { LaneStatus } from '../lane/tracker.js';

export const BELT_SERVICE_UUID = '6e1f0001-5a5d-4c3a-9b1e-7c2f8a0d4b10';
export const BELT_COMMAND_UUID = '6e1f0002-5a5d-4c3a-9b1e-7c2f8a0d4b10';
export const PROTOCOL_VERSION = 1;

const STATUS_CODES = {
  [LaneStatus.UNLOCKED]: 0,
  [LaneStatus.INSIDE]: 1,
  [LaneStatus.OUTSIDE_LEFT]: 2,
  [LaneStatus.OUTSIDE_RIGHT]: 3,
  [LaneStatus.LOST]: 4,
};

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const int8 = (v) => clamp(Math.round(v), -128, 127) & 0xff;

export function encodeCommand(cmd, { eY = 0, ePsi = 0, status = LaneStatus.UNLOCKED } = {}) {
  return Uint8Array.from([
    PROTOCOL_VERSION,
    cmd.pattern,
    cmd.motors & 0x1f,
    clamp(Math.round(cmd.level), 0, 255),
    clamp(Math.round(cmd.periodMs / 10), 0, 255),
    int8(eY * 100),
    int8((ePsi * 180) / Math.PI),
    STATUS_CODES[status] ?? 0,
  ]);
}

// Web Bluetooth connection (Bluefy on iPhone, Chrome on Ubuntu). send() is rate-limited; the
// latest packet wins, so a slow link drops stale commands instead of queueing them.
export class BeltLink {
  constructor({ minIntervalMs = 50, onLog = () => {} } = {}) {
    this.minIntervalMs = minIntervalMs;
    this.onLog = onLog;
    this.characteristic = null;
    this.pending = null;
    this.busy = false;
    this.lastSent = 0;
  }

  get connected() {
    return !!this.characteristic;
  }

  async connect() {
    const device = await navigator.bluetooth.requestDevice({ filters: [{ services: [BELT_SERVICE_UUID] }] });
    device.addEventListener('gattserverdisconnected', () => {
      this.characteristic = null;
      this.onLog('belt disconnected');
    });
    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(BELT_SERVICE_UUID);
    this.characteristic = await service.getCharacteristic(BELT_COMMAND_UUID);
    this.onLog(`belt connected: ${device.name || device.id}`);
  }

  send(packet) {
    this.pending = packet;
    this.flush();
  }

  async flush() {
    if (this.busy || !this.pending || !this.characteristic) return;
    const wait = this.lastSent + this.minIntervalMs - performance.now();
    if (wait > 0) {
      setTimeout(() => this.flush(), wait);
      return;
    }
    const packet = this.pending;
    this.pending = null;
    this.busy = true;
    try {
      const ch = this.characteristic;
      await (ch.writeValueWithoutResponse ? ch.writeValueWithoutResponse(packet) : ch.writeValue(packet));
      this.lastSent = performance.now();
    } catch (err) {
      this.onLog(`belt write failed: ${err.message || err}`);
    } finally {
      this.busy = false;
    }
    if (this.pending) this.flush();
  }
}
