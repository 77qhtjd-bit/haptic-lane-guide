// BLE link to the belt running firmware/lane_actuator_ble (the team's measured Plan A protocol,
// see docs/phone-pipeline.md). UUIDs, packet and level mapping must match that sketch.
//
// Command (write without response, ~20Hz), 7 bytes: [seq lo, seq hi, L1, L2, L3, L4, L5]
//   M1 big left, M2 small left, M3 straight heartbeat, M4 small right, M5 big right.
//   Level per motor: 0 = off, 255 = continuously on, 1..254 = 60 ms pulses whose period goes from
//   1000 ms (level 1) to 120 ms (level 254). The board generates the pulses.
// ACK (notify), 2 bytes: the seq echoed back, used for round-trip time and stall detection.
// Board watchdog: no command for 300 ms, or a disconnect -> all motors off.

import { Pattern } from './controller.js';

export const BELT_NAME = 'LaneActuator';
export const BELT_SERVICE_UUID = '06096ed2-b23b-4ba5-9737-27dba2053749';
export const BELT_COMMAND_UUID = '14dd8746-dd9e-4308-8f98-08701035fc46';
export const BELT_ACK_UUID = '535f34b1-32f0-4532-97d1-18f2fff4bfbe';

export const PULSE_PERIOD_MAX_MS = 1000; // level 1
export const PULSE_PERIOD_MIN_MS = 120; // level 254

// Inverse of periodFor() in the firmware.
export function levelForPeriod(periodMs) {
  const t = (PULSE_PERIOD_MAX_MS - periodMs) / (PULSE_PERIOD_MAX_MS - PULSE_PERIOD_MIN_MS);
  return Math.min(254, Math.max(1, 1 + Math.round(t * 253)));
}

// HapticController command -> 5 motor levels. The firmware pulses at full duty, so the controller's
// level (PWM strength) is not transmitted; direction is the motor, magnitude is the pulse rate.
export function commandToLevels(cmd) {
  const levels = [0, 0, 0, 0, 0];
  if (cmd.pattern === Pattern.OFF || !cmd.periodMs) return levels;
  const level = levelForPeriod(cmd.periodMs);
  for (let i = 0; i < 5; i++) if (cmd.motors & (1 << i)) levels[i] = level;
  return levels;
}

export function encodeLevels(seq, levels) {
  return Uint8Array.from([seq & 0xff, (seq >> 8) & 0xff, ...levels.map((l) => Math.min(255, Math.max(0, Math.round(l))))]);
}

// Web Bluetooth link with the behavior proven in web/runner-test.html: one tap to choose the
// device, then automatic reconnects with backoff, a stall watchdog (writing but no ACK for 1 s ->
// reconnect), and reconnect-after-reload through getDevices() where the browser supports it.
// send() drops a command while the previous write is still in flight (latest-wins, never queues).
export class BeltLink {
  constructor({ onLog = () => {}, onStatus = () => {} } = {}) {
    this.onLog = onLog;
    this.onStatus = onStatus;
    this.device = null;
    this.cmdChar = null;
    this.noResponse = false;
    this.connecting = false;
    this.writing = false;
    this.attempts = 0;
    this.retryTimer = null;
    this.restored = false;
    this.seq = 0;
    this.pending = new Map();
    this.lastAckT = 0;
    this.lastWriteT = 0;
    this.rttMs = null;
    this.stats = { writes: 0, acks: 0, skipped: 0, lost: 0, reconnects: 0 };
    this.onDisconnected = this.onDisconnected.bind(this);
    this.onAck = this.onAck.bind(this);
    this.stallTimer = setInterval(() => this.checkStall(), 250);
  }

  get connected() {
    return !!this.cmdChar;
  }

  // Must run inside a tap: opens the device chooser.
  async choose() {
    if (this.cmdChar) return;
    this.use(await navigator.bluetooth.requestDevice({ filters: [{ services: [BELT_SERVICE_UUID] }] }), false);
  }

  // After a reload: reuse a previously permitted board without a tap, if supported.
  async restore() {
    if (!navigator.bluetooth?.getDevices) return false;
    const devices = await navigator.bluetooth.getDevices();
    const dev = devices.find((d) => d.name === BELT_NAME);
    if (dev && !this.device) this.use(dev, true);
    return !!dev;
  }

  use(device, restored) {
    this.drop();
    this.device = device;
    this.restored = restored;
    this.attempts = 0;
    device.addEventListener('gattserverdisconnected', this.onDisconnected);
    this.connectGatt();
  }

  drop() {
    if (!this.device) return;
    this.device.removeEventListener('gattserverdisconnected', this.onDisconnected);
    try {
      this.device.gatt.disconnect();
    } catch {}
    clearTimeout(this.retryTimer);
    this.retryTimer = null;
    this.device = null;
    this.cmdChar = null;
  }

  async connectGatt() {
    if (this.connecting || !this.device) return;
    this.connecting = true;
    this.attempts++;
    this.onStatus(`belt connecting (attempt ${this.attempts})`, false);
    try {
      const server = await withTimeout(this.device.gatt.connect(), 5000);
      const service = await server.getPrimaryService(BELT_SERVICE_UUID);
      const cmd = await service.getCharacteristic(BELT_COMMAND_UUID);
      const ack = await service.getCharacteristic(BELT_ACK_UUID);
      ack.addEventListener('characteristicvaluechanged', this.onAck);
      await ack.startNotifications();
      this.noResponse = typeof cmd.writeValueWithoutResponse === 'function';
      this.cmdChar = cmd;
      this.lastAckT = performance.now();
      this.attempts = 0;
      this.restored = false;
      this.onStatus('belt connected', true);
      this.onLog('belt connected');
    } catch (err) {
      this.onLog(`belt connect failed: ${err.name || ''} ${err.message || err}`);
      try {
        this.device.gatt.disconnect();
      } catch {}
      if (this.restored && this.attempts >= 3) {
        this.drop();
        this.onStatus('tap Connect belt', false);
      } else {
        this.scheduleRetry([500, 1000, 2000][Math.min(this.attempts - 1, 2)]);
      }
    } finally {
      this.connecting = false;
    }
  }

  scheduleRetry(ms) {
    if (this.retryTimer) return;
    this.retryTimer = setTimeout(() => {
      this.retryTimer = null;
      this.connectGatt();
    }, ms);
  }

  onDisconnected() {
    this.cmdChar = null;
    this.stats.lost += this.pending.size;
    this.pending.clear();
    this.stats.reconnects++;
    this.onStatus('belt disconnected, reconnecting', false);
    if (!this.connecting) this.scheduleRetry(500);
  }

  checkStall() {
    const t = performance.now();
    if (this.cmdChar && t - this.lastWriteT < 200 && t - this.lastAckT > 1000) {
      this.onLog(`belt stalled (no ACK for ${Math.round(t - this.lastAckT)} ms), reconnecting`);
      try {
        this.device.gatt.disconnect(); // fires gattserverdisconnected -> reconnect
      } catch {}
    }
  }

  onAck(e) {
    const seq = e.target.value.getUint16(0, true);
    const sentT = this.pending.get(seq);
    if (sentT === undefined) return;
    this.pending.delete(seq);
    this.lastAckT = performance.now();
    this.rttMs = this.lastAckT - sentT;
    this.stats.acks++;
  }

  async send(levels) {
    if (!this.cmdChar) return;
    if (this.writing) {
      this.stats.skipped++;
      return;
    }
    this.writing = true;
    this.seq = (this.seq + 1) & 0xffff;
    const packet = encodeLevels(this.seq, levels);
    try {
      this.lastWriteT = performance.now();
      this.pending.set(this.seq, this.lastWriteT);
      if (this.pending.size > 64) this.pending.delete(this.pending.keys().next().value);
      if (this.noResponse) await this.cmdChar.writeValueWithoutResponse(packet);
      else await this.cmdChar.writeValue(packet);
      this.stats.writes++;
    } catch (err) {
      this.onLog(`belt write failed: ${err.message || err}`);
    } finally {
      this.writing = false;
    }
  }
}

const withTimeout = (p, ms) =>
  Promise.race([p, new Promise((_, reject) => setTimeout(() => reject(new Error('timeout')), ms))]);
