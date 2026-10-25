// Audio comparison cue: the same command the belt gets, played as stereo beeps, so runs with audio
// feedback can be compared against the belt. Direction = left/right ear, magnitude = pitch + tempo.

import { Pattern } from './controller.js';

// Pan per motor (1..5) and pitch per zone.
const PAN = [-1, -0.5, 0, 0.5, 1];
const PITCH_HZ = { heartbeat: 440, weak: 660, strong: 880, stop: 1000 };

export class AudioCue {
  constructor() {
    this.ctx = null;
  }

  // Must be called from a user gesture (iOS will not start audio otherwise).
  start() {
    if (this.ctx) return;
    const Ctx = window.AudioContext || window.webkitAudioContext;
    this.ctx = new Ctx();
    this.osc = this.ctx.createOscillator();
    this.gain = this.ctx.createGain();
    this.pan = this.ctx.createStereoPanner();
    this.gain.gain.value = 0;
    this.osc.connect(this.gain).connect(this.pan).connect(this.ctx.destination);
    this.osc.start();
  }

  // Call every frame; on/off follows the same pulse rhythm the belt firmware produces.
  update(cmd, enabled, nowMs = performance.now()) {
    if (!this.ctx) return;
    const t = this.ctx.currentTime;
    const on = enabled && cmd.pattern !== Pattern.OFF && cmd.periodMs > 0 && nowMs % cmd.periodMs < cmd.periodMs / 2;
    let freq = PITCH_HZ.weak;
    let pan = 0;
    if (cmd.pattern === Pattern.HEARTBEAT) freq = PITCH_HZ.heartbeat;
    else if (cmd.pattern === Pattern.STOP) {
      freq = PITCH_HZ.stop;
      pan = Math.floor(nowMs / cmd.periodMs) % 2 ? 1 : -1; // alternate ears
    } else if (cmd.pattern === Pattern.STEER || cmd.pattern === Pattern.CURVE) {
      freq = cmd.zone === 2 ? PITCH_HZ.strong : PITCH_HZ.weak;
      pan = PAN[Math.log2(cmd.motors & -cmd.motors)] ?? 0;
    }
    this.osc.frequency.setTargetAtTime(freq, t, 0.01);
    this.pan.pan.setTargetAtTime(pan, t, 0.01);
    this.gain.gain.setTargetAtTime(on ? (cmd.pattern === Pattern.HEARTBEAT ? 0.08 : 0.25) : 0, t, 0.005);
  }
}
