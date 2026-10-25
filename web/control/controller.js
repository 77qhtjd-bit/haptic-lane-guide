// PD + heading control and the 5-motor haptic mapping.
//
// Motors around the waist, left to right:
//   1 = big left turn, 2 = small left turn, 3 = straight, 4 = small right turn, 5 = big right turn
// A motor on the side the runner should move toward vibrates; pulses get faster as the error grows.
// Inside the deadband motor 3 gives a slow heartbeat so the runner knows the system is alive.

import { LaneStatus } from '../lane/tracker.js';

export const Pattern = Object.freeze({ OFF: 0, STEER: 1, HEARTBEAT: 2, STOP: 3, CURVE: 4 });

export const motorBit = (n) => 1 << (n - 1);

export const DEFAULT_CONTROL_CONFIG = {
  kp: 1.0, // on predicted eY (m -> m)
  // Jogging (measured on real-track clips, docs/phone-pipeline.md): the chest camera yaws ±15° at
  // stride frequency (1.4 Hz) and the measured heading carries a steady +3..6° offset (mount yaw).
  // Through a 2.5 m pure-pursuit lookahead that became ±50-90 cm of effort: the belt flipped
  // left/right with every stride (~390 cue changes/min while inside the lane). So the cue is driven
  // by the stride-averaged lateral offset, the heading only after a stride-cancelling average minus
  // its slowly learned offset and through a short lookahead, and the curvature as before.
  kd: 0, // s, on vY (vY follows the stride sway; 0.3 in the original pure-pursuit tuning)
  reactionTime: 0, // s: lead on eY from vY (0.25 originally)
  positionAvgS: 0.7, // s: moving average of eY, about one stride (two steps) when jogging
  headingAvgS: 2, // s: moving average of ePsi, several strides
  headingBiasS: 15, // s: time constant of the learned steady heading offset (0 = no correction)
  headingLookahead: 1.0, // m: weight of the averaged heading error (2.5 in pure pursuit)
  // Curvature acts through this lookahead, so on a 12 m bend the cue already says "turn" while the
  // runner is still centered; a 36.5 m track bend alone stays under the deadband.
  lookahead: 2.5, // m (curvature lookahead; aim point drawn this far ahead)
  deadband: 0.2, // m of |u|: silent zone (heartbeat only); 0.15 with pure pursuit, 0.2 measured best with the averaged cue
  strong: 0.4, // m of |u|: big-turn zone starts here (0.35 with pure pursuit)
  hysteresis: 0.05, // m: leave a zone only this far below its entry threshold
  saturate: 0.6, // m of |u|: fastest pulses from here on
  slowPeriodMs: 600,
  fastPeriodMs: 150,
  weakLevel: 170,
  strongLevel: 255,
  heartbeatPeriodMs: 1000,
  heartbeatLevel: 90,
  stopPeriodMs: 120,
  // Curve guidance (needs frame times): on a bend the runner must keep turning, so "centered" is
  // not silence/heartbeat but a gentle cue toward the bend on the small-turn motor (M2 for a left
  // bend, M4 right), announced by a short double tap. Measured on the real-track clips: bends read
  // |curvature| 0.015-0.024 /m, straights under 0.005 /m; the camera sees the bend ~1-1.5 s ahead.
  curveGuide: 1, // 0 = off (centered on a bend gives the heartbeat, as before)
  // Track running is counterclockwise, so bends are left; guiding only left bends halved the false
  // curve cues on straights (7.9 -> 3.9 s/min on the clips) without delaying any bend. 0 = both.
  curveSides: -1,
  curveOnK: 0.0125, // 1/m (R 80 m): averaged curvature above this starts curve guidance
  curveOffK: 0.0067, // 1/m (R 150 m): below this it ends; in between the state is kept
  curveAvgS: 1, // s: curvature averaging window
  curveHoldS: 0.5, // s: a new curve state must persist this long before it is used
  curvePeriodMs: 800, // gentle pulses while centered on a bend
  curveStartMs: 400, // the first curveStartMs of a bend: fast pulses (a double tap)
  curveStartPeriodMs: 150,
};

export class HapticController {
  constructor(config = {}) {
    this.cfg = { ...DEFAULT_CONTROL_CONFIG, ...config };
    this.zone = 0; // 0 straight, 1 small turn, 2 big turn
    this.reset();
  }

  // Forget the averaging history (new lock, lost lane).
  reset() {
    this.history = []; // { t, eY, ePsi } over the last max(positionAvgS, headingAvgS) seconds
    this.headingBias = 0;
    this.lastT = null;
    this.curveHist = []; // { t, k }
    this.curveSide = 0; // -1 left bend, +1 right bend, 0 straight
    this.curvePending = null;
    this.curvePendingSince = 0;
    this.curveStartUntil = 0;
  }

  // Curve state from the averaged curvature with on/off thresholds and a hold time.
  updateCurve(curvature, tMs) {
    const c = this.cfg;
    this.curveHist.push({ t: tMs, k: curvature || 0 });
    while (this.curveHist.length && this.curveHist[0].t < tMs - c.curveAvgS * 1000) this.curveHist.shift();
    const k = this.curveHist.reduce((acc, h) => acc + h.k, 0) / this.curveHist.length;
    let want = this.curveSide;
    if (Math.abs(k) >= c.curveOnK) want = Math.sign(k);
    else if (Math.abs(k) <= c.curveOffK) want = 0;
    if (c.curveSides && want !== 0 && want !== c.curveSides) want = 0;
    if (want === this.curveSide) { this.curvePending = null; return; }
    if (this.curvePending !== want) { this.curvePending = want; this.curvePendingSince = tMs; return; }
    if (tMs - this.curvePendingSince >= c.curveHoldS * 1000) {
      if (want !== 0) this.curveStartUntil = tMs + c.curveStartMs;
      this.curveSide = want;
      this.curvePending = null;
    }
  }

  // Averages eY and ePsi over their windows ending at tMs, and learns the steady heading offset.
  // Without a time (tests, legacy callers) the current values are used as they are.
  filtered(state, tMs) {
    const c = this.cfg;
    if (tMs == null) return { eY: state.eY, ePsi: state.ePsi ?? 0 };
    this.history.push({ t: tMs, eY: state.eY, ePsi: state.ePsi ?? 0 });
    const keep = Math.max(c.positionAvgS, c.headingAvgS) * 1000;
    while (this.history.length && this.history[0].t < tMs - keep) this.history.shift();
    const mean = (key, windowS) => {
      const from = tMs - windowS * 1000;
      let sum = 0, n = 0;
      for (const h of this.history) if (h.t >= from) { sum += h[key]; n++; }
      return n ? sum / n : state[key] ?? 0;
    };
    const eY = mean('eY', c.positionAvgS);
    let ePsi = mean('ePsi', c.headingAvgS);
    if (c.headingBiasS > 0) {
      const dt = this.lastT == null ? 0 : Math.max(0, (tMs - this.lastT) / 1000);
      this.headingBias += Math.min(1, dt / c.headingBiasS) * (ePsi - this.headingBias);
      ePsi -= this.headingBias;
    }
    this.lastT = tMs;
    return { eY, ePsi };
  }

  // Lateral position (m, + = right) of the aim point, seen from the runner's straight-ahead path:
  // the centerline at the runner is at -eY, heading error ePsi acts over headingLookahead and the
  // curvature over lookahead (both lookaheads 2.5 m = the original pure pursuit).
  aimPoint({ eY, ePsi = 0 }, curvature = 0) {
    const { lookahead: L, headingLookahead: Lh } = this.cfg;
    return { x: L, y: -eY - Math.tan(ePsi) * Lh + (curvature / 2) * L * L };
  }

  // Control effort in meters: + = the runner should move left. Toward the aim point, plus an
  // optional damping term on lateral speed and reaction-time lead on eY (both off by default).
  effort(state, curvature = 0) {
    const { kp, kd, reactionTime } = this.cfg;
    const vY = state.vY ?? 0;
    const aim = this.aimPoint({ ...state, eY: state.eY + vY * reactionTime }, curvature);
    return -kp * aim.y + kd * vY;
  }

  // state: LaneEstimator.state; status: LaneTracker status; curvature: LaneTracker curvature (1/m).
  // tMs: frame time (ms); when given, eY and ePsi are averaged as described in the config.
  // lowConfidence (just re-locked): the cue stays at most in the small-turn zone.
  step(state, status, curvature = 0, tMs = null, lowConfidence = false) {
    const c = this.cfg;
    if (status === LaneStatus.UNLOCKED || status === LaneStatus.LOST) this.reset();
    if (status === LaneStatus.UNLOCKED) {
      this.zone = 0;
      return { pattern: Pattern.OFF, motors: 0, level: 0, periodMs: 0, zone: 0, u: 0 };
    }
    if (status === LaneStatus.LOST) {
      this.zone = 0;
      return {
        pattern: Pattern.STOP,
        motors: motorBit(1) | motorBit(5),
        level: c.strongLevel,
        periodMs: c.stopPeriodMs,
        zone: 0,
        u: 0,
      };
    }

    const f = this.filtered(state, tMs);
    if (c.curveGuide && tMs != null) this.updateCurve(curvature, tMs);
    this.lastAim = this.aimPoint({ ...state, eY: f.eY, ePsi: f.ePsi }, curvature); // what the cue steers toward (for drawing)
    const u = this.effort({ ...state, eY: f.eY, ePsi: f.ePsi }, curvature);
    let goLeft = u > 0;
    const mag = Math.abs(u);
    if (status === LaneStatus.OUTSIDE_LEFT || status === LaneStatus.OUTSIDE_RIGHT) {
      // Over the line: always the strongest cue back toward the locked lane.
      this.zone = 2;
      goLeft = status === LaneStatus.OUTSIDE_RIGHT;
    } else {
      this.zone = nextZone(this.zone, mag, c);
    }
    if (lowConfidence) this.zone = Math.min(this.zone, 1);

    if (this.zone === 0 && c.curveGuide && tMs != null && this.curveSide !== 0) {
      // Centered on a bend: keep turning, gently, on the small-turn motor toward the bend.
      return {
        pattern: Pattern.CURVE,
        motors: motorBit(this.curveSide < 0 ? 2 : 4),
        level: c.weakLevel,
        periodMs: tMs < this.curveStartUntil ? c.curveStartPeriodMs : c.curvePeriodMs,
        zone: 0,
        u,
        curve: this.curveSide,
      };
    }
    if (this.zone === 0) {
      return {
        pattern: Pattern.HEARTBEAT,
        motors: motorBit(3),
        level: c.heartbeatLevel,
        periodMs: c.heartbeatPeriodMs,
        zone: 0,
        u,
      };
    }
    const motor = this.zone === 2 ? (goLeft ? 1 : 5) : goLeft ? 2 : 4;
    const t = Math.min(1, Math.max(0, (mag - c.deadband) / (c.saturate - c.deadband)));
    return {
      pattern: Pattern.STEER,
      motors: motorBit(motor),
      level: this.zone === 2 ? c.strongLevel : c.weakLevel,
      periodMs: Math.round(c.slowPeriodMs + (c.fastPeriodMs - c.slowPeriodMs) * t),
      zone: this.zone,
      u,
    };
  }
}

// Zones with hysteresis so the cue does not flicker when |u| sits on a threshold.
function nextZone(zone, mag, { deadband, strong, hysteresis }) {
  if (mag >= strong) return 2;
  if (zone === 2 && mag > strong - hysteresis) return 2;
  if (mag >= deadband) return 1;
  if (zone >= 1 && mag > deadband - hysteresis) return 1;
  return 0;
}
