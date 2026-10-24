// PD + heading control and the 5-motor haptic mapping.
//
// Motors around the waist, left to right:
//   1 = big left turn, 2 = small left turn, 3 = straight, 4 = small right turn, 5 = big right turn
// A motor on the side the runner should move toward vibrates; pulses get faster as the error grows.
// Inside the deadband motor 3 gives a slow heartbeat so the runner knows the system is alive.

import { LaneStatus } from '../lane/tracker.js';

export const Pattern = Object.freeze({ OFF: 0, STEER: 1, HEARTBEAT: 2, STOP: 3 });

export const motorBit = (n) => 1 << (n - 1);

export const DEFAULT_CONTROL_CONFIG = {
  kp: 1.0, // on predicted eY (m -> m)
  kd: 0.3, // s, on vY
  reactionTime: 0.25, // s: look ahead by the runner's reaction delay
  // Pure pursuit: aim at the centerline point this far ahead. Heading error and curvature both act
  // through it, so on a bend the cue already says "turn" while the runner is still centered
  // (with 1.5 m and no shared aim point, a 12 m bend stayed under the deadband and read STRAIGHT).
  lookahead: 2.5, // m
  deadband: 0.15, // m of |u|: silent zone (heartbeat only)
  strong: 0.35, // m of |u|: big-turn zone starts here
  hysteresis: 0.05, // m: leave a zone only this far below its entry threshold
  saturate: 0.6, // m of |u|: fastest pulses from here on
  slowPeriodMs: 600,
  fastPeriodMs: 150,
  weakLevel: 170,
  strongLevel: 255,
  heartbeatPeriodMs: 1000,
  heartbeatLevel: 90,
  stopPeriodMs: 120,
};

export class HapticController {
  constructor(config = {}) {
    this.cfg = { ...DEFAULT_CONTROL_CONFIG, ...config };
    this.zone = 0; // 0 straight, 1 small turn, 2 big turn
  }

  // Lateral position (m, + = right) of the aim point: the centerline `lookahead` meters ahead, seen
  // from the runner's straight-ahead path. Centerline at the runner is at -eY, its direction is
  // -ePsi relative to the runner, and it bends by curvature/2 * L^2 over L meters.
  aimPoint({ eY, ePsi = 0 }, curvature = 0) {
    const L = this.cfg.lookahead;
    return { x: L, y: -eY - Math.tan(ePsi) * L + (curvature / 2) * L * L };
  }

  // Control effort in meters: + = the runner should move left. Pure pursuit toward the aim point,
  // plus a damping term on lateral speed and the reaction-time lead on eY.
  effort(state, curvature = 0) {
    const { kp, kd, reactionTime } = this.cfg;
    const vY = state.vY ?? 0;
    const aim = this.aimPoint({ ...state, eY: state.eY + vY * reactionTime }, curvature);
    return -kp * aim.y + kd * vY;
  }

  // state: LaneEstimator.state; status: LaneTracker status; curvature: LaneTracker curvature (1/m).
  step(state, status, curvature = 0) {
    const c = this.cfg;
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

    const u = this.effort(state, curvature);
    let goLeft = u > 0;
    const mag = Math.abs(u);
    if (status === LaneStatus.OUTSIDE_LEFT || status === LaneStatus.OUTSIDE_RIGHT) {
      // Over the line: always the strongest cue back toward the locked lane.
      this.zone = 2;
      goLeft = status === LaneStatus.OUTSIDE_RIGHT;
    } else {
      this.zone = nextZone(this.zone, mag, c);
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
