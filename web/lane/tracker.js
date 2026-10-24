// Step 2 of perception: lock the runner's own lane, then keep following those same two lines.
//
// Every lane looks identical, so a single frame cannot tell lane 3 from lane 4. The lane's
// identity comes from continuity: each frame only accepts peaks near where the locked lines are
// predicted to be, so a neighbor lane's line is never picked up, and a runner who crosses a line
// is reported as outside the lane instead of being silently re-locked onto the next one.
//
// Units: meters and radians in the heading-aligned ground frame of GroundLineDetector, where the
// runner (camera foot point) is at y = 0 and peaks are perpendicular offsets of the lines.
// Outputs follow one sign convention: positive = to the right.
//   eY:   runner's lateral offset from the lane centerline (+ = runner is right of center)
//   ePsi: runner's heading relative to the lane (+ = runner points right of the lane direction)

import { LANE_WIDTH_M } from './ground.js';

export const DEFAULT_TRACKER_CONFIG = {
  laneWidth: LANE_WIDTH_M,
  lockWidthTolerance: 0.25, // fraction of laneWidth allowed between L and R at lock time
  gate: 0.25, // m: max distance between a predicted line and the peak it is matched to
  gateGrowthPerFrame: 0.01, // m: the gate widens while coasting, capped below half a lane
  alpha: 0.5, // alpha-beta filter on the lane center (until the Kalman filter replaces it)
  beta: 0.1,
  widthAlpha: 0.05, // lane width adapts slowly, only when both lines are seen
  coastFrames: 30, // ~1s at 30fps without a matched line, then 'lost'
};

export const LaneStatus = Object.freeze({
  UNLOCKED: 'unlocked',
  INSIDE: 'inside',
  OUTSIDE_LEFT: 'outside-left',
  OUTSIDE_RIGHT: 'outside-right',
  LOST: 'lost',
});

export class LaneTracker {
  constructor(config = {}) {
    this.cfg = { ...DEFAULT_TRACKER_CONFIG, ...config };
    this.reset();
  }

  reset() {
    this.locked = false;
    this.center = 0; // lateral position of the lane centerline relative to the runner
    this.centerRate = 0; // per frame
    this.width = this.cfg.laneWidth;
    this.heading = 0;
    this.missed = 0;
    this.last = { status: LaneStatus.UNLOCKED };
  }

  // Locks onto the nearest line on each side of the runner. Returns true on success.
  lock(peaks, heading) {
    const { laneWidth, lockWidthTolerance } = this.cfg;
    const left = nearest(peaks.filter((p) => p.y < 0), 0);
    const right = nearest(peaks.filter((p) => p.y >= 0), 0);
    if (!left || !right) return false;
    const width = right.y - left.y;
    if (Math.abs(width - laneWidth) > lockWidthTolerance * laneWidth) return false;
    this.reset();
    this.locked = true;
    this.center = (left.y + right.y) / 2;
    this.width = width;
    this.heading = heading;
    this.last = this.output(left, right, 1);
    return true;
  }

  // peaks: GroundLineDetector peaks for this frame ([] when the frame was skipped or failed).
  update(peaks, heading) {
    if (!this.locked) return this.last;
    if (this.last.status === LaneStatus.LOST) return this.last;

    const { gate, gateGrowthPerFrame, alpha, beta, widthAlpha, coastFrames } = this.cfg;
    const predicted = this.center + this.centerRate;
    const half = this.width / 2;
    const g = Math.min(gate + this.missed * gateGrowthPerFrame, 0.45 * this.width);
    const left = nearestWithin(peaks, predicted - half, g);
    const right = nearestWithin(peaks, predicted + half, g);

    let measured = null;
    let confidence = 0;
    if (left && right && right.y - left.y > 0.5 * this.width) {
      measured = (left.y + right.y) / 2;
      this.width += widthAlpha * (right.y - left.y - this.width);
      confidence = 1;
    } else if (left || right) {
      measured = left ? left.y + half : right.y - half;
      confidence = 0.6;
    }

    if (measured === null) {
      this.center = predicted;
      this.missed++;
      if (this.missed > coastFrames) {
        this.last = { ...this.output(null, null, 0), status: LaneStatus.LOST };
        return this.last;
      }
      confidence = Math.max(0, 0.5 * (1 - this.missed / coastFrames));
    } else {
      const residual = measured - predicted;
      this.center = predicted + alpha * residual;
      this.centerRate += beta * residual;
      this.heading = heading;
      this.missed = 0;
    }
    this.last = this.output(left, right, confidence);
    return this.last;
  }

  output(left, right, confidence) {
    const half = this.width / 2;
    const leftY = this.center - half;
    const rightY = this.center + half;
    let status = LaneStatus.INSIDE;
    if (leftY > 0) status = LaneStatus.OUTSIDE_LEFT;
    else if (rightY < 0) status = LaneStatus.OUTSIDE_RIGHT;
    return {
      status,
      eY: -this.center,
      ePsi: -this.heading,
      center: this.center,
      leftY,
      rightY,
      width: this.width,
      heading: this.heading,
      matched: { left: !!left, right: !!right },
      confidence,
    };
  }
}

function nearest(peaks, y) {
  let best = null;
  for (const p of peaks) if (!best || Math.abs(p.y - y) < Math.abs(best.y - y)) best = p;
  return best;
}

function nearestWithin(peaks, y, gate) {
  const p = nearest(peaks, y);
  return p && Math.abs(p.y - y) <= gate ? p : null;
}
