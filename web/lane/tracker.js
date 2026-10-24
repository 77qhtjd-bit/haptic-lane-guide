// Step 2 of perception: lock the runner's own lane, then keep following those same two lines,
// straight or curved.
//
// Every lane looks identical, so a single frame cannot tell lane 3 from lane 4. The lane's
// identity comes from continuity: each frame only accepts peaks near where the locked lines are
// predicted to be, so a neighbor lane's line is never picked up, and a runner who crosses a line
// is reported as outside the lane instead of being silently re-locked onto the next one.
//
// Lane model, in the heading-aligned ground frame of GroundLineDetector (x forward, y right, the
// runner at the origin): both lines are parallel curves
//   y(x) = a + b·x + c·x² ∓ width/2        (L: minus, R: plus)
// fitted to per-segment peaks (1-2 m, 2-3 m, ... ahead), like lane fitting in driving. With fewer
// segments the model drops to a line (a, b) or an offset (a). A track bend (radius ~36.5 m) moves
// a line ~22 cm off a straight extrapolation 4 m ahead, more than the 15 cm deadband.
//
// Outputs follow one sign convention: positive = to the right.
//   eY:        runner's lateral offset from the centerline (+ = runner is right of center)
//   ePsi:      runner's heading relative to the lane tangent (+ = runner points right of the lane)
//   curvature: 1/m, + = lane bends right

import { LANE_WIDTH_M } from './ground.js';

export const DEFAULT_TRACKER_CONFIG = {
  laneWidth: LANE_WIDTH_M,
  lockWidthTolerance: 0.25, // fraction of laneWidth allowed between L and R at lock time
  gate: 0.25, // m: max distance between a predicted line and the peak it is matched to
  gateGrowthPerFrame: 0.01, // m: the gate widens while coasting, capped below half a lane
  alpha: 0.5, // alpha-beta filter on the lane offset a (the Kalman filter smooths further)
  beta: 0.1,
  shapeAlpha: 0.4, // smoothing of slope b and curve c
  maxCurvature: 0.1, // 1/m (10 m radius): anything sharper is a bad fit, not a track
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
    this.center = 0; // a: centerline offset at the runner
    this.centerRate = 0; // per frame
    this.slope = 0; // b
    this.curve = 0; // c
    this.width = this.cfg.laneWidth;
    this.heading = 0; // rotation of the aligned frame (from the detector's heading search)
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
    this.last = this.output({ left: true, right: true }, 1);
    return true;
  }

  // peaks: detector peaks over the whole band (used when segments are not given: straight model).
  // segments: detector segments [{ x, peaks }] for the curved model.
  update(peaks, heading, segments = null) {
    if (!this.locked) return this.last;
    if (this.last.status === LaneStatus.LOST) return this.last;

    const { gate, gateGrowthPerFrame, alpha, beta, shapeAlpha, maxCurvature, widthAlpha, coastFrames } = this.cfg;
    // Re-express the previous slope in this frame's rotation.
    const slope = Math.tan(Math.atan(this.slope) + this.heading - heading);
    const predicted = this.center + this.centerRate;
    const half = this.width / 2;
    const g = Math.min(gate + this.missed * gateGrowthPerFrame, 0.45 * this.width);
    const curveAt = (x) => predicted + slope * x + this.curve * x * x;

    // Gated association per segment; legacy callers pass one band of peaks taken as x = 0.
    const segs = segments ?? [{ x: 0, peaks }];
    const obs = []; // { x, y (centerline-equivalent), side }
    const matched = { left: false, right: false };
    const widths = [];
    for (const seg of segs) {
      const mid = curveAt(seg.x);
      const left = nearestWithin(seg.peaks, mid - half, g);
      const right = nearestWithin(seg.peaks, mid + half, g);
      if (left) {
        obs.push({ x: seg.x, y: left.y + half });
        matched.left = true;
      }
      if (right) {
        obs.push({ x: seg.x, y: right.y - half });
        matched.right = true;
      }
      if (left && right && right.y - left.y > 0.5 * this.width) widths.push(right.y - left.y);
    }

    if (obs.length === 0) {
      this.center = predicted;
      this.slope = slope;
      this.heading = heading;
      this.missed++;
      if (this.missed > coastFrames) {
        this.last = { ...this.output({ left: false, right: false }, 0), status: LaneStatus.LOST };
        return this.last;
      }
      this.last = this.output(matched, Math.max(0, 0.5 * (1 - this.missed / coastFrames)));
      return this.last;
    }

    const fit = fitCenterline(obs, this.curve);
    const residual = fit.a - predicted;
    this.center = predicted + alpha * residual;
    this.centerRate += beta * residual;
    // Without two distinct distances the slope is unobservable here; the detector's heading search
    // already measured the lane direction, so the aligned frame itself is the tangent (b = 0).
    this.slope = fit.hasB ? slope + shapeAlpha * (fit.b - slope) : 0;
    const c = Math.max(-maxCurvature / 2, Math.min(maxCurvature / 2, fit.c));
    this.curve += shapeAlpha * (c - this.curve);
    if (widths.length) {
      const w = widths.reduce((s, v) => s + v, 0) / widths.length;
      this.width += widthAlpha * (w - this.width);
    }
    this.heading = heading;
    this.missed = 0;
    const segmentsSeen = new Set(obs.map((o) => o.x)).size;
    const confidence = matched.left && matched.right ? 1 : segmentsSeen >= 2 ? 0.7 : 0.6;
    this.last = this.output(matched, confidence);
    return this.last;
  }

  output(matched, confidence) {
    const half = this.width / 2;
    const leftY = this.center - half;
    const rightY = this.center + half;
    let status = LaneStatus.INSIDE;
    if (leftY > 0) status = LaneStatus.OUTSIDE_LEFT;
    else if (rightY < 0) status = LaneStatus.OUTSIDE_RIGHT;
    const tangent = this.heading + Math.atan(this.slope);
    return {
      status,
      eY: -this.center,
      ePsi: -tangent,
      curvature: (2 * this.curve) / (1 + this.slope * this.slope) ** 1.5,
      center: this.center,
      leftY,
      rightY,
      width: this.width,
      heading: this.heading,
      shape: { a: this.center, b: this.slope, c: this.curve }, // y(x) in the aligned frame
      matched: { left: !!matched.left, right: !!matched.right },
      confidence,
    };
  }
}

// Least squares of y = a + b·x + c·x² over centerline-equivalent observations. Drops to a line or
// an offset when the observations do not span enough distinct distances; the curve term then
// keeps its previous value.
function fitCenterline(obs, prevC) {
  const xs = [...new Set(obs.map((o) => o.x))];
  if (xs.length >= 3) {
    const [a, b, c] = solveLeastSquares(obs, (x) => [1, x, x * x]);
    return { a, b, c, hasB: true };
  }
  if (xs.length === 2) {
    const [a, b] = solveLeastSquares(obs.map((o) => ({ x: o.x, y: o.y - prevC * o.x * o.x })), (x) => [1, x]);
    return { a, b, c: prevC, hasB: true };
  }
  const mean = obs.reduce((s, o) => s + o.y - prevC * o.x * o.x, 0) / obs.length;
  return { a: mean, b: 0, c: prevC, hasB: false };
}

function solveLeastSquares(obs, basis) {
  const n = basis(0).length;
  const A = Array.from({ length: n }, () => new Array(n).fill(0));
  const v = new Array(n).fill(0);
  for (const { x, y } of obs) {
    const f = basis(x);
    for (let i = 0; i < n; i++) {
      v[i] += f[i] * y;
      for (let j = 0; j < n; j++) A[i][j] += f[i] * f[j];
    }
  }
  // Gaussian elimination with partial pivoting (n <= 3).
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    [A[col], A[piv]] = [A[piv], A[col]];
    [v[col], v[piv]] = [v[piv], v[col]];
    for (let r = col + 1; r < n; r++) {
      const f = A[r][col] / A[col][col];
      for (let k = col; k < n; k++) A[r][k] -= f * A[col][k];
      v[r] -= f * v[col];
    }
  }
  const out = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let acc = v[r];
    for (let k = r + 1; k < n; k++) acc -= A[r][k] * out[k];
    out[r] = acc / A[r][r];
  }
  return out;
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
