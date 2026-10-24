// Step 2 of perception: lock the runner's own lane, then keep following those same two lines,
// straight or curved.
//
// Every lane looks identical, so a single frame cannot tell lane 3 from lane 4. The lane's
// identity comes from continuity: each frame only accepts peaks near where the locked lines are
// predicted to be, so a neighbor lane's line is never picked up, and a runner who crosses a line
// is reported as outside the lane instead of being silently re-locked onto the next one.
//
// Finding the lines (sliding window, as in driving lane detection): start in the segment nearest the
// runner with the line just left and just right of the runner, then search each farther segment
// only where that line is heading. A line must continue over at least two segments, so short
// white things (a chair leg, a shoe, a reflection) never count as a lane line.
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
  laneWidth: LANE_WIDTH_M, // starting width before a lock measures the real one
  // Accepted distance between the two lines at lock time. The default fits a real track
  // (1.22 m); taped practice lanes vary, and vision only measures width roughly anyway.
  minWidth: 0.9,
  maxWidth: 1.55,
  gate: 0.25, // m: max distance between a predicted line and the peak it is matched to
  gateGrowthPerFrame: 0.01, // m: the gate widens while coasting, capped below half a lane
  segmentGate: 0.12, // m: from one segment to the next, the line may deviate this much from its course
  alpha: 0.5, // alpha-beta filter on the lane offset a (the Kalman filter smooths further)
  beta: 0.1,
  shapeAlpha: 0.4, // smoothing of slope b and curve c
  maxCurvature: 0.1, // 1/m (10 m radius): anything sharper is a bad fit, not a track
  widthAlpha: 0.05, // lane width adapts slowly, only when both lines are seen
  coastFrames: 30, // ~1s at 30fps without a matched line, then 'lost'
  // While coasting the lane offset keeps its last rate, decayed each frame: jogging sway makes that
  // rate noisy, and integrating it undecayed walked the lane away from the real lines.
  coastRateDecay: 0.7,
  // Inside <-> outside only after this many consecutive frames agree, and never on coasting
  // frames alone (a drifting prediction is not evidence that the runner left the lane).
  statusFrames: 5,
  // The detector re-aligns its ground frame to the lane every frame (heading search), so in that
  // frame the lane's slope at the runner stays near 0. Re-expressing the previous slope through the
  // frame-to-frame heading change counted the rotation twice: with jogging heading jitter (±8°/frame)
  // the predicted slope reached ~20°, the predicted line missed the real one by ~40 cm at 1.5 m,
  // and the lane coasted to LOST. The previous slope is carried over; while coasting (no line
  // matched last frame) it decays by slopeDecay per frame toward the aligned frame's 0.
  slopeDecay: 0.5,
  // --- Hybrid rules (each one off with 0/false; all off = the tracker above exactly) ---
  // Lock / re-lock takes the pair around the runner: the nearest line that continues on each side
  // (left < 0 < right), about one lane width apart, instead of the longest/straightest pair, which
  // could be a neighbor lane (e.g. jogging 120 s: peaks at -63/+49 cm but the lane to the right won).
  lockStraddle: true,
  // For this many frames after a lock the output is marked lowConfidence (and inside/outside is not
  // switched), so the belt does not give strong cues from a lane it has only just found (0.5 s).
  settleFrames: 15,
  // Both lines matched but their spacing is off the lane width by more than this fraction: the line
  // whose implied centerline is farther from the prediction is dropped; the other + width is used.
  widthCheck: 0.15,
  // A centerline jump larger than this (m) from the prediction is held as a coasting frame until it
  // has persisted jumpFrames frames: a runner does not teleport, a neighbor/diagonal line does.
  jumpGuard: 0.3,
  jumpFrames: 6,
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
    this.polylines = { left: [], right: [] };
    this.last = { status: LaneStatus.UNLOCKED };
    this.shownStatus = LaneStatus.INSIDE; // debounced inside/outside status
    this.pendingStatus = null;
    this.pendingFrames = 0;
    this.settle = 0; // frames left in the low-confidence period after a lock
    this.jumpCount = 0;
  }

  // The lane a Lock would take right now, or null: in the nearest segment that shows a line on each
  // side of the runner about one lane width apart, follow both lines outward (sliding window).
  // Used to preview the lane before locking and by lock() itself.
  candidate(segments) {
    const { segmentGate } = this.cfg;
    const minChain = segments.length > 1 ? 2 : 1;
    if (this.cfg.lockStraddle) return this.straddleCandidate(segments, minChain);
    for (const seg of segments) {
      // Per side, the nearest peak that continues as a line. The second nearest is only considered
      // when the nearest does not continue (a white thing between runner and line); otherwise a
      // narrow lane could be locked as two lanes at once.
      const sideOptions = (peaks) => {
        const out = [];
        for (const p of byDistance(peaks).slice(0, 2)) {
          const chain = followLine(segments, p.y, 0, 0, segmentGate, segmentGate);
          if (chain.length < minChain) continue;
          out.push({ p, chain });
          // A clean, straight line (3+ segments, no hop): no need to look past it.
          if (chain.length >= Math.min(3, segments.length) && chainRms(chain) < 0.02) break;
        }
        return out;
      };
      const lefts = sideOptions(seg.peaks.filter((p) => p.y < 0));
      const rights = sideOptions(seg.peaks.filter((p) => p.y >= 0));
      let best = null;
      for (const { p: left, chain: leftChain } of lefts) {
        for (const { p: right, chain: rightChain } of rights) {
          const width = right.y - left.y;
          if (!this.widthOk(width)) continue;
          // Longer and straighter wins: a chain that started on junk and hopped onto the real line
          // bends where it hopped.
          const score = leftChain.length + rightChain.length - (chainRms(leftChain) + chainRms(rightChain)) / 0.02;
          if (!best || score > best.score) best = { score, width, leftChain, rightChain };
        }
      }
      if (!best) continue;
      const { width, leftChain, rightChain } = best;
      const obs = [
        ...leftChain.map((p) => ({ x: p.x, y: p.y + width / 2 })),
        ...rightChain.map((p) => ({ x: p.x, y: p.y - width / 2 })),
      ];
      const fit = fitCenterline(obs, 0);
      return { center: fit.a, slope: fit.hasB ? fit.b : 0, curve: fit.c, width, polylines: { left: leftChain, right: rightChain } };
    }
    return null;
  }

  // The pair around the runner: in the nearest segment with a usable line on each side, the line
  // nearest to the runner on the left and on the right that continue over minChain segments, if
  // their spacing is a lane width.
  straddleCandidate(segments, minChain) {
    const { segmentGate } = this.cfg;
    // Nearest line that continues; a chain that starts on a short white thing and hops onto a line
    // (it bends where it hops) yields to the next nearest one if that one is straight.
    const nearestLine = (peaks) => {
      const options = [];
      for (const p of [...peaks].sort((a, b) => Math.abs(a.y) - Math.abs(b.y))) {
        const chain = followLine(segments, p.y, 0, 0, segmentGate, segmentGate);
        if (chain.length < minChain) continue;
        options.push({ p, chain, rms: chainRms(chain) });
        if (options.length === 2) break;
      }
      if (!options.length) return null;
      if (options.length === 2 && options[0].rms > 0.02 && options[1].rms < options[0].rms) return options[1];
      return options[0];
    };
    for (const seg of segments) {
      const left = nearestLine(seg.peaks.filter((p) => p.y < 0));
      const right = nearestLine(seg.peaks.filter((p) => p.y >= 0));
      if (!left || !right) continue;
      const width = right.p.y - left.p.y;
      if (!this.widthOk(width)) continue;
      const obs = [
        ...left.chain.map((p) => ({ x: p.x, y: p.y + width / 2 })),
        ...right.chain.map((p) => ({ x: p.x, y: p.y - width / 2 })),
      ];
      const fit = fitCenterline(obs, 0);
      return { center: fit.a, slope: fit.hasB ? fit.b : 0, curve: fit.c, width,
               polylines: { left: left.chain, right: right.chain } };
    }
    return null;
  }

  // Locks onto the line just left and just right of the runner. With segments, both lines must
  // continue outward (see candidate()). Returns true on success.
  lock(peaks, heading, segments = null) {
    if (segments && segments.length) {
      const cand = this.candidate(segments);
      if (!cand) return false;
      this.reset();
      this.locked = true;
      this.center = cand.center;
      this.slope = cand.slope;
      this.curve = cand.curve;
      this.width = cand.width;
      this.heading = heading;
      this.polylines = cand.polylines;
      this.settle = this.cfg.settleFrames;
      this.last = this.output({ left: true, right: true }, 1);
      return true;
    }
    const left = nearest(peaks.filter((p) => p.y < 0), 0);
    const right = nearest(peaks.filter((p) => p.y >= 0), 0);
    if (!left || !right) return false;
    const width = right.y - left.y;
    if (!this.widthOk(width)) return false;
    this.reset();
    this.locked = true;
    this.center = (left.y + right.y) / 2;
    this.width = width;
    this.heading = heading;
    this.last = this.output({ left: true, right: true }, 1);
    return true;
  }

  widthOk(width) {
    return width >= this.cfg.minWidth && width <= this.cfg.maxWidth;
  }

  // peaks: detector peaks over the whole band (used when segments are not given: straight model).
  // segments: detector segments [{ x, peaks }] for the curved model.
  update(peaks, heading, segments = null) {
    if (!this.locked) return this.last;
    if (this.last.status === LaneStatus.LOST) return this.last;

    const { gate, gateGrowthPerFrame, segmentGate, alpha, beta, shapeAlpha, maxCurvature, widthAlpha, coastFrames } = this.cfg;
    // Previous slope, in the detector's lane-aligned frame (see slopeDecay).
    const slope = this.missed > 0 ? this.slope * this.cfg.slopeDecay : this.slope;
    const predicted = this.center + this.centerRate;
    const half = this.width / 2;
    const g = Math.min(gate + this.missed * gateGrowthPerFrame, 0.45 * this.width);

    // Sliding window per line, starting where last frame's lane puts it; legacy callers pass one
    // band of peaks taken as a single segment at x = 0.
    const segs = segments ?? [{ x: 0, peaks }];
    const minChain = segs.length > 1 ? 2 : 1;
    const follow = (startY) => {
      const chain = followLine(segs, startY, slope, this.curve, g, segmentGate);
      return chain.length >= minChain ? chain : [];
    };
    let leftChain = follow(predicted - half);
    let rightChain = follow(predicted + half);
    if (this.settle > 0) this.settle--;
    // Spacing check: drop the side that disagrees with the prediction when the two lines are not a
    // lane width apart (one of them is a neighbor line, a diagonal or a shadow edge).
    if (this.cfg.widthCheck > 0 && leftChain.length && rightChain.length) {
      const meanY = (c) => c.reduce((acc, p) => acc + p.y, 0) / c.length;
      const spacing = meanY(rightChain) - meanY(leftChain);
      if (Math.abs(spacing - this.width) > this.cfg.widthCheck * this.width) {
        const offL = Math.abs(meanY(leftChain) + half - predicted);
        const offR = Math.abs(meanY(rightChain) - half - predicted);
        if (offL > offR) leftChain = [];
        else rightChain = [];
      }
    }
    const matched = { left: leftChain.length > 0, right: rightChain.length > 0 };
    const obs = [
      ...leftChain.map((p) => ({ x: p.x, y: p.y + half })),
      ...rightChain.map((p) => ({ x: p.x, y: p.y - half })),
    ];
    const widths = [];
    for (const l of leftChain) {
      const r = rightChain.find((p) => p.x === l.x);
      if (r && r.y - l.y > 0.5 * this.width) widths.push(r.y - l.y);
    }
    this.polylines = { left: leftChain, right: rightChain };

    if (obs.length === 0) {
      this.centerRate *= this.cfg.coastRateDecay;
      this.center = this.center + this.centerRate;
      this.slope = slope;
      this.heading = heading;
      this.missed++;
      if (this.missed > coastFrames) {
        this.last = { ...this.output({ left: false, right: false }, 0), status: LaneStatus.LOST };
        return this.last;
      }
      this.last = this.debounced(this.output(matched, Math.max(0, 0.5 * (1 - this.missed / coastFrames))), false);
      return this.last;
    }

    const fit = fitCenterline(obs, this.curve);
    const residual = fit.a - predicted;
    // Jump guard: a sudden centerline jump is not trusted until it persists.
    if (this.cfg.jumpGuard > 0 && Math.abs(residual) > this.cfg.jumpGuard && ++this.jumpCount < this.cfg.jumpFrames) {
      this.centerRate *= this.cfg.coastRateDecay;
      this.center += this.centerRate;
      this.slope = slope;
      this.heading = heading;
      this.missed++;
      this.last = this.debounced(this.output({ left: false, right: false }, 0.3), false);
      return this.last;
    }
    this.jumpCount = 0;
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
    this.last = this.debounced(this.output(matched, confidence), true);
    return this.last;
  }

  // Applies the inside/outside debounce to an output. observed: at least one line matched this frame.
  debounced(out, observed) {
    const raw = out.status;
    if (raw === this.shownStatus) {
      this.pendingStatus = null;
      this.pendingFrames = 0;
    } else if (observed && this.settle === 0) {
      if (raw === this.pendingStatus) this.pendingFrames++;
      else { this.pendingStatus = raw; this.pendingFrames = 1; }
      if (this.pendingFrames >= this.cfg.statusFrames) {
        this.shownStatus = raw;
        this.pendingStatus = null;
        this.pendingFrames = 0;
      }
    }
    return { ...out, status: this.shownStatus, rawStatus: raw };
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
      polylines: this.polylines, // matched line points per segment, aligned frame
      matched: { left: !!matched.left, right: !!matched.right },
      confidence: this.settle > 0 ? Math.min(confidence, 0.5) : confidence,
      lowConfidence: this.settle > 0,
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

// Follows one line through the segments from near to far. The first match is searched at startY
// (+ the lane's course) within firstGate; each later one where the line seen so far is heading,
// within segmentGate. One missing segment is tolerated; two end the line.
function followLine(segs, startY, slope, curve, firstGate, segmentGate) {
  const chain = [];
  let lastX = 0;
  let lastY = startY;
  let dir = slope;
  let misses = 0;
  for (const seg of segs) {
    const expect = lastY + dir * (seg.x - lastX) + curve * (seg.x * seg.x - lastX * lastX);
    const p = nearestWithin(seg.peaks, expect, chain.length ? segmentGate : firstGate);
    if (!p) {
      if (chain.length && ++misses > 1) break;
      continue;
    }
    if (chain.length) dir = (p.y - lastY - curve * (seg.x * seg.x - lastX * lastX)) / (seg.x - lastX);
    chain.push({ x: seg.x, y: p.y });
    lastX = seg.x;
    lastY = p.y;
  }
  return chain;
}

// RMS distance of a chain's points from their own best-fit line (0 for 2 points or fewer).
function chainRms(chain) {
  if (chain.length < 3) return 0;
  const [a, b] = solveLeastSquares(chain, (x) => [1, x]);
  return Math.sqrt(chain.reduce((acc, p) => acc + (p.y - a - b * p.x) ** 2, 0) / chain.length);
}

const byDistance = (peaks) => [...peaks].sort((p, q) => Math.abs(p.y) - Math.abs(q.y));

function nearest(peaks, y) {
  let best = null;
  for (const p of peaks) if (!best || Math.abs(p.y - y) < Math.abs(best.y - y)) best = p;
  return best;
}

function nearestWithin(peaks, y, gate) {
  const p = nearest(peaks, y);
  return p && Math.abs(p.y - y) <= gate ? p : null;
}
