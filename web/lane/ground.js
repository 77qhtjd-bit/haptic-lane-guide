// Ground-plane geometry: turns camera pixels into meters on the track using the phone's attitude.
//
// Frames
//   device (W3C):  x right, y toward the top of the screen, z out of the screen
//   camera (rear camera, portrait, OpenCV style): x right, y down, z = optical axis
//                  => camera = (dx, -dy, -dz)
//   ground:        x forward (optical axis projected on the ground), y right; meters from
//                  the point on the ground directly below the camera

export const LANE_WIDTH_M = 1.22;

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const normalize = (a) => scale(a, 1 / norm(a));

// Pinhole intrinsics from the horizontal field of view of the image as delivered (width side).
export function intrinsicsFromFov(width, height, hfovDeg) {
  const fx = width / 2 / Math.tan((hfovDeg * Math.PI) / 360);
  return { fx, fy: fx, cx: width / 2, cy: height / 2, width, height };
}

// "Up" in device coords from a W3C deviceorientation event (Z-X'-Y'' Euler angles).
// Only beta and gamma matter, and the result stays continuous near beta = 90 (phone upright),
// where gamma on its own becomes unstable.
export function upFromOrientation(betaDeg, gammaDeg) {
  const b = (betaDeg * Math.PI) / 180;
  const g = (gammaDeg * Math.PI) / 180;
  return [-Math.cos(b) * Math.sin(g), Math.sin(b), Math.cos(b) * Math.cos(g)];
}

export function deviceToCamera(v) {
  return [v[0], -v[1], -v[2]];
}

// Ring buffer of timestamped up-vectors (camera coords), so each frame uses the attitude at the
// moment it was captured instead of the latest sample.
export class AttitudeBuffer {
  constructor(size = 64) {
    this.size = size;
    this.samples = [];
  }

  push(tMs, upCam) {
    this.samples.push({ t: tMs, up: normalize(upCam) });
    if (this.samples.length > this.size) this.samples.shift();
  }

  at(tMs) {
    const s = this.samples;
    if (s.length === 0) return null;
    if (tMs <= s[0].t) return s[0].up;
    if (tMs >= s[s.length - 1].t) return s[s.length - 1].up;
    let i = s.length - 1;
    while (s[i - 1].t > tMs) i--;
    const a = s[i - 1];
    const b = s[i];
    const w = (tMs - a.t) / (b.t - a.t);
    return normalize([
      a.up[0] + (b.up[0] - a.up[0]) * w,
      a.up[1] + (b.up[1] - a.up[1]) * w,
      a.up[2] + (b.up[2] - a.up[2]) * w,
    ]);
  }
}

// Ground basis (up, forward, right) expressed in camera coords.
// Returns null when the optical axis is within ~6 degrees of vertical (forward is undefined).
export function groundBasis(upCam) {
  const u = normalize(upCam);
  const f0 = sub([0, 0, 1], scale(u, u[2]));
  const n = norm(f0);
  if (n < 0.1) return null;
  const f = scale(f0, 1 / n);
  return { u, f, r: cross(f, u) };
}

// pixels: flat [u0, v0, u1, v1, ...] in the image K describes. h: camera height above ground (m).
// Returns ground points as flat [x0, y0, ...]; pixels above the horizon or beyond maxRange are dropped.
export function projectToGround(pixels, count, K, basis, h, maxRange = 8) {
  const { u, f, r } = basis;
  const out = new Float32Array(count * 2);
  let n = 0;
  for (let i = 0; i < count; i++) {
    const a = (pixels[2 * i] - K.cx) / K.fx;
    const b = (pixels[2 * i + 1] - K.cy) / K.fy;
    const du = a * u[0] + b * u[1] + u[2];
    if (du > -1e-3) continue;
    const s = h / -du;
    const x = s * (a * f[0] + b * f[1] + f[2]);
    if (x <= 0 || x > maxRange) continue;
    out[2 * n] = x;
    out[2 * n + 1] = s * (a * r[0] + b * r[1] + r[2]);
    n++;
  }
  return { points: out, count: n };
}

// Inverse of projectToGround, used by tests and for drawing the locked lane back onto the video.
export function groundToPixel(x, y, K, basis, h) {
  const { u, f, r } = basis;
  const p = [
    x * f[0] + y * r[0] - h * u[0],
    x * f[1] + y * r[1] - h * u[1],
    x * f[2] + y * r[2] - h * u[2],
  ];
  if (p[2] <= 1e-6) return null;
  return [K.fx * (p[0] / p[2]) + K.cx, K.fy * (p[1] / p[2]) + K.cy];
}

// Rotates ground points by -theta about the camera foot point, so lines heading at angle theta
// (radians, positive = lane turns to the right of the camera's forward) become parallel to x.
// After rotation, y is the perpendicular distance to each line.
export function rotatePoints(points, count, theta) {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  const out = new Float32Array(count * 2);
  for (let i = 0; i < count; i++) {
    const x = points[2 * i];
    const y = points[2 * i + 1];
    out[2 * i] = x * c + y * s;
    out[2 * i + 1] = -x * s + y * c;
  }
  return out;
}

// Sum of squared bin counts: highest when every line's points fall into as few bins as possible.
function sharpness(H) {
  let acc = 0;
  for (const v of H.hist) acc += v * v;
  return acc;
}

// Lane heading in the ground frame: the rotation that makes the lateral histogram sharpest.
// When the body yaws against the lane, lines are diagonal in the ground frame and their points
// smear across many bins; rotating them back makes the peaks sharp again.
// center: search around a prior heading (previous frame or gyro prediction), in radians.
export function findHeading(points, count, band, { center = 0, rangeDeg = 25, stepDeg = 1.5 } = {}) {
  const step = (stepDeg * Math.PI) / 180;
  const n = Math.round(rangeDeg / stepDeg);
  const scores = [];
  for (let k = -n; k <= n; k++) {
    const theta = center + k * step;
    scores.push(sharpness(lateralHistogram(rotatePoints(points, count, theta), count, band)));
  }
  let best = n; // ties (e.g. no points at all) keep the prior
  for (let i = 0; i < scores.length; i++) if (scores[i] > scores[best]) best = i;
  let offset = 0;
  if (best > 0 && best < scores.length - 1) {
    const denom = scores[best - 1] - 2 * scores[best] + scores[best + 1];
    if (denom < 0) offset = (0.5 * (scores[best - 1] - scores[best + 1])) / denom;
  }
  return center + (best - n + offset) * step;
}

// Histogram of the lateral position y of ground points inside a forward band.
// Track lines are parallel, so in the ground frame each line becomes one sharp peak.
export function lateralHistogram(points, count, { xMin = 1, xMax = 4, yHalf = 3, bin = 0.02 } = {}) {
  const bins = Math.round((2 * yHalf) / bin);
  const hist = new Float32Array(bins);
  for (let i = 0; i < count; i++) {
    const x = points[2 * i];
    const y = points[2 * i + 1];
    if (x < xMin || x > xMax || y <= -yHalf || y >= yHalf) continue;
    hist[Math.floor((y + yHalf) / bin)] += 1;
  }
  return { hist, bin, yHalf };
}

function gaussianSmooth(hist, sigmaBins) {
  const radius = Math.ceil(3 * sigmaBins);
  const kernel = [];
  let sum = 0;
  for (let k = -radius; k <= radius; k++) {
    const w = Math.exp(-(k * k) / (2 * sigmaBins * sigmaBins));
    kernel.push(w);
    sum += w;
  }
  const out = new Float32Array(hist.length);
  for (let i = 0; i < hist.length; i++) {
    let acc = 0;
    for (let k = -radius; k <= radius; k++) {
      const j = i + k;
      if (j >= 0 && j < hist.length) acc += hist[j] * kernel[k + radius];
    }
    out[i] = acc / sum;
  }
  return out;
}

// Peaks of the lateral histogram: [{ y, strength, width }] sorted by y (left to right).
// minSeparation keeps the two edges of one 5cm line from becoming two peaks.
// maxWidth rejects broad humps: a painted line is a few cm wide, while a white floor, wall base or
// bright patch spreads over tens of cm. minFraction is taken relative to the strongest narrow peak,
// so a big bright area cannot raise the bar and hide the real lines.
export function findPeaks(
  H,
  { sigma = 0.03, minSeparation = 0.6, minFraction = 0.2, minCount = 3, maxWidth = 0.25 } = {},
) {
  const { bin, yHalf } = H;
  const s = gaussianSmooth(H.hist, sigma / bin);
  const floor = minCount / (sigma / bin);
  const narrow = [];
  for (let i = 1; i < s.length - 1; i++) {
    if (s[i] < floor || !(s[i] > s[i - 1] && s[i] >= s[i + 1])) continue;
    const width = halfMaxWidth(s, i) * bin;
    if (width <= maxWidth) narrow.push({ i, width });
  }
  let max = 0;
  for (const c of narrow) max = Math.max(max, s[c.i]);
  const candidates = narrow.filter((c) => s[c.i] >= minFraction * max).sort((a, b) => s[b.i] - s[a.i]);
  const kept = [];
  for (const c of candidates) {
    if (kept.every((k) => Math.abs(c.i - k.i) * bin >= minSeparation)) kept.push(c);
  }
  return kept
    .map(({ i, width }) => {
      const denom = s[i - 1] - 2 * s[i] + s[i + 1];
      const offset = denom < 0 ? (0.5 * (s[i - 1] - s[i + 1])) / denom : 0;
      return { y: (i + 0.5 + offset) * bin - yHalf, strength: s[i], width };
    })
    .sort((a, b) => a.y - b.y);
}

// Full width at half maximum around index i, in bins.
function halfMaxWidth(s, i) {
  const half = s[i] / 2;
  let l = i;
  while (l > 0 && s[l - 1] > half) l--;
  let r = i;
  while (r < s.length - 1 && s[r + 1] > half) r++;
  return r - l + 1;
}

// Ground coordinates scale linearly with h, and the lane width is known, so the spacing of
// adjacent peaks re-estimates the true camera height (running bounce, wrong initial guess).
// Returns null if no adjacent pair looks like one lane.
export function estimateHeight(peaks, hAssumed, laneWidth = LANE_WIDTH_M) {
  const ratios = [];
  for (let i = 1; i < peaks.length; i++) {
    const spacing = peaks[i].y - peaks[i - 1].y;
    if (spacing > 0.6 * laneWidth && spacing < 1.6 * laneWidth) ratios.push(laneWidth / spacing);
  }
  if (ratios.length === 0) return null;
  ratios.sort((a, b) => a - b);
  return hAssumed * ratios[Math.floor(ratios.length / 2)];
}
