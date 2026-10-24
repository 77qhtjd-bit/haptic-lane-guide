import test from 'node:test';
import assert from 'node:assert/strict';
import {
  intrinsicsFromFov,
  upFromOrientation,
  deviceToCamera,
  AttitudeBuffer,
  groundBasis,
  projectToGround,
  groundToPixel,
  rotatePoints,
  findHeading,
  lateralHistogram,
  findPeaks,
  estimateHeight,
} from '../lane/ground.js';

const K = intrinsicsFromFov(120, 160, 53);
const close = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} not within ${tol} of ${b}`);
const deg = (d) => (d * Math.PI) / 180;

// Renders painted lines (5cm wide) into quantized pixel centers, like a real low-res mask would.
// lineYs are perpendicular offsets of the lines; laneHeading rotates the whole lane (body yaw).
function renderLines(lineYs, basis, h, laneHeading = 0) {
  const c = Math.cos(laneHeading);
  const s = Math.sin(laneHeading);
  const seen = new Set();
  const pixels = [];
  for (const yc of lineYs) {
    for (let t = 0.3; t <= 7; t += 0.004) {
      for (let dy = -0.025; dy <= 0.025; dy += 0.005) {
        const x = t * c - (yc + dy) * s;
        const y = t * s + (yc + dy) * c;
        const p = groundToPixel(x, y, K, basis, h);
        if (!p || p[0] < 0 || p[0] >= K.width || p[1] < 0 || p[1] >= K.height) continue;
        const key = `${Math.floor(p[0])},${Math.floor(p[1])}`;
        if (seen.has(key)) continue;
        seen.add(key);
        pixels.push(Math.floor(p[0]) + 0.5, Math.floor(p[1]) + 0.5);
      }
    }
  }
  return { pixels: Float32Array.from(pixels), count: pixels.length / 2 };
}

function groundPoints(lineYs, { beta = 65, gamma = 4, hTrue = 1.3, hAssumed = 1.3, laneHeading = 0 } = {}) {
  const basis = groundBasis(deviceToCamera(upFromOrientation(beta, gamma)));
  const { pixels, count } = renderLines(lineYs, basis, hTrue, laneHeading);
  return projectToGround(pixels, count, K, basis, hAssumed);
}

// Same pipeline as GroundLineDetector.process, minus OpenCV.
function detectWithHeading(lineYs, opts = {}) {
  const ground = groundPoints(lineYs, opts);
  const heading = findHeading(ground.points, ground.count);
  const aligned = rotatePoints(ground.points, ground.count, heading);
  return { heading, peaks: findPeaks(lateralHistogram(aligned, ground.count)) };
}

const detect = (lineYs, opts) => detectWithHeading(lineYs, opts).peaks;

test('orientation gives up-vectors for flat and upright phones', () => {
  assert.deepEqual(upFromOrientation(0, 0).map((v) => +v.toFixed(6)), [0, 0, 1]);
  assert.deepEqual(upFromOrientation(90, 0).map((v) => +v.toFixed(6)), [0, 1, 0]);
  // Upright portrait phone: up points toward the top of the image (-y in camera coords).
  close(deviceToCamera(upFromOrientation(90, 0))[1], -1, 1e-9);
});

test('camera conventions: ahead is below center when tilted down, right is right', () => {
  const basis = groundBasis(deviceToCamera(upFromOrientation(65, 0)));
  const ahead = groundToPixel(2, 0, K, basis, 1.3);
  close(ahead[0], K.cx, 1e-6);
  assert.ok(ahead[1] > K.cy);
  assert.ok(groundToPixel(2, 0.5, K, basis, 1.3)[0] > K.cx);
});

test('projection round-trips through groundToPixel', () => {
  const basis = groundBasis(deviceToCamera(upFromOrientation(70, -6)));
  const p = groundToPixel(2.7, -0.4, K, basis, 1.25);
  const g = projectToGround(Float32Array.from(p), 1, K, basis, 1.25);
  close(g.points[0], 2.7, 1e-4);
  close(g.points[1], -0.4, 1e-4);
});

test('camera pointing straight down has no forward direction', () => {
  assert.equal(groundBasis([0, 0, -1]), null);
});

// A portrait 1x camera (~53 deg) on the chest sees about +-0.7m at 1m and +-2m at 4m ahead, so the
// two lines of the runner's own lane are always in view; lines one lane further out only appear far
// away. Those must be accurate if reported, but they are not required.
function assertLines(peaks, required, optional, tol) {
  for (const y of required) {
    assert.ok(peaks.some((p) => Math.abs(p.y - y) <= tol), `missing line at ${y}: ${JSON.stringify(peaks)}`);
  }
  for (const p of peaks) {
    assert.ok([...required, ...optional].some((y) => Math.abs(p.y - y) <= tol), `spurious peak ${p.y}`);
  }
}

test("finds the runner's two lane lines within 2cm, runner centered", () => {
  assertLines(detect([-1.83, -0.61, 0.61, 1.83]), [-0.61, 0.61], [-1.83, 1.83], 0.02);
});

test('lane lines stay put when the body pitches and rolls (running shake)', () => {
  for (const [beta, gamma] of [[55, -8], [65, 0], [75, 8]]) {
    assertLines(detect([-2.17, -0.95, 0.27, 1.49], { beta, gamma }), [-0.95, 0.27], [-2.17, 1.49], 0.03);
  }
});

test('without heading search, a 10 degree body yaw smears the lines (why the search exists)', () => {
  const sharpest = (laneHeading) => {
    const g = groundPoints([-0.61, 0.61], { laneHeading });
    return Math.max(...lateralHistogram(g.points, g.count).hist);
  };
  assert.ok(sharpest(deg(10)) < sharpest(0) / 2);
});

test('heading search recovers body yaw within 1 degree and line offsets within 3cm', () => {
  for (const yawDeg of [-12, -10, -4, 0, 6, 12]) {
    for (const [beta, gamma] of [[58, -6], [70, 5]]) {
      const label = `yaw=${yawDeg} beta=${beta} gamma=${gamma}`;
      const { heading, peaks } = detectWithHeading([-1.83, -0.61, 0.61, 1.83], {
        beta,
        gamma,
        laneHeading: deg(yawDeg),
      });
      assert.ok(Math.abs(heading - deg(yawDeg)) <= deg(1), `${label}: heading ${(heading * 180) / Math.PI}`);
      assertLines(peaks, [-0.61, 0.61], [-1.83, 1.83], 0.03);
    }
  }
});

// At ~18 degrees of yaw the 53 degree portrait view loses the own-lane line on the side the body
// turns away from, so only the other line is required (centerline = that line +/- half a lane).
test('at large yaw, heading and the line still in view are recovered', () => {
  for (const [yawDeg, inView] of [[-18, 0.61], [18, -0.61]]) {
    const { heading, peaks } = detectWithHeading([-1.83, -0.61, 0.61, 1.83], { laneHeading: deg(yawDeg) });
    assert.ok(Math.abs(heading - deg(yawDeg)) <= deg(1), `yaw=${yawDeg}: heading ${(heading * 180) / Math.PI}`);
    assertLines(peaks, [inView], [-1.83, -0.61, 0.61, 1.83], 0.03);
  }
});

test('heading search follows a prior outside its own range', () => {
  const ground = groundPoints([-0.61, 0.61], { laneHeading: deg(32) });
  const heading = findHeading(ground.points, ground.count, undefined, { center: deg(28) });
  close(heading, deg(32), deg(1));
});

test('lane width re-estimates a wrong camera height', () => {
  const peaks = detect([-1.83, -0.61, 0.61, 1.83], { hTrue: 1.3, hAssumed: 1.1 });
  close(estimateHeight(peaks, 1.1), 1.3, 0.03);
});

test('attitude buffer interpolates between samples', () => {
  const buf = new AttitudeBuffer();
  buf.push(0, [0, -1, 0]);
  buf.push(10, [0, 0, -1]);
  const mid = buf.at(5);
  close(mid[1], -Math.SQRT1_2, 1e-9);
  close(mid[2], -Math.SQRT1_2, 1e-9);
  assert.deepEqual(buf.at(-5), [0, -1, 0]);
});
