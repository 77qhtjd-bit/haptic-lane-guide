import test from 'node:test';
import assert from 'node:assert/strict';
import {
  intrinsicsFromFov,
  upFromOrientation,
  deviceToCamera,
  groundBasis,
  projectToGround,
  groundToPixel,
  findHeading,
  rotatePoints,
  lateralHistogram,
  findPeaks,
  segmentPeaks,
} from '../lane/ground.js';
import { LaneTracker, LaneStatus } from '../lane/tracker.js';

const K = intrinsicsFromFov(120, 160, 53);
const DEG = Math.PI / 180;
const close = (a, b, tol, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} ${a} not within ${tol} of ${b}`);

// A track bend: lane lines are concentric arcs. radius > 0 bends right, < 0 left, Infinity straight.
// The runner stands offsetY right of the lane centerline, pointing yawDeg right of the lane tangent.
function renderBend({ radius, offsetY = 0, yawDeg = 0, beta = 65, gamma = 0 }) {
  const basis = groundBasis(deviceToCamera(upFromOrientation(beta, gamma)));
  const yaw = yawDeg * DEG;
  const seen = new Set();
  const pixels = [];
  for (const d of [-1.83, -0.61, 0.61, 1.83]) {
    for (let s = 0; s <= 8; s += 0.004) {
      for (let w = -0.025; w <= 0.025; w += 0.005) {
        // Point on the line at arc length s, in lane coordinates (x along the tangent at the runner).
        let lx;
        let ly;
        if (!Number.isFinite(radius)) {
          lx = s;
          ly = d + w;
        } else {
          const r = radius - (d + w); // positive radius: center to the right
          const phi = s / radius;
          lx = r * Math.sin(phi);
          ly = radius - r * Math.cos(phi);
        }
        // Into the camera's ground frame: shift by the runner offset, rotate by the runner's yaw.
        const yy = ly - offsetY;
        const x = lx * Math.cos(yaw) + yy * Math.sin(yaw);
        const y = -lx * Math.sin(yaw) + yy * Math.cos(yaw);
        const p = groundToPixel(x, y, K, basis, 1.3);
        if (!p || p[0] < 0 || p[0] >= K.width || p[1] < 0 || p[1] >= K.height) continue;
        const key = `${Math.floor(p[0])},${Math.floor(p[1])}`;
        if (seen.has(key)) continue;
        seen.add(key);
        pixels.push(Math.floor(p[0]) + 0.5, Math.floor(p[1]) + 0.5);
      }
    }
  }
  const g = projectToGround(Float32Array.from(pixels), pixels.length / 2, K, basis, 1.3);
  return g;
}

// Same pipeline as GroundLineDetector.process + LaneTracker, minus OpenCV.
function runTracker(scene, { frames = 15, useSegments = true } = {}) {
  const g = renderBend(scene);
  const heading = findHeading(g.points, g.count);
  const aligned = rotatePoints(g.points, g.count, heading);
  const peaks = findPeaks(lateralHistogram(aligned, g.count));
  const segments = segmentPeaks(aligned, g.count);
  const t = new LaneTracker();
  assert.ok(t.lock(peaks, heading), `lock failed: ${JSON.stringify(peaks)}`);
  let out = t.last;
  for (let i = 0; i < frames; i++) out = t.update(peaks, heading, useSegments ? segments : null);
  return out;
}

test('straight lane: curve fitting keeps the straight result (regression)', () => {
  const out = runTracker({ radius: Infinity, offsetY: 0.1 });
  close(out.eY, 0.1, 0.03, 'eY');
  close(out.curvature, 0, 0.01, 'curvature');
  close(out.ePsi, 0, 1.5 * DEG, 'ePsi');
});

test('track bend (R = 36.5 m) right and left: curvature, offset and heading are recovered', () => {
  for (const radius of [36.5, -36.5]) {
    for (const offsetY of [-0.2, 0, 0.2]) {
      const out = runTracker({ radius, offsetY });
      const label = `R=${radius} offset=${offsetY}`;
      assert.equal(out.status, LaneStatus.INSIDE, label);
      close(out.eY, offsetY, 0.04, `${label} eY`);
      close(out.curvature, 1 / radius, 0.012, `${label} curvature`);
      close(out.ePsi, 0, 2 * DEG, `${label} ePsi`);
    }
  }
});

test('on a bend, the runner pointing off the tangent is measured against the tangent', () => {
  const out = runTracker({ radius: 36.5, offsetY: 0, yawDeg: 8 });
  close(out.ePsi, 8 * DEG, 2 * DEG, 'ePsi');
  close(out.eY, 0, 0.05, 'eY');
});

test('on a bend, the straight model is biased; the curve model is not', () => {
  const scene = { radius: 36.5, offsetY: 0 };
  const straight = runTracker(scene, { useSegments: false });
  const curved = runTracker(scene, { useSegments: true });
  // The straight model reports the lane direction averaged over 1-4 m as the tangent at the runner.
  assert.ok(Math.abs(straight.ePsi) > 2 * DEG, `straight ePsi ${straight.ePsi / DEG} deg`);
  assert.ok(Math.abs(curved.ePsi) < Math.abs(straight.ePsi) / 2, `curved ePsi ${curved.ePsi / DEG} deg`);
});
