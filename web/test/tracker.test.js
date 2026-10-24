import test from 'node:test';
import assert from 'node:assert/strict';
import { LaneTracker, LaneStatus } from '../lane/tracker.js';

const close = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} not within ${tol} of ${b}`);

// A track with lane lines every 1.22m; lane 0 spans -0.61..0.61 in track coordinates.
// Returns the peaks a detector would report for a runner at runnerY (track coords):
// lines within `view` meters of the runner, optional noise and extra junk peaks.
function peaksFor(runnerY, { view = 2, noise = 0, rand = Math.random, extra = [] } = {}) {
  const peaks = [];
  for (let k = -4; k <= 4; k++) {
    const y = k * 1.22 - 0.61 - runnerY;
    if (Math.abs(y) <= view) peaks.push({ y: y + (rand() - 0.5) * 2 * noise, strength: 50 });
  }
  for (const y of extra) peaks.push({ y, strength: 20 });
  return peaks.sort((a, b) => a.y - b.y);
}

function seeded(seed) {
  return () => {
    seed = (seed * 1664525 + 1013904223) % 4294967296;
    return seed / 4294967296;
  };
}

function lockedAt(runnerY = 0) {
  const t = new LaneTracker();
  assert.ok(t.lock(peaksFor(runnerY), 0));
  return t;
}

test('locks the lane around the runner', () => {
  const t = lockedAt(0.1);
  const out = t.last;
  assert.equal(out.status, LaneStatus.INSIDE);
  close(out.eY, 0.1, 1e-9);
  close(out.width, 1.22, 1e-9);
});

test('refuses to lock without a line on each side or with an implausible width', () => {
  const t = new LaneTracker();
  assert.equal(t.lock([{ y: 0.61, strength: 50 }], 0), false);
  assert.equal(t.lock([{ y: -0.35, strength: 50 }, { y: 0.35, strength: 50 }], 0), false);
  assert.equal(t.last.status, LaneStatus.UNLOCKED);
});

test('drifting right across the line reports outside-right instead of re-locking the next lane', () => {
  const t = lockedAt(0);
  let out;
  for (let i = 1; i <= 100; i++) out = t.update(peaksFor(i * 0.01), 0);
  assert.equal(out.status, LaneStatus.OUTSIDE_RIGHT);
  close(out.eY, 1.0, 0.03); // a silent re-lock onto lane +1 would read about -0.22
});

test('drifting left across the line reports outside-left', () => {
  const t = lockedAt(0);
  let out;
  for (let i = 1; i <= 100; i++) out = t.update(peaksFor(-i * 0.01), 0);
  assert.equal(out.status, LaneStatus.OUTSIDE_LEFT);
  close(out.eY, -1.0, 0.03);
});

test('one visible line is enough: center = that line +/- half a lane', () => {
  const t = lockedAt(0);
  for (let i = 1; i <= 20; i++) {
    const runnerY = 0.01 * i;
    const peaks = peaksFor(runnerY).filter((p) => p.y > 0 && p.y < 1); // only the right line in view
    const out = t.update(peaks, 0);
    assert.deepEqual(out.matched, { left: false, right: true });
    assert.equal(out.confidence, 0.6);
  }
  close(t.last.eY, 0.2, 0.02);
});

test('noisy peaks and junk near the runner (shoes, shadows) do not pull the lane', () => {
  const rand = seeded(7);
  const t = lockedAt(0);
  let out;
  for (let i = 0; i < 200; i++) {
    const runnerY = 0.15 * Math.sin(i / 20);
    out = t.update(peaksFor(runnerY, { noise: 0.03, rand, extra: [0.02 * (rand() - 0.5)] }), 0);
    close(out.eY, runnerY, 0.06);
  }
  assert.equal(out.status, LaneStatus.INSIDE);
});

test('coasts through short dropouts and reacquires the same lane', () => {
  const t = lockedAt(0);
  for (let i = 0; i < 10; i++) t.update([], 0);
  assert.equal(t.last.status, LaneStatus.INSIDE);
  assert.ok(t.last.confidence < 0.5);
  const out = t.update(peaksFor(0.05), 0);
  assert.deepEqual(out.matched, { left: true, right: true });
  close(out.eY, 0.05, 0.05);
});

test('a long dropout ends in lost, which only a new lock clears', () => {
  const t = lockedAt(0);
  for (let i = 0; i < 31; i++) t.update([], 0);
  assert.equal(t.last.status, LaneStatus.LOST);
  assert.equal(t.update(peaksFor(0), 0).status, LaneStatus.LOST);
  assert.ok(t.lock(peaksFor(0), 0));
  assert.equal(t.last.status, LaneStatus.INSIDE);
});

test('heading error has the right sign', () => {
  const t = lockedAt(0);
  // Lane turns 5 degrees to the right of the camera => runner points 5 degrees left of the lane.
  const out = t.update(peaksFor(0), (5 * Math.PI) / 180);
  close(out.ePsi, (-5 * Math.PI) / 180, 1e-12);
});
