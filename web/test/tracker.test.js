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

// ---- Sliding window over forward segments (what the detector gives the tracker) ----

// Segments 1-2 / 2-3 / 3-4 / 4-5.5 m with the lane lines of a track (spacing `width`) seen from a
// runner at runnerY; `extra` adds junk peaks as { x, y } (only in the segment containing x).
function segmentsFor(runnerY, { width = 1.22, slope = 0, extra = [] } = {}) {
  return [1.4, 2.4, 3.4, 4.6].map((x) => {
    const peaks = [];
    for (let k = -3; k <= 3; k++) {
      const y = k * width - width / 2 - runnerY + slope * x;
      if (Math.abs(y) <= 2.2) peaks.push({ y, strength: 50 });
    }
    for (const e of extra) if (Math.abs(e.x - x) < 0.5) peaks.push({ y: e.y, strength: 80 });
    return { x, peaks: peaks.sort((a, b) => a.y - b.y) };
  });
}

test('lock with segments takes the lines just left and right of the runner, not neighbor lanes', () => {
  const t = new LaneTracker();
  assert.ok(t.lock([], 0, segmentsFor(0.2)));
  close(t.last.eY, 0.2, 1e-9);
  close(t.last.width, 1.22, 1e-9);
  assert.equal(t.last.polylines.left.length, 4);
  assert.equal(t.last.polylines.right.length, 4);
});

test('a short white thing near the runner (chair leg) is not a lane line', () => {
  const t = new LaneTracker();
  // A leg 50 cm right of the runner, seen only in the nearest segment: it would pass the width
  // check with the left line, but it does not continue, so the real lines are locked instead.
  assert.ok(t.lock([], 0, segmentsFor(0, { extra: [{ x: 1.4, y: 0.5 }] })));
  close(t.last.rightY, 0.61, 1e-9);
  // While locked, the same leg inside the lane does not move it.
  const out = t.update([], 0, segmentsFor(0, { extra: [{ x: 1.4, y: 0.45 }, { x: 2.4, y: -0.4 }] }));
  close(out.eY, 0, 0.01);
  assert.deepEqual(out.matched, { left: true, right: true });
});

test('no lane: nothing to lock when the lines do not continue over two segments', () => {
  const t = new LaneTracker();
  const scattered = [1.4, 2.4, 3.4, 4.6].map((x, i) => ({ x, peaks: [{ y: -0.6 + 0.4 * i, strength: 30 }, { y: 0.7 - 0.5 * i, strength: 30 }] }));
  assert.equal(t.candidate(scattered), null);
  assert.equal(t.lock([], 0, scattered), false);
});

test('lane width is a range: default fits a real track, a 0.5-1.0 m range fits taped practice lanes', () => {
  assert.equal(new LaneTracker().lock([], 0, segmentsFor(0, { width: 0.73 })), false);
  assert.ok(new LaneTracker().lock([], 0, segmentsFor(0, { width: 1.22 })));
  const practice = { minWidth: 0.5, maxWidth: 1.0 };
  for (const width of [0.55, 0.73, 0.95]) {
    const t = new LaneTracker(practice);
    assert.ok(t.lock([], 0, segmentsFor(0.1, { width })), `width ${width}`);
    close(t.last.width, width, 1e-9);
    close(t.last.eY, 0.1, 1e-9);
  }
  assert.equal(new LaneTracker(practice).lock([], 0, segmentsFor(0, { width: 1.22 })), false);
});

test('sliding window follows a lane that bends away and keeps the lines when one segment is missing', () => {
  const t = new LaneTracker();
  assert.ok(t.lock([], 0, segmentsFor(0)));
  let out;
  for (let i = 0; i < 10; i++) {
    const segs = segmentsFor(0, { slope: 0.08 }); // lines drift 8 cm per meter to the right
    segs[2].peaks = []; // e.g. a shadow over 3-4 m
    out = t.update([], 0, segs);
  }
  assert.equal(out.polylines.right.length, 3);
  close(out.eY, 0, 0.02);
  close(out.ePsi, -Math.atan(0.08), 0.02); // lane points right of the runner
});

test('jogging heading jitter does not bend the predicted lane off the lines (slope is not re-rotated)', () => {
  // The detector re-aligns to the lane every frame, so the peaks stay straight in its frame while
  // the reported heading jitters by ±8 degrees from frame to frame, as measured while jogging.
  const t = new LaneTracker();
  assert.ok(t.lock([], 0, segmentsFor(0)));
  let out;
  for (let i = 0; i < 30; i++) {
    const heading = (i % 2 ? 1 : -1) * 0.14;
    out = t.update([], heading, segmentsFor(0));
    assert.ok(out.matched.left && out.matched.right, `frame ${i}: both lines matched`);
  }
  close(out.eY, 0, 0.02);
  assert.equal(out.status, LaneStatus.INSIDE);
});

test('inside/outside changes only after statusFrames observed frames, not on coasting frames', () => {
  const t = new LaneTracker();
  assert.ok(t.lock([], 0, segmentsFor(0)));
  // Runner steps 0.8 m to the right: right line now left of the runner.
  const crossed = segmentsFor(0.8);
  for (let i = 0; i < t.cfg.statusFrames - 1; i++) assert.equal(t.update([], 0, crossed).status, LaneStatus.INSIDE);
  // Frames with no lines at all never flip the status, however the prediction drifts.
  const empty = crossed.map((s) => ({ ...s, peaks: [] }));
  for (let i = 0; i < 10; i++) assert.equal(t.update([], 0, empty).status, LaneStatus.INSIDE);
});

// Detector segments of the jogging clip at 119.99 s (h 1.5 m): the runner's lane lines at
// -0.73 / +0.57 m, the next lane's right line at +1.75..1.81 m and a line two lanes left.
const JOG_119_99 = [
  { x: 1.4, peaks: [{ y: -0.73, strength: 50 }, { y: 0.57, strength: 50 }] },
  { x: 2.4, peaks: [{ y: -0.72, strength: 50 }, { y: 0.55, strength: 50 }] },
  { x: 3.5, peaks: [{ y: -0.72, strength: 50 }, { y: 0.53, strength: 50 }, { y: 1.81, strength: 50 }] },
  { x: 4.6, peaks: [{ y: -1.99, strength: 50 }, { y: -0.75, strength: 50 }, { y: 0.47, strength: 50 }, { y: 1.75, strength: 50 }] },
];

test('re-lock on the jogging 119.99 s frame takes the pair around the runner, not the lane to the right', () => {
  const t = new LaneTracker({ minWidth: 0.6, maxWidth: 1.3 }); // the page's lane width range
  assert.ok(t.lock([], 0, JOG_119_99));
  // Lines extrapolated to the runner (x = 0): own lane, never the neighbor line at +1.75 m.
  close(t.last.leftY, -0.75, 0.08);
  close(t.last.rightY, 0.55, 0.08);
  assert.equal(t.last.status, LaneStatus.INSIDE);
});

test('right after a lock the output is low-confidence for settleFrames, then normal', () => {
  const t = new LaneTracker();
  assert.ok(t.lock([], 0, segmentsFor(0)));
  assert.equal(t.last.lowConfidence, true);
  let out;
  for (let i = 0; i < t.cfg.settleFrames; i++) out = t.update([], 0, segmentsFor(0));
  assert.equal(out.lowConfidence, false);
  assert.equal(out.confidence, 1);
});

test('two lines that are not a lane width apart: the one off the prediction is dropped', () => {
  const t = new LaneTracker();
  assert.ok(t.lock([], 0, segmentsFor(0)));
  // Right line hidden, a shadow edge 20 cm inside the lane instead (within the first gate):
  // spacing 1.02 m, off the lane width by 16%.
  const segs = segmentsFor(0).map((s) => ({ ...s, peaks: s.peaks.map((p) => (Math.abs(p.y - 0.61) < 0.05 ? { ...p, y: 0.41 } : p)) }));
  const out = t.update([], 0, segs);
  assert.deepEqual(out.matched, { left: true, right: false });
  close(out.eY, 0, 0.02);
});

test('after coasting, a sudden centerline jump is held until it persists (jumpFrames)', () => {
  const t = new LaneTracker();
  assert.ok(t.lock([], 0, segmentsFor(0)));
  const empty = segmentsFor(0).map((s) => ({ ...s, peaks: [] }));
  for (let i = 0; i < 15; i++) t.update([], 0, empty); // coasting widens the gate to 0.40 m
  // Then lines 0.34 m off (a neighbor/diagonal line inside the widened gate): not trusted at once.
  const out = t.update([], 0, segmentsFor(0.34));
  close(out.eY, 0, 0.05);
});

test('a crossing diagonal (staggered start line) is not taken as the lane line', () => {
  const t = new LaneTracker();
  assert.ok(t.lock([], 0, segmentsFor(0)));
  // Ground points: both lane lines straight at ±0.61, except the right line is not visible around
  // x = 3.4 where a start line crosses at a 27° angle; its peak sits 9 cm inside the right line.
  const pts = [];
  for (let x = 1.0; x <= 5.5; x += 0.05) {
    pts.push(x, -0.61);
    if (x < 2.9 || x > 3.9) pts.push(x, 0.61);
  }
  for (let x = 3.0; x <= 3.8; x += 0.02) pts.push(x, 0.52 + 0.5 * (x - 3.4));
  const segs = segmentsFor(0).map((s) => (s.x === 3.4 ? { ...s, peaks: s.peaks.map((p) => (Math.abs(p.y - 0.61) < 0.05 ? { ...p, y: 0.52 } : p)) } : s));
  const out = t.update([], 0, segs, { pts: Float32Array.from(pts), count: pts.length / 2 });
  assert.ok(!out.polylines.right.some((p) => p.x === 3.4), 'diagonal point dropped');
  assert.ok(out.polylines.right.length >= 2, 'right line still matched');
  close(out.eY, 0, 0.02);
});
