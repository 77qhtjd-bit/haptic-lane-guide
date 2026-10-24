import test from 'node:test';
import assert from 'node:assert/strict';
import { RunLogger, Condition, CSV_COLUMNS, summarize } from '../control/logger.js';
import { LaneStatus } from '../lane/tracker.js';

const close = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} not within ${tol} of ${b}`);

function memoryStorage() {
  const data = new Map();
  return { getItem: (k) => data.get(k) ?? null, setItem: (k, v) => data.set(k, String(v)) };
}

const row = (status, eY) => ({
  status, confidence: 1, eY_raw_m: eY, ePsi_raw_rad: 0, eY_kf_m: eY, vY_kf_mps: 0, ePsi_kf_rad: 0,
  u_m: eY, zone: 0, motors: 4, period_ms: 1000, L1: 0, L2: 0, L3: 1, L4: 0, L5: 0, imu_age_ms: 12, fps: 30,
});

test('summary RMS ignores frames where the lane was not tracked', () => {
  const s = summarize([
    { t_ms: 0, ...row(LaneStatus.INSIDE, 0.1) },
    { t_ms: 33, ...row(LaneStatus.INSIDE, -0.1) },
    { t_ms: 66, ...row(LaneStatus.LOST, 5) },
    { t_ms: 100, ...row(LaneStatus.OUTSIDE_RIGHT, 0.7) },
  ]);
  close(s.rmsEyKf, Math.sqrt((0.01 + 0.01 + 0.49) / 3), 1e-9);
  close(s.trackedPct, 75, 1e-9);
  close(s.insidePct, (100 * 2) / 3, 1e-9);
  close(s.durationS, 0.1, 1e-9);
});

test('runs are timed from their first frame, persisted, and exported as CSV', () => {
  const storage = memoryStorage();
  const log = new RunLogger(storage);
  log.start(Condition.BELT);
  log.record(row(LaneStatus.INSIDE, 0.05), 5000);
  log.record(row(LaneStatus.INSIDE, -0.05), 5033);
  const run = log.stop();
  assert.equal(run.rows[1].t_ms, 33);

  const reloaded = new RunLogger(storage);
  assert.equal(reloaded.runs.length, 1);
  const lines = reloaded.toCsv().trim().split('\n');
  assert.equal(lines[0], ['run_id', ...CSV_COLUMNS].join(','));
  assert.equal(lines.length, 3);
  assert.match(lines[1], /,0,belt,inside,1,0\.0500,/);
});

test('stopping without frames saves nothing; recording only happens between start and stop', () => {
  const log = new RunLogger(memoryStorage());
  log.record(row(LaneStatus.INSIDE, 0), 0);
  log.start(Condition.NONE);
  assert.equal(log.stop(), null);
  assert.equal(log.runs.length, 0);
});

test('works without storage (private mode)', () => {
  const log = new RunLogger(null);
  log.start(Condition.AUDIO);
  log.record(row(LaneStatus.INSIDE, 0), 0);
  assert.ok(log.stop());
  assert.equal(log.runs.length, 1);
});
