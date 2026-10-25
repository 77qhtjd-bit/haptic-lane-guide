import test from 'node:test';
import assert from 'node:assert/strict';
import { LaneEstimator, yawRateFromMotion } from '../control/estimator.js';
import { HapticController, Pattern, motorBit } from '../control/controller.js';
import { commandToLevels, encodeLevels, levelForPeriod } from '../control/protocol.js';
import { LaneStatus } from '../lane/tracker.js';

const DEG = Math.PI / 180;
const close = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} not within ${tol} of ${b}`);

function seeded(seed) {
  return () => {
    seed = (seed * 1664525 + 1013904223) % 4294967296;
    return seed / 4294967296;
  };
}
const gauss = (rand) => Math.sqrt(-2 * Math.log(rand() + 1e-12)) * Math.cos(2 * Math.PI * rand());

test('yaw rate: turning left about the vertical is positive', () => {
  // Phone upright (up = device +y), rotating counterclockwise seen from above = about +y.
  close(yawRateFromMotion({ alpha: 0, beta: 0, gamma: 30 }, [0, 1, 0]), 30 * DEG, 1e-12);
  // Phone flat face up (up = device +z): same turn is about +z.
  close(yawRateFromMotion({ alpha: 30, beta: 0, gamma: 0 }, [0, 0, 1]), 30 * DEG, 1e-12);
});

test('estimator smooths noisy vision and recovers drift velocity', () => {
  const rand = seeded(3);
  const est = new LaneEstimator();
  const dtImu = 1 / 60;
  let truth = 0;
  const v = 0.2; // drifting right at 20 cm/s
  let rawErr = 0;
  let estErr = 0;
  let n = 0;
  for (let i = 0; i < 180; i++) {
    truth += v * dtImu;
    est.predict(dtImu, 0);
    if (i % 2 === 0) {
      const z = truth + 0.05 * gauss(rand); // 5 cm vision noise at 30 fps
      est.update(z, 0, 1);
      if (i > 60) {
        rawErr += (z - truth) ** 2;
        estErr += (est.state.eY - truth) ** 2;
        n++;
      }
    }
  }
  assert.ok(Math.sqrt(estErr / n) < 0.6 * Math.sqrt(rawErr / n), 'filter should cut the noise');
  close(est.state.vY, v, 0.08);
});

test('estimator rejects a one-frame outlier (wrong line matched)', () => {
  const est = new LaneEstimator();
  est.reset(0, 0);
  for (let i = 0; i < 30; i++) {
    est.predict(1 / 30, 0);
    est.update(0, 0, 1);
  }
  const res = est.update(0.6, 0, 1);
  assert.equal(res.accepted, false);
  close(est.state.eY, 0, 0.01);
});

test('estimator re-initializes when the jump persists instead of locking onto a stale state', () => {
  const est = new LaneEstimator();
  est.reset(0.5, 0);
  for (let i = 0; i < 30; i++) {
    est.predict(1 / 30, 0);
    est.update(0.5, 0, 1);
  }
  let res;
  for (let i = 0; i < 5; i++) {
    est.predict(1 / 30, 0);
    res = est.update(-0.2, 0, 1);
  }
  assert.equal(res.reset, true);
  close(est.state.eY, -0.2, 1e-9);
  close(est.state.vY, 0, 1e-9);
});

test('gyro keeps heading moving between frames, with the right sign', () => {
  const est = new LaneEstimator();
  est.reset(0, 0);
  for (let i = 0; i < 30; i++) est.predict(1 / 60, 20 * DEG); // turning left 20 deg/s for 0.5 s
  close(est.state.ePsi, -10 * DEG, 1e-9);
});

test('controller: heartbeat on motor 3 inside the deadband', () => {
  const cmd = new HapticController().step({ eY: 0.05, vY: 0, ePsi: 0 }, LaneStatus.INSIDE);
  assert.equal(cmd.pattern, Pattern.HEARTBEAT);
  assert.equal(cmd.motors, motorBit(3));
});

test('controller: small and big errors use motors 2/4 and 1/5 toward the lane center', () => {
  const ctl = new HapticController();
  const at = (eY) => ctl.step({ eY, vY: 0, ePsi: 0 }, LaneStatus.INSIDE);
  // Runner right of center -> move left -> left-side motors.
  assert.equal(at(0.2).motors, motorBit(2));
  assert.equal(at(0.5).motors, motorBit(1));
  const ctl2 = new HapticController();
  assert.equal(ctl2.step({ eY: -0.2 }, LaneStatus.INSIDE).motors, motorBit(4));
  assert.equal(ctl2.step({ eY: -0.5 }, LaneStatus.INSIDE).motors, motorBit(5));
});

test('controller: heading error alone steers before the offset grows', () => {
  // Centered but pointing 15 deg right -> move left (aim 27 cm left over the 1 m heading lookahead).
  const cmd = new HapticController().step({ eY: 0, vY: 0, ePsi: 15 * DEG }, LaneStatus.INSIDE);
  assert.equal(cmd.motors, motorBit(2));
});

test('controller: centered on a 12 m bend, the cue already says turn into the bend', () => {
  const ctl = new HapticController();
  assert.equal(ctl.effort({ eY: 0, vY: 0, ePsi: 0 }, 0), 0);
  // Right bend: small right turn (motor 4), not STRAIGHT. Left bend: motor 2.
  assert.equal(new HapticController().step({ eY: 0, vY: 0, ePsi: 0 }, LaneStatus.INSIDE, 1 / 12).motors, motorBit(4));
  assert.equal(new HapticController().step({ eY: 0, vY: 0, ePsi: 0 }, LaneStatus.INSIDE, -1 / 12).motors, motorBit(2));
  // A gentle track bend (R 36.5 m) alone stays quiet.
  assert.equal(new HapticController().step({ eY: 0, vY: 0, ePsi: 0 }, LaneStatus.INSIDE, 1 / 36.5).motors, motorBit(3));
});

test('controller: aim point is the centerline ahead as seen from the runner', () => {
  const ctl = new HapticController();
  close(ctl.aimPoint({ eY: 0.2, ePsi: 0 }).y, -0.2, 1e-12); // runner right of center -> aim left
  close(ctl.aimPoint({ eY: 0, ePsi: 0.1 }).y, -Math.tan(0.1) * ctl.cfg.headingLookahead, 1e-12); // pointing right -> aim left
});

test('controller: hysteresis stops the cue flickering on a threshold', () => {
  const ctl = new HapticController();
  const motors = [];
  for (let i = 0; i < 40; i++) {
    const eY = ctl.cfg.deadband + 0.02 * Math.sin(i); // jitter around the deadband edge
    motors.push(ctl.step({ eY }, LaneStatus.INSIDE).motors);
  }
  const changes = motors.slice(1).filter((m, i) => m !== motors[i]).length;
  assert.ok(changes <= 1, `cue changed ${changes} times`);
});

test('controller: pulses get faster as the error grows', () => {
  const p = (eY) => new HapticController().step({ eY }, LaneStatus.INSIDE).periodMs;
  assert.ok(p(0.2) > p(0.3));
  assert.ok(p(0.4) > p(0.6));
  assert.equal(p(0.6), p(0.9));
});

test('controller: outside the lane always gives the strongest cue back, lost gives stop', () => {
  const ctl = new HapticController();
  assert.equal(ctl.step({ eY: 0.9 }, LaneStatus.OUTSIDE_RIGHT).motors, motorBit(1));
  assert.equal(ctl.step({ eY: -0.9 }, LaneStatus.OUTSIDE_LEFT).motors, motorBit(5));
  const lost = ctl.step({ eY: 0 }, LaneStatus.LOST);
  assert.equal(lost.pattern, Pattern.STOP);
  assert.equal(lost.motors, motorBit(1) | motorBit(5));
  assert.equal(ctl.step({ eY: 0 }, LaneStatus.UNLOCKED).pattern, Pattern.OFF);
});

test('levels match the lane_actuator_ble firmware pulse mapping', () => {
  // Firmware: periodFor(level) = 1000 - (level - 1) * 880 / 253.
  const periodFor = (lv) => 1000 - Math.floor(((lv - 1) * 880) / 253);
  for (const period of [1000, 600, 420, 150, 120]) close(periodFor(levelForPeriod(period)), period, 4);
  assert.equal(levelForPeriod(5000), 1);
  assert.equal(levelForPeriod(10), 254);
});

test('commands become per-motor levels: one motor, both outer motors for stop, zeros for off', () => {
  const ctl = new HapticController();
  const steer = commandToLevels(ctl.step({ eY: 0.2 }, LaneStatus.INSIDE));
  assert.deepEqual(steer.map((l) => l > 0), [false, true, false, false, false]);
  assert.deepEqual(commandToLevels(ctl.step({ eY: 0 }, LaneStatus.INSIDE)), [0, 0, 1, 0, 0]); // heartbeat = slowest pulse
  assert.deepEqual(commandToLevels(ctl.step({ eY: 0 }, LaneStatus.LOST)), [254, 0, 0, 0, 254]);
  assert.deepEqual(commandToLevels(ctl.step({ eY: 0 }, LaneStatus.UNLOCKED)), [0, 0, 0, 0, 0]);
});

test('packet layout matches the firmware: [seq lo, seq hi, L1..L5]', () => {
  assert.deepEqual([...encodeLevels(0x1234, [0, 116, 0, 0, 300])], [0x34, 0x12, 0, 116, 0, 0, 255]);
});

test('controller: stride sway (1.4 Hz) is averaged out instead of flipping the belt every step', () => {
  // Jogging: the chest camera sways ±15 cm and yaws ±15 deg at stride frequency around a runner who
  // is 5 cm right of center. With frame times the cue stays on the heartbeat.
  const ctl = new HapticController();
  const cues = [];
  for (let i = 0; i < 150; i++) {
    const t = (i * 1000) / 30;
    const w = Math.sin(2 * Math.PI * 1.4 * (t / 1000));
    cues.push(ctl.step({ eY: 0.05 + 0.15 * w, vY: 0, ePsi: 15 * DEG * w }, LaneStatus.INSIDE, 0, t).motors);
  }
  const settled = cues.slice(30);
  assert.ok(settled.every((m) => m === motorBit(3)), 'heartbeat only once the window is full');
});

test('controller: a steady heading offset (camera mounted turned) is learned, a real drift still steers', () => {
  const ctl = new HapticController();
  let cmd;
  // 60 s centered with the measured heading always 12 deg right: steers at first, then quiet.
  for (let i = 0; i <= 1800; i++) cmd = ctl.step({ eY: 0, vY: 0, ePsi: 12 * DEG }, LaneStatus.INSIDE, 0, (i * 1000) / 30);
  assert.equal(cmd.motors, motorBit(3));
  // Then the runner really drifts 30 cm right: the averaged offset steers left within a second.
  for (let i = 1; i <= 30; i++) cmd = ctl.step({ eY: 0.3, vY: 0, ePsi: 12 * DEG }, LaneStatus.INSIDE, 0, 60000 + (i * 1000) / 30);
  assert.equal(cmd.motors, motorBit(2));
});

test('curve guidance: centered on a left bend gives a double tap, then gentle pulses on M2; a straight gives the heartbeat', () => {
  const ctl = new HapticController();
  const at = (i, curvature) => ctl.step({ eY: 0, vY: 0, ePsi: 0 }, LaneStatus.INSIDE, curvature, (i * 1000) / 30);
  let cmd;
  for (let i = 0; i < 30; i++) cmd = at(i, 0); // 1 s straight
  assert.equal(cmd.pattern, Pattern.HEARTBEAT);
  const cues = [];
  for (let i = 30; i < 150; i++) cues.push(at(i, -1 / 40)); // track bend to the left (R 40 m)
  const first = cues.findIndex((c) => c.pattern === Pattern.CURVE);
  assert.ok(first >= 0 && first <= 30, `curve cue within ~1 s (frame ${first})`);
  assert.equal(cues[first].motors, motorBit(2));
  assert.equal(cues[first].periodMs, ctl.cfg.curveStartPeriodMs); // double tap at the start
  const last = cues[cues.length - 1];
  assert.equal(last.pattern, Pattern.CURVE);
  assert.equal(last.periodMs, ctl.cfg.curvePeriodMs);
  for (let i = 150; i < 240; i++) cmd = at(i, 0); // back on a straight
  assert.equal(cmd.pattern, Pattern.HEARTBEAT);
});

test('curve guidance: drifting outward on the bend strengthens the left cue; right bends are ignored by default', () => {
  const ctl = new HapticController();
  let cmd;
  for (let i = 0; i < 90; i++) cmd = ctl.step({ eY: 0, vY: 0, ePsi: 0 }, LaneStatus.INSIDE, -1 / 40, (i * 1000) / 30);
  assert.equal(cmd.pattern, Pattern.CURVE);
  for (let i = 90; i < 150; i++) cmd = ctl.step({ eY: 0.45, vY: 0, ePsi: 0 }, LaneStatus.INSIDE, -1 / 40, (i * 1000) / 30);
  assert.equal(cmd.pattern, Pattern.STEER);
  assert.equal(cmd.motors, motorBit(1)); // runner 45 cm right of center (outside of a left bend): big left
  const right = new HapticController();
  for (let i = 0; i < 90; i++) cmd = right.step({ eY: 0, vY: 0, ePsi: 0 }, LaneStatus.INSIDE, 1 / 40, (i * 1000) / 30);
  assert.equal(cmd.pattern, Pattern.HEARTBEAT);
  const both = new HapticController({ curveSides: 0 });
  for (let i = 0; i < 90; i++) cmd = both.step({ eY: 0, vY: 0, ePsi: 0 }, LaneStatus.INSIDE, 1 / 40, (i * 1000) / 30);
  assert.equal(cmd.motors, motorBit(4));
});
