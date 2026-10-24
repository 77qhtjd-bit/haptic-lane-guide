// Kalman filter over the runner's lane error, x = [eY, vY, ePsi].
//   eY   (m)     lateral offset from the locked lane's centerline, + = runner right of center
//   vY   (m/s)   rate of eY, + = drifting right
//   ePsi (rad)   heading relative to the lane, + = runner points right of the lane
// predict() runs on every IMU sample (gyro yaw rate moves ePsi between frames);
// update() runs on every frame the lane tracker matched at least one line.

const DEG = Math.PI / 180;

export const DEFAULT_ESTIMATOR_CONFIG = {
  qAccel: 0.5, // (m/s^2)^2 per s: how fast lateral drift may change
  qPsi: (5 * DEG) ** 2, // rad^2 per s: gyro bias + track curvature the gyro does not see
  rY: 0.04 ** 2, // m^2: vision noise on eY at confidence 1
  rPsi: (2 * DEG) ** 2, // rad^2: vision noise on ePsi at confidence 1
  gateChi2: 9.21, // 99% for 2 dof: innovations beyond this are rejected as outliers
  maxRejects: 5, // this many outliers in a row means the filter, not vision, is wrong: re-initialize
  maxDt: 0.2, // s: longer gaps (app paused) are clamped
};

// Yaw rate about the world vertical from a W3C devicemotion event, in rad/s.
// + = turning left (counterclockwise seen from above).
// rotationRate is deg/s: alpha about device z, beta about x, gamma about y. upDevice: unit up-vector.
export function yawRateFromMotion(rotationRate, upDevice) {
  const wx = rotationRate.beta ?? 0;
  const wy = rotationRate.gamma ?? 0;
  const wz = rotationRate.alpha ?? 0;
  return (wx * upDevice[0] + wy * upDevice[1] + wz * upDevice[2]) * DEG;
}

export class LaneEstimator {
  constructor(config = {}) {
    this.cfg = { ...DEFAULT_ESTIMATOR_CONFIG, ...config };
    this.initialized = false;
    this.x = [0, 0, 0];
    this.P = diag(1, 1, 1);
  }

  reset(eY, ePsi) {
    this.x = [eY, 0, ePsi];
    this.P = diag(this.cfg.rY, 0.25, this.cfg.rPsi);
    this.initialized = true;
    this.rejects = 0;
  }

  // yawRate: rad/s about the world vertical, + = turning left (see yawRateFromMotion).
  predict(dt, yawRate = 0) {
    if (!this.initialized || dt <= 0) return;
    dt = Math.min(dt, this.cfg.maxDt);
    const [eY, vY, ePsi] = this.x;
    this.x = [eY + vY * dt, vY, ePsi - yawRate * dt];

    // P = F P F^T + Q with F = [[1, dt, 0], [0, 1, 0], [0, 0, 1]]
    const P = this.P;
    const p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1];
    const p01 = P[0][1] + dt * P[1][1];
    const p02 = P[0][2] + dt * P[1][2];
    const { qAccel: q, qPsi } = this.cfg;
    this.P = [
      [p00 + (q * dt ** 4) / 4, p01 + (q * dt ** 3) / 2, p02],
      [p01 + (q * dt ** 3) / 2, P[1][1] + q * dt * dt, P[1][2]],
      [p02, P[1][2], P[2][2] + qPsi * dt],
    ];
  }

  // Measurement z = [eY, ePsi] from the lane tracker; noise grows as confidence drops.
  // Returns { accepted, nis, reset } (nis = normalized innovation squared, for tuning).
  update(eY, ePsi, confidence = 1) {
    if (!this.initialized) {
      this.reset(eY, ePsi);
      return { accepted: true, nis: 0 };
    }
    const c = Math.max(confidence, 0.1);
    const P = this.P;
    // H = [[1, 0, 0], [0, 0, 1]]
    const y0 = eY - this.x[0];
    const y1 = ePsi - this.x[2];
    const s00 = P[0][0] + this.cfg.rY / c;
    const s01 = P[0][2];
    const s11 = P[2][2] + this.cfg.rPsi / c;
    const det = s00 * s11 - s01 * s01;
    const i00 = s11 / det;
    const i01 = -s01 / det;
    const i11 = s00 / det;
    const nis = y0 * (i00 * y0 + i01 * y1) + y1 * (i01 * y0 + i11 * y1);
    if (nis > this.cfg.gateChi2) {
      // A one-off jump (wrong line matched) is dropped; a persistent one would otherwise lock the
      // filter onto a stale state forever (seen after big real jumps or an IMU stall).
      if (++this.rejects < this.cfg.maxRejects) return { accepted: false, nis };
      this.reset(eY, ePsi);
      return { accepted: true, nis, reset: true };
    }
    this.rejects = 0;

    // K = P H^T S^-1 (3x2)
    const K = [0, 1, 2].map((r) => {
      const a = P[r][0];
      const b = P[r][2];
      return [a * i00 + b * i01, a * i01 + b * i11];
    });
    this.x = this.x.map((v, r) => v + K[r][0] * y0 + K[r][1] * y1);
    // P = (I - K H) P
    this.P = [0, 1, 2].map((r) => [0, 1, 2].map((col) => P[r][col] - K[r][0] * P[0][col] - K[r][1] * P[2][col]));
    return { accepted: true, nis };
  }

  get state() {
    const [eY, vY, ePsi] = this.x;
    return { eY, vY, ePsi, sigmaY: Math.sqrt(this.P[0][0]) };
  }
}

function diag(a, b, c) {
  return [
    [a, 0, 0],
    [0, b, 0],
    [0, 0, c],
  ];
}
