// Step 1 of perception: white track-line pixels (OpenCV.js) -> ground points (IMU attitude)
// -> lateral histogram peaks, one per track line.

import {
  intrinsicsFromFov,
  groundBasis,
  projectToGround,
  rotatePoints,
  findHeading,
  lateralHistogram,
  findPeaks,
  estimateHeight,
} from './ground.js';

export const DEFAULT_CONFIG = {
  procWidth: 120, // frames are drawn into a canvas of this size before OpenCV sees them
  procHeight: 160,
  hfovDeg: 53, // horizontal FOV of the frame as delivered; ~53 for the iPhone 1x camera in portrait 4:3, calibrate
  cameraHeight: 1.3, // m, chest mount
  whiteVMin: 180, // HSV value floor for white paint
  whiteSMax: 60, // HSV saturation ceiling (red/blue track surface is saturated)
  openKernel: 3, // morphological open removes speckle
  maxLineWidthPx: 7, // top-hat: white runs wider than this (walls, white floor, lights) are not lines
  maxPoints: 4000,
  band: { xMin: 1, xMax: 4, yHalf: 3, bin: 0.02 },
  peaks: { sigma: 0.03, minSeparation: 0.6, minFraction: 0.2, minCount: 3 },
  heading: { rangeDeg: 25, stepDeg: 1.5 },
};

export class GroundLineDetector {
  constructor(cv, config = {}) {
    this.cv = cv;
    this.cfg = { ...DEFAULT_CONFIG, ...config };
    const { procWidth: w, procHeight: h } = this.cfg;
    this.K = intrinsicsFromFov(w, h, this.cfg.hfovDeg);
    this.rgb = new cv.Mat();
    this.hsv = new cv.Mat();
    this.mask = new cv.Mat();
    this.kernel = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(this.cfg.openKernel, this.cfg.openKernel));
    this.lineKernel = cv.getStructuringElement(cv.MORPH_RECT, new cv.Size(this.cfg.maxLineWidthPx, 1));
    this.setThresholds(this.cfg.whiteVMin, this.cfg.whiteSMax);
    this.pixels = new Float32Array(this.cfg.maxPoints * 2);
  }

  setThresholds(vMin, sMax) {
    const { cv } = this;
    const { procWidth: w, procHeight: h } = this.cfg;
    this.low?.delete();
    this.high?.delete();
    this.low = new cv.Mat(h, w, cv.CV_8UC3, [0, 0, vMin, 0]);
    this.high = new cv.Mat(h, w, cv.CV_8UC3, [180, sMax, 255, 0]);
    this.cfg.whiteVMin = vMin;
    this.cfg.whiteSMax = sMax;
  }

  // rgba: cv.Mat (CV_8UC4) of procWidth x procHeight. upCam: up-vector in camera coords at capture time.
  // headingPrior: lane heading (rad) to search around, e.g. the previous frame's result.
  process(rgba, upCam, headingPrior = 0) {
    const { cv, cfg } = this;
    cv.cvtColor(rgba, this.rgb, cv.COLOR_RGBA2RGB);
    cv.cvtColor(this.rgb, this.hsv, cv.COLOR_RGB2HSV);
    cv.inRange(this.hsv, this.low, this.high, this.mask);
    cv.morphologyEx(this.mask, this.mask, cv.MORPH_OPEN, this.kernel);
    // Keep only thin white structures: top-hat = mask minus its opening with a wide horizontal kernel.
    // Track lines are 2-4 px wide at this resolution; wide bright areas vanish.
    cv.morphologyEx(this.mask, this.mask, cv.MORPH_TOPHAT, this.lineKernel);

    const basis = upCam && groundBasis(upCam);
    if (!basis) return { ok: false, reason: upCam ? 'camera-vertical' : 'no-attitude' };

    const count = this.collectMaskPixels();
    const ground = projectToGround(this.pixels, count, this.K, basis, cfg.cameraHeight);
    const heading = findHeading(ground.points, ground.count, cfg.band, { ...cfg.heading, center: headingPrior });
    const aligned = rotatePoints(ground.points, ground.count, heading);
    const hist = lateralHistogram(aligned, ground.count, cfg.band);
    const peaks = findPeaks(hist, cfg.peaks);
    return {
      ok: true,
      basis,
      maskPixels: count,
      ground,
      heading,
      hist,
      peaks,
      heightEstimate: estimateHeight(peaks, cfg.cameraHeight),
    };
  }

  // Mask pixel coordinates, subsampled evenly when there are more than maxPoints.
  collectMaskPixels() {
    const { data, cols, rows } = this.mask;
    const nonZero = this.cv.countNonZero(this.mask);
    const step = Math.max(1, Math.ceil(nonZero / this.cfg.maxPoints));
    let seen = 0;
    let n = 0;
    for (let v = 0; v < rows; v++) {
      const row = v * cols;
      for (let u = 0; u < cols; u++) {
        if (data[row + u] === 0) continue;
        if (seen++ % step !== 0 || n >= this.cfg.maxPoints) continue;
        this.pixels[2 * n] = u + 0.5;
        this.pixels[2 * n + 1] = v + 0.5;
        n++;
      }
    }
    return n;
  }

  delete() {
    for (const m of [this.rgb, this.hsv, this.mask, this.kernel, this.lineKernel, this.low, this.high]) m.delete();
  }
}
