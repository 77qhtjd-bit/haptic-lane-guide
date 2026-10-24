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
  segmentPeaks,
  CURVE_SEGMENTS,
  estimateHeight,
} from './ground.js';

export const DEFAULT_CONFIG = {
  // Frames are drawn into a canvas of this size before OpenCV sees them. At 120 px wide a 5 cm
  // line is 1-2 px beyond 2 m and vanishes; 240 px keeps lines visible to ~5 m (~2 ms per frame).
  procWidth: 240,
  procHeight: 320,
  hfovDeg: 53, // horizontal FOV of the frame as delivered; ~53 for the iPhone 1x camera in portrait 4:3, calibrate
  cameraHeight: 1.3, // m, chest mount
  laneWidth: 1.22, // m between the two lines of one lane (track standard; practice lanes differ)
  whiteVMin: 180, // HSV value floor for white paint
  whiteSMax: 60, // HSV saturation ceiling (red/blue track surface is saturated)
  openKernel: 2, // morphological open removes speckle (3 would erase far lines)
  // Top-hat: white runs wider than a line are not lines (walls, white floor, lights). The limit is
  // in meters and converted per horizontal band, because a near line is many pixels wide and a far
  // one only a few (a fixed 14 px erased near tape in the first phone tests).
  maxLineWidthM: 0.15,
  lineFilterBands: 6,
  maxPoints: 4000,
  band: { xMin: 1, xMax: 4, yHalf: 3, bin: 0.02 },
  peaks: { sigma: 0.03, minSeparation: 0.6, minFraction: 0.2, minCount: 3 },
  heading: { rangeDeg: 25, stepDeg: 1.5 },
  segments: CURVE_SEGMENTS, // forward segments for curved-lane fitting
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
    this.lineKernels = new Map(); // kernel width (px) -> structuring element
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

    const basis = upCam && groundBasis(upCam);
    if (!basis) return { ok: false, reason: upCam ? 'camera-vertical' : 'no-attitude' };
    this.keepThinLines(basis);

    const count = this.collectMaskPixels();
    const ground = projectToGround(this.pixels, count, this.K, basis, cfg.cameraHeight);
    const heading = findHeading(ground.points, ground.count, cfg.band, { ...cfg.heading, center: headingPrior });
    const aligned = rotatePoints(ground.points, ground.count, heading);
    const hist = lateralHistogram(aligned, ground.count, cfg.band);
    const peaks = findPeaks(hist, cfg.peaks);
    const segments = segmentPeaks(aligned, ground.count, { ...cfg.band, segments: cfg.segments }, cfg.peaks);
    return {
      ok: true,
      basis,
      maskPixels: count,
      ground,
      heading,
      hist,
      peaks,
      segments,
      heightEstimate: estimateHeight(peaks, cfg.cameraHeight, cfg.laneWidth),
    };
  }

  // Top-hat per horizontal band (mask minus its opening with a wide horizontal kernel): keeps white
  // runs up to maxLineWidthM wide at that band's ground distance, removes anything wider.
  keepThinLines(basis) {
    const { cv, cfg, K } = this;
    const rows = this.mask.rows;
    const cols = this.mask.cols;
    const bands = cfg.lineFilterBands;
    for (let b = 0; b < bands; b++) {
      const v0 = Math.floor((b * rows) / bands);
      const v1 = Math.floor(((b + 1) * rows) / bands);
      const kw = this.lineKernelWidth(basis, (v0 + v1) / 2);
      const roi = this.mask.roi(new cv.Rect(0, v0, cols, v1 - v0));
      cv.morphologyEx(roi, roi, cv.MORPH_TOPHAT, this.lineKernelFor(kw));
      roi.delete();
    }
  }

  // Kernel width (px) for a line maxLineWidthM wide at image row v, measured at the image center.
  lineKernelWidth(basis, v) {
    const { K, cfg } = this;
    const g = projectToGround(Float32Array.from([K.cx, v, K.cx + 1, v]), 2, K, basis, cfg.cameraHeight, 50);
    if (g.count < 2) return 5; // at or above the horizon: nothing here is a ground line anyway
    const metersPerPx = Math.hypot(g.points[2] - g.points[0], g.points[3] - g.points[1]);
    const px = Math.ceil(cfg.maxLineWidthM / metersPerPx) + 2;
    return Math.min(81, Math.max(5, px | 1));
  }

  lineKernelFor(width) {
    let k = this.lineKernels.get(width);
    if (!k) {
      k = this.cv.getStructuringElement(this.cv.MORPH_RECT, new this.cv.Size(width, 1));
      this.lineKernels.set(width, k);
    }
    return k;
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
    for (const m of [this.rgb, this.hsv, this.mask, this.kernel, this.low, this.high]) m.delete();
    for (const k of this.lineKernels.values()) k.delete();
  }
}
