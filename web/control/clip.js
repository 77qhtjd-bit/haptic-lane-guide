// Records the live camera stream + full-rate IMU while the app runs, in the same format as
// web/record.html (rec_<time>.mp4 + rec_<time>.json), so web/replay.html can play the run back
// through the pipeline (and drive the belt) for a demo.
//
// Clock: performance.timeOrigin + performance.now() (ms), like record.html. Video time on that clock
// is rec_start_t + the container timestamp; frames[] holds [capture time, mediaTime] per frame.

const PAGE_VERSION = 'spike-ground-clip-1';
const now = () => performance.timeOrigin + performance.now();

function pickMime() {
  return ['video/mp4;codecs=avc1', 'video/mp4', 'video/webm;codecs=vp8', 'video/webm']
    .find((m) => MediaRecorder.isTypeSupported && MediaRecorder.isTypeSupported(m)) || '';
}

function stamp() {
  const d = new Date();
  const p = (x) => String(x).padStart(2, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

export class ClipRecorder {
  // stream: the camera MediaStream; video: the <video> element showing it (for frame timestamps).
  constructor(stream, video, { bitrate = 2500000, maxMinutes = 10 } = {}) {
    this.stream = stream;
    this.video = video;
    this.bitrate = bitrate;
    this.maxMinutes = maxMinutes;
    this.recorder = null;
    this.rec = null;
    this.chunks = [];
    this.lastMotionT = 0;
  }

  get recording() {
    return !!this.recorder;
  }

  static supported() {
    return typeof MediaRecorder !== 'undefined';
  }

  start(memo = '') {
    const mimeType = pickMime();
    const opts = mimeType ? { mimeType, videoBitsPerSecond: this.bitrate } : { videoBitsPerSecond: this.bitrate };
    this.recorder = new MediaRecorder(this.stream, opts);
    this.chunks = [];
    const track = this.stream.getVideoTracks()[0];
    const t0 = now();
    this.rec = {
      page_version: PAGE_VERSION, name: `rec_${stamp()}`, userAgent: navigator.userAgent, memo,
      mimeType: this.recorder.mimeType || mimeType, bitrate: this.bitrate, max_minutes: this.maxMinutes,
      trackLabel: track?.label, trackSettings: track?.getSettings?.(),
      rec_start_call_t: t0, rec_start_event_t: null, rec_start_t: t0, rec_stop_t: null, stop_reason: null,
      clock: 'performance.timeOrigin + performance.now() (ms)',
      samples: [], frames: [], imu_gaps: [], events: [],
    };
    this.recorder.onstart = () => {
      this.rec.rec_start_event_t = now();
      this.rec.rec_start_t = this.rec.rec_start_event_t;
    };
    this.recorder.ondataavailable = (e) => { if (e.data && e.data.size) this.chunks.push(e.data); };
    this.done = new Promise((resolve) => { this.recorder.onstop = () => resolve(this.finish()); });
    this.recorder.start(1000);
    this.stopTimer = setTimeout(() => this.stop('max length'), this.maxMinutes * 60000);
    if (this.video.requestVideoFrameCallback) this.video.requestVideoFrameCallback((ts, md) => this.onFrame(ts, md));
  }

  // Resolves to { files: [video, json], info } once the recorder has flushed.
  stop(reason = 'manual') {
    if (!this.recorder) return this.done ?? Promise.resolve(null);
    clearTimeout(this.stopTimer);
    this.rec.rec_stop_t = now();
    this.rec.stop_reason = reason;
    this.recorder.stop();
    return this.done;
  }

  finish() {
    const rec = this.rec;
    const type = rec.mimeType || 'video/mp4';
    const ext = type.includes('webm') ? 'webm' : 'mp4';
    const files = [
      new File(this.chunks, `${rec.name}.${ext}`, { type }),
      new File([JSON.stringify(rec)], `${rec.name}.json`, { type: 'application/json' }),
    ];
    const info = `${rec.name}: ${((rec.rec_stop_t - rec.rec_start_t) / 1000).toFixed(0)} s, ` +
      `${(files[0].size / 1e6).toFixed(1)} MB video, ${rec.samples.length} IMU samples, ${rec.frames.length} frames`;
    this.recorder = null;
    this.rec = null;
    this.chunks = [];
    return { files, info };
  }

  onFrame(ts, md) {
    if (!this.rec) return;
    this.rec.frames.push([+(performance.timeOrigin + (md && md.captureTime ? md.captureTime : ts)).toFixed(1),
      md && md.mediaTime != null ? +md.mediaTime.toFixed(4) : null]);
    this.video.requestVideoFrameCallback((t, m) => this.onFrame(t, m));
  }

  // IMU listeners call these with the raw events.
  onMotion(e) {
    const t = now();
    if (this.rec && this.lastMotionT && t - this.lastMotionT > 200) {
      this.rec.imu_gaps.push([Math.round(t - this.rec.rec_start_t), Math.round(t - this.lastMotionT)]);
    }
    this.lastMotionT = t;
    if (!this.rec) return;
    const a = e.acceleration || {};
    const g = e.accelerationIncludingGravity || {};
    const r = e.rotationRate || {};
    this.rec.samples.push({ k: 'm', t, ax: a.x, ay: a.y, az: a.z, gx: g.x, gy: g.y, gz: g.z,
      ra: r.alpha, rb: r.beta, rg: r.gamma, iv: e.interval });
  }

  onOrientation(e) {
    if (this.rec) this.rec.samples.push({ k: 'o', t: now(), al: e.alpha, be: e.beta, ga: e.gamma, abs: e.absolute });
  }

  event(name) {
    if (this.rec) this.rec.events.push([Math.round(now() - this.rec.rec_start_t), name]);
  }
}

// iOS share sheet ("Save to Files") where available (Bluefy blocks download links); returns false
// when the caller should fall back to download links.
export async function saveFiles(files) {
  if (navigator.canShare && navigator.canShare({ files })) {
    await navigator.share({ files, title: files[0].name });
    return true;
  }
  return false;
}
