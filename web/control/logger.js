// Per-frame run logs for the proof chart: the same lane walked with no feedback, audio feedback,
// and the belt, compared by lateral-error RMS. Runs persist in localStorage when it is available.

import { LaneStatus } from '../lane/tracker.js';

export const Condition = Object.freeze({ NONE: 'none', AUDIO: 'audio', BELT: 'belt' });

export const CSV_COLUMNS = [
  't_ms', 'condition', 'status', 'confidence',
  'eY_raw_m', 'ePsi_raw_rad', 'curvature_1pm', 'eY_kf_m', 'vY_kf_mps', 'ePsi_kf_rad',
  'lane_width_m', 'matched', 'u_m', 'zone', 'cue', 'motors', 'period_ms', 'L1', 'L2', 'L3', 'L4', 'L5', 'imu_age_ms', 'fps',
];

const STORAGE_KEY = 'laneguide.runs.v1';
const TRACKED = new Set([LaneStatus.INSIDE, LaneStatus.OUTSIDE_LEFT, LaneStatus.OUTSIDE_RIGHT]);

export class RunLogger {
  constructor(storage = safeStorage()) {
    this.storage = storage;
    this.runs = this.load();
    this.current = null;
  }

  get recording() {
    return !!this.current;
  }

  start(condition) {
    this.current = { id: Date.now(), condition, rows: [] };
  }

  // row: object with CSV_COLUMNS keys except t_ms and condition.
  record(row, nowMs) {
    if (!this.current) return;
    if (this.current.rows.length === 0) this.current.t0 = nowMs;
    this.current.rows.push({ t_ms: Math.round(nowMs - this.current.t0), condition: this.current.condition, ...row });
  }

  stop() {
    const run = this.current;
    this.current = null;
    if (!run || run.rows.length === 0) return null;
    run.summary = summarize(run.rows);
    this.runs.push(run);
    this.save();
    return run;
  }

  clear() {
    this.runs = [];
    this.save();
  }

  toCsv(runs = this.runs) {
    const lines = [['run_id', ...CSV_COLUMNS].join(',')];
    for (const run of runs) {
      for (const row of run.rows) lines.push([run.id, ...CSV_COLUMNS.map((c) => formatCell(row[c]))].join(','));
    }
    return lines.join('\n') + '\n';
  }

  load() {
    try {
      const raw = this.storage?.getItem(STORAGE_KEY);
      return raw ? JSON.parse(raw) : [];
    } catch {
      return [];
    }
  }

  save() {
    try {
      this.storage?.setItem(STORAGE_KEY, JSON.stringify(this.runs));
    } catch {
      // Storage full or blocked: runs stay in memory for this session and can still be exported.
    }
  }
}

// RMS uses only frames where the lane was tracked (lost/unlocked frames have no meaningful error).
export function summarize(rows) {
  const tracked = rows.filter((r) => TRACKED.has(r.status));
  const rms = (key) =>
    tracked.length ? Math.sqrt(tracked.reduce((s, r) => s + r[key] ** 2, 0) / tracked.length) : NaN;
  const inside = tracked.filter((r) => r.status === LaneStatus.INSIDE).length;
  return {
    frames: rows.length,
    durationS: rows.length ? (rows[rows.length - 1].t_ms - rows[0].t_ms) / 1000 : 0,
    trackedPct: rows.length ? (100 * tracked.length) / rows.length : 0,
    rmsEyKf: rms('eY_kf_m'),
    rmsEyRaw: rms('eY_raw_m'),
    maxAbsEy: tracked.reduce((m, r) => Math.max(m, Math.abs(r.eY_kf_m)), 0),
    insidePct: tracked.length ? (100 * inside) / tracked.length : 0,
  };
}

function formatCell(v) {
  if (v === undefined || v === null || Number.isNaN(v)) return '';
  if (typeof v === 'number') return Number.isInteger(v) ? String(v) : v.toFixed(4);
  return String(v);
}

function safeStorage() {
  try {
    return typeof localStorage === 'undefined' ? null : localStorage;
  } catch {
    return null;
  }
}
