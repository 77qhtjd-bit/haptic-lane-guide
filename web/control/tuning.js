// On-page tuning panel for the tracker and haptic controller rules, shared by spike-ground.html and
// replay.html. Values apply immediately to the running tracker/controller, are kept on this device
// (localStorage, when available) and can be copied as a link (?t_<key>=..&c_<key>=..). Precedence
// when the page opens: defaults < saved values < values in the address.

import { DEFAULT_TRACKER_CONFIG } from '../lane/tracker.js';
import { DEFAULT_CONTROL_CONFIG } from './controller.js';

// t = tracker, c = controller. type: 'num' (input), 'bool' (checkbox, true/false), 'flag' (checkbox, 1/0).
export const TUNING_FIELDS = [
  { group: 'Belt cue', o: 'c', key: 'deadband', label: 'quiet zone (m)', step: 0.01 },
  { group: 'Belt cue', o: 'c', key: 'strong', label: 'big-turn zone from (m)', step: 0.01 },
  { group: 'Belt cue', o: 'c', key: 'hysteresis', label: 'zone hysteresis (m)', step: 0.01 },
  { group: 'Belt cue', o: 'c', key: 'positionAvgS', label: 'position average (s)', step: 0.1 },
  { group: 'Belt cue', o: 'c', key: 'headingAvgS', label: 'heading average (s)', step: 0.5 },
  { group: 'Belt cue', o: 'c', key: 'headingLookahead', label: 'heading lookahead (m)', step: 0.1 },
  { group: 'Belt cue', o: 'c', key: 'heartbeatPeriodMs', label: 'heartbeat period (ms)', step: 100 },
  { group: 'Curve guidance', o: 'c', key: 'curveGuide', label: 'on', type: 'flag' },
  { group: 'Curve guidance', o: 'c', key: 'curveSides', label: 'bends (-1 left only, 0 both)', step: 1 },
  { group: 'Curve guidance', o: 'c', key: 'curvePeriodMs', label: 'pulse period (ms)', step: 50 },
  { group: 'Curve guidance', o: 'c', key: 'curveOnK', label: 'start above curvature (1/m)', step: 0.001 },
  { group: 'Curve guidance', o: 'c', key: 'curveOffK', label: 'end below curvature (1/m)', step: 0.001 },
  { group: 'Curve guidance', o: 'c', key: 'curveAvgS', label: 'curvature average (s)', step: 0.1 },
  { group: 'Curve guidance', o: 'c', key: 'curveHoldS', label: 'hold before switching (s)', step: 0.1 },
  { group: 'Lane tracking', o: 't', key: 'verifySlope', label: 'crossing-line filter (tan; 0 off)', step: 0.01 },
  { group: 'Lane tracking', o: 't', key: 'traceMinLength', label: 'trace fill min length (m; 0 off)', step: 0.1 },
  { group: 'Lane tracking', o: 't', key: 'jumpGuard', label: 'jump guard (m; 0 off)', step: 0.05 },
  { group: 'Lane tracking', o: 't', key: 'jumpFrames', label: 'jump must persist (frames)', step: 1 },
  { group: 'Lane tracking', o: 't', key: 'widthCheck', label: 'width check (fraction; 0 off)', step: 0.01 },
  { group: 'Lane tracking', o: 't', key: 'statusFrames', label: 'inside/outside after (frames)', step: 1 },
  { group: 'Lane tracking', o: 't', key: 'settleFrames', label: 'low confidence after lock (frames)', step: 1 },
  { group: 'Lane tracking', o: 't', key: 'lockStraddle', label: 'lock the pair around the runner', type: 'bool' },
];

const DEFAULTS = { t: DEFAULT_TRACKER_CONFIG, c: DEFAULT_CONTROL_CONFIG };

function loadSaved(storageKey) {
  try {
    const v = JSON.parse(localStorage.getItem(storageKey) || 'null');
    return v && v.t && v.c ? v : { t: {}, c: {} };
  } catch (_) {
    return { t: {}, c: {} };
  }
}
function save(storageKey, overrides) {
  try { localStorage.setItem(storageKey, JSON.stringify(overrides)); } catch (_) { /* private mode: not kept */ }
}

// container: an element to fill. overrides: { t: {...}, c: {...} } from the address (mutated in place
// so the pages' own creation code sees the merged values). getTracker/getController: current objects
// (may return null before Start). onChange(text): called with a one-line summary of non-defaults.
export function setupTuning(container, { overrides, getTracker, getController, storageKey, onChange = () => {} }) {
  const saved = loadSaved(storageKey);
  for (const o of ['t', 'c']) for (const [k, v] of Object.entries(saved[o])) if (!(k in overrides[o])) overrides[o][k] = v;

  const value = (f) => (f.key in overrides[f.o] ? overrides[f.o][f.key] : DEFAULTS[f.o][f.key]);
  const isChanged = (f) => f.key in overrides[f.o] && overrides[f.o][f.key] !== DEFAULTS[f.o][f.key];
  const summary = () => {
    const parts = TUNING_FIELDS.filter(isChanged).map((f) => `${f.o === 't' ? 'tracker' : 'control'}.${f.key}=${value(f)}`);
    return parts.length ? `changed: ${parts.join('  ')}` : 'all rules at their defaults';
  };
  const applyLive = (f, v) => {
    const target = f.o === 't' ? getTracker() : getController();
    if (target && target.cfg) target.cfg[f.key] = v;
  };

  container.innerHTML = '';
  const style = document.createElement('style');
  style.textContent = `.tun-row{display:flex;justify-content:space-between;align-items:center;gap:8px;padding:3px 0;border-bottom:1px solid #333}
    .tun-row input[type=number]{width:90px;font-size:15px}.tun-row.changed span{color:#ffd60a}.tun-group{margin:10px 0 2px;color:#9cf;font-weight:700}
    .tun-buttons button{margin:6px 6px 0 0;padding:8px 12px;font-size:15px}`;
  container.appendChild(style);
  let group = null;
  const rows = [];
  for (const f of TUNING_FIELDS) {
    if (!(f.key in DEFAULTS[f.o])) continue; // field from a newer/older module version
    if (f.group !== group) {
      group = f.group;
      const g = document.createElement('div');
      g.className = 'tun-group';
      g.textContent = group;
      container.appendChild(g);
    }
    const row = document.createElement('label');
    row.className = 'tun-row';
    const name = document.createElement('span');
    name.textContent = f.label;
    const input = document.createElement('input');
    if (f.type === 'bool' || f.type === 'flag') {
      input.type = 'checkbox';
      input.checked = !!value(f);
    } else {
      input.type = 'number';
      input.step = f.step;
      input.value = value(f);
    }
    input.addEventListener('change', () => {
      const v = f.type === 'bool' ? input.checked : f.type === 'flag' ? (input.checked ? 1 : 0) : +input.value;
      if (v === DEFAULTS[f.o][f.key]) delete overrides[f.o][f.key];
      else overrides[f.o][f.key] = v;
      applyLive(f, v);
      save(storageKey, { t: overrides.t, c: overrides.c });
      row.classList.toggle('changed', isChanged(f));
      onChange(summary());
    });
    row.classList.toggle('changed', isChanged(f));
    row.append(name, input);
    container.appendChild(row);
    rows.push({ f, input, row });
  }

  const buttons = document.createElement('div');
  buttons.className = 'tun-buttons';
  const reset = document.createElement('button');
  reset.textContent = 'Reset to defaults';
  reset.onclick = () => {
    for (const { f, input, row } of rows) {
      delete overrides[f.o][f.key];
      const d = DEFAULTS[f.o][f.key];
      if (input.type === 'checkbox') input.checked = !!d;
      else input.value = d;
      applyLive(f, d);
      row.classList.remove('changed');
    }
    save(storageKey, { t: {}, c: {} });
    onChange(summary());
  };
  const link = document.createElement('button');
  link.textContent = 'Copy link with these settings';
  link.onclick = async () => {
    const url = new URL(location.href);
    for (const k of [...url.searchParams.keys()]) if (k.startsWith('t_') || k.startsWith('c_')) url.searchParams.delete(k);
    for (const { f } of rows) if (isChanged(f)) url.searchParams.set(`${f.o}_${f.key}`, String(+value(f)));
    try { await navigator.clipboard.writeText(url.toString()); link.textContent = 'Link copied'; }
    catch (_) { prompt('Copy this link', url.toString()); }
    setTimeout(() => { link.textContent = 'Copy link with these settings'; }, 2000);
  };
  buttons.append(reset, link);
  container.appendChild(buttons);
  onChange(summary());
}
