"""Verify a recorded session: IMU/frame rates, gaps, plot, and IMU-overlay video.

Usage: python verify.py data/<session_id>

Analyzes every source found in the session directory:
  - live:   imu.csv + frames/<t>.jpg
  - record: rec_<tag>_imu.csv + rec_<tag>.<ext> (start timestamp from meta.json)
Outputs go to <session_dir>/verify/.
"""
import csv
import json
import math
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

GAP_MS = 50.0


def load_imu(path):
    """Return dict of numpy arrays for 'motion' rows (and orientation timestamps)."""
    motion, orient_t = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            if row["kind"] == "motion":
                motion.append(row)
            elif row["kind"] == "orientation":
                orient_t.append(float(row["t"]))

    def col(name):
        return np.array([float(r[name]) if r[name] not in ("", "None") else np.nan for r in motion])

    cols = ["t", "accg_x", "accg_y", "accg_z", "rot_alpha", "rot_beta", "rot_gamma"]
    d = {c: col(c) for c in cols} if motion else {c: np.array([]) for c in cols}
    d["orient_t"] = np.array(orient_t)
    return d


def rate_report(name, t, gap_ms=None):
    """Print rate stats for timestamps t (ms). Returns median interval in ms."""
    print(f"  {name}: {len(t)} samples")
    if len(t) < 2:
        print("    not enough samples")
        return None
    dt = np.diff(t)
    dur = (t[-1] - t[0]) / 1000.0
    nonmono = int(np.sum(dt <= 0))
    # Per-second window counts give mean/min/max Hz.
    bins = np.arange(t[0], t[-1], 1000.0)
    counts = np.histogram(t, bins=bins)[0] if len(bins) > 1 else np.array([len(t) / max(dur, 1e-9)])
    print(f"    duration {dur:.1f} s, overall {(len(t) - 1) / dur:.1f} Hz")
    print(f"    per-second Hz: mean {counts.mean():.1f}, min {counts.min()}, max {counts.max()}")
    print(f"    interval ms: median {np.median(dt):.2f}, min {dt.min():.2f}, max {dt.max():.2f}")
    print(f"    monotonic: {'yes' if nonmono == 0 else f'NO ({nonmono} non-increasing steps)'}")
    gap = gap_ms if gap_ms is not None else 1.5 * np.median(dt)
    gaps = np.where(dt > gap)[0]
    print(f"    gaps > {gap:.0f} ms: {len(gaps)}" + (
        "  (largest: " + ", ".join(f"{dt[i]:.0f} ms @ {(t[i] - t[0]) / 1000:.1f}s"
                                   for i in gaps[np.argsort(dt[gaps])[::-1][:5]]) + ")" if len(gaps) else ""))
    return float(np.median(dt))


def plot_imu(imu, frame_t, out_png, title):
    t0 = imu["t"][0]
    ts = (imu["t"] - t0) / 1000.0
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for c, lbl in (("accg_x", "x"), ("accg_y", "y"), ("accg_z", "z")):
        axes[0].plot(ts, imu[c], lw=0.8, label=lbl)
    axes[0].set_ylabel("accelerationIncludingGravity (m/s²)")
    for c, lbl in (("rot_alpha", "alpha"), ("rot_beta", "beta"), ("rot_gamma", "gamma")):
        axes[1].plot(ts, imu[c], lw=0.8, label=lbl)
    axes[1].set_ylabel("rotationRate (deg/s)")
    axes[1].set_xlabel("time (s)")
    if len(frame_t):
        # Frame times as a rug along the bottom of the gyro plot.
        y = np.nanmin(np.concatenate([imu[c] for c in ("rot_alpha", "rot_beta", "rot_gamma")]))
        axes[1].plot((frame_t - t0) / 1000.0, np.full(len(frame_t), y), "|", color="k", ms=6, label="frames")
    for ax in axes:
        ax.legend(loc="upper right")
        ax.grid(alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"  plot: {out_png}")


def roll_pitch(gx, gy, gz):
    roll = math.degrees(math.atan2(gy, gz))
    pitch = math.degrees(math.atan2(-gx, math.hypot(gy, gz)))
    return roll, pitch


def put_lines(img, lines):
    cv2.rectangle(img, (0, 0), (min(img.shape[1], 470), 24 * len(lines) + 8), (0, 0, 0), -1)
    y = 22
    for line in lines:
        cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        y += 24


def write_overlay(frames, imu, out_mp4, fps):
    """frames: iterable of (t_ms, BGR image). Overlays nearest IMU motion sample."""
    writer = None
    t_first = None
    n = 0
    for t, img in frames:
        if writer is None:
            h, w = img.shape[:2]
            writer = cv2.VideoWriter(str(out_mp4), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            t_first = t
        lines = [f"t = {(t - t_first) / 1000:7.3f} s"]
        if len(imu["t"]):
            i = int(np.searchsorted(imu["t"], t))
            if i > 0 and (i == len(imu["t"]) or abs(imu["t"][i - 1] - t) < abs(imu["t"][i] - t)):
                i -= 1
            r, p = roll_pitch(imu["accg_x"][i], imu["accg_y"][i], imu["accg_z"][i])
            lines += [
                f"IMU dt = {imu['t'][i] - t:+6.1f} ms",
                f"roll {r:+6.1f}  pitch {p:+6.1f} deg",
                f"gyro a {imu['rot_alpha'][i]:+7.1f} b {imu['rot_beta'][i]:+7.1f} g {imu['rot_gamma'][i]:+7.1f}",
            ]
        put_lines(img, lines)
        writer.write(img)
        n += 1
    if writer is not None:
        writer.release()
        print(f"  overlay video: {out_mp4} ({n} frames @ {fps:.1f} fps)")
    else:
        print("  overlay video: no frames")


def live_frames(paths):
    for p, t in paths:
        img = cv2.imread(str(p))
        if img is not None:
            yield t, img


def video_frames(path, start_t):
    cap = cv2.VideoCapture(str(path))
    while True:
        ok, img = cap.read()
        if not ok:
            break
        yield start_t + cap.get(cv2.CAP_PROP_POS_MSEC), img
    cap.release()


def video_frame_times(path, start_t):
    cap = cv2.VideoCapture(str(path))
    ts = []
    while cap.grab():
        ts.append(start_t + cap.get(cv2.CAP_PROP_POS_MSEC))
    cap.release()
    return np.array(ts)


def analyze(label, imu_csv, frame_t, frames_iter, out_dir):
    print(f"\n=== {label} ===")
    imu = load_imu(imu_csv)
    rate_report("IMU motion", imu["t"], GAP_MS)
    rate_report("IMU orientation", imu["orient_t"], GAP_MS)
    med = rate_report("frames", frame_t)
    if len(imu["t"]) and len(frame_t):
        print(f"  IMU span  {imu['t'][0]:.0f} .. {imu['t'][-1]:.0f}")
        print(f"  frame span {frame_t[0]:.0f} .. {frame_t[-1]:.0f}")
    if len(imu["t"]) < 2:
        print("  no IMU motion data: skipping plot and overlay")
        return
    plot_imu(imu, frame_t, out_dir / f"{label}_imu.png", f"{label}: IMU")
    fps = 1000.0 / med if med else 10.0
    write_overlay(frames_iter, imu, out_dir / f"{label}_overlay.mp4", fps)


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sdir = Path(sys.argv[1])
    out_dir = sdir / "verify"
    out_dir.mkdir(exist_ok=True)
    meta = json.loads((sdir / "meta.json").read_text()) if (sdir / "meta.json").exists() else {}
    found = False

    frame_paths = sorted(((p, float(p.stem)) for p in (sdir / "frames").glob("*.jpg")), key=lambda x: x[1])
    if (sdir / "imu.csv").exists() or frame_paths:
        found = True
        frame_t = np.array([t for _, t in frame_paths])
        if (sdir / "imu.csv").exists():
            analyze("live", sdir / "imu.csv", frame_t, live_frames(frame_paths), out_dir)
        else:
            print("\n=== live ===\n  imu.csv missing")
            rate_report("frames", frame_t)

    for rec in meta.get("recordings", []):
        found = True
        tag = rec["tag"]
        video = sdir / rec["video_file"] if rec.get("video_file") else None
        if video is None or not video.exists():
            print(f"\n=== rec_{tag} ===\n  video file missing (upload incomplete?)")
            continue
        start_t = rec.get("rec_start_t") or float(tag)
        print(f"\n(rec_{tag}: video timestamps = rec_start_t {start_t:.1f} + container pts; "
              f"start source: {'MediaRecorder onstart' if rec.get('start_event_t') or rec.get('rec_start_event_t') else 'start() call'})")
        frame_t = video_frame_times(video, start_t)
        analyze(f"rec_{tag}", sdir / rec["imu_csv"], frame_t, video_frames(video, start_t), out_dir)

    if not found:
        sys.exit(f"No live data or recordings found in {sdir}")


if __name__ == "__main__":
    main()
