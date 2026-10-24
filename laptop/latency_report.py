"""Summarize an actuator log: UDP RTT and the phone-timestamp -> ACK latency breakdown.

Usage: python latency_report.py [data/actuator/<time>.csv]   (default: newest log)
       python latency_report.py data/ble/<time>.csv          (integrated frame -> laptop -> phone -> BLE log)
"""
import csv
import sys
from pathlib import Path

import numpy as np


def stats(name, x):
    x = np.asarray(x, dtype=float)
    if len(x) == 0:
        print(f"  {name:38s} no data")
        return
    print(f"  {name:38s} median {np.median(x):7.1f}  p95 {np.percentile(x, 95):7.1f}  "
          f"min {x.min():7.1f}  max {x.max():7.1f} ms  (n={len(x)})")


def main():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        logs = sorted((Path(__file__).parent.parent / "data" / "actuator").glob("*.csv"))
        if not logs:
            sys.exit("no logs in data/actuator/")
        path = logs[-1]
    rows = list(csv.DictReader(open(path)))
    f = lambda r, k: float(r[k]) if r.get(k) else None
    if rows and "frame_t" in rows[0]:
        return integrated_report(path, rows, f)
    acked = [r for r in rows if r["rtt_ms"]]
    print(f"{path}: {len(rows)} commands, {len(acked)} ACKed, {len(rows) - len(acked)} lost "
          f"({100 * (len(rows) - len(acked)) / max(1, len(rows)):.1f}%)")
    if not rows:
        return
    stats("UDP round trip (send -> ACK)", [f(r, "rtt_ms") for r in acked])

    tilt = [r for r in acked if r.get("imu_t_server")]
    print(f"\nTilt-mode breakdown ({len(tilt)} commands with an IMU sample and a clock offset):")
    if tilt:
        offs = [f(r, "clock_offset_ms") for r in tilt]
        print(f"  clock offset (server - phone): {np.median(offs):.0f} ms, "
              f"range {min(offs):.0f} .. {max(offs):.0f} ms")
        stats("1. phone sample -> server receive", [f(r, "server_recv_t") - f(r, "imu_t_server") for r in tilt])
        stats("2. server receive -> UDP send", [f(r, "udp_send_t") - f(r, "server_recv_t") for r in tilt])
        stats("3. UDP send -> board (RTT/2 estimate)", [f(r, "rtt_ms") / 2 for r in tilt])
        stats("phone sample -> board (1+2+3)", [f(r, "udp_send_t") + f(r, "rtt_ms") / 2 - f(r, "imu_t_server") for r in tilt])
        stats("phone sample -> ACK received", [f(r, "ack_t") - f(r, "imu_t_server") for r in tilt])
        print("  note: the same IMU sample can be reused by several 20 Hz commands until the next batch\n"
              "  arrives (the phone sends IMU every 100 ms), which shows up in step 2.")


def integrated_report(path, rows, f):
    """BLE log from the phone page. Phone clock: frame_t, cmd_ready_t, ble_write_t, ack_t.
    Laptop rows also have server-clock srv_recv_t/srv_send_t and offset = server - phone."""
    key = lambda r: ((r.get("mode") or "laptop") + (f" [{r['ts_src']}]" if r.get("ts_src") else "")
                     + (f" @ live {r['live_fps']} fps" if r.get("live_fps") else ""))
    for mode in sorted({key(r) for r in rows}):
        rs = [r for r in rows if key(r) == mode]
        for r in rs:
            r.setdefault("cmd_ready_t", r.get("cmd_recv_t"))
        acked = [r for r in rs if r["rtt_ms"]]
        t = [f(r, "ble_write_t") for r in rs]
        dur = (max(t) - min(t)) / 1000 if len(t) > 1 else 0
        print(f"\n=== {mode}: {len(rs)} BLE writes over {dur:.1f} s ({len(rs) / max(dur, 1e-9):.1f}/s), "
              f"{len(acked)} ACKed, {len(rs) - len(acked)} lost")
        if not acked:
            continue
        if mode.startswith("phone"):
            stats("1. frame capture -> processing done", [f(r, "cmd_ready_t") - f(r, "frame_t") for r in acked])
            stats("   of which frame grab + decide()", [f(r, "proc_ms") for r in acked])
        else:
            with_off = [r for r in acked if r.get("offset")]
            if with_off:
                stats("1a. frame capture -> server receive", [f(r, "srv_recv_t") - f(r, "offset") - f(r, "frame_t") for r in with_off])
            stats("1b. server processing (recv -> send)", [f(r, "srv_send_t") - f(r, "srv_recv_t") for r in acked])
            if any(r.get("proc_ms") for r in acked):
                stats("    of which JPEG decode + decide()", [f(r, "proc_ms") for r in acked if r.get("proc_ms")])
            if with_off:
                stats("1c. server send -> phone receive", [f(r, "cmd_ready_t") - (f(r, "srv_send_t") - f(r, "offset")) for r in with_off])
            stats("1. frame capture -> command on phone", [f(r, "cmd_ready_t") - f(r, "frame_t") for r in acked])
        stats("2. command -> BLE write", [f(r, "ble_write_t") - f(r, "cmd_ready_t") for r in acked])
        stats("3. BLE write -> board (RTT/2)", [f(r, "rtt_ms") / 2 for r in acked])
        stats("TOTAL frame capture -> board", [f(r, "ble_write_t") + f(r, "rtt_ms") / 2 - f(r, "frame_t") for r in acked])
        stats("BLE RTT", [f(r, "rtt_ms") for r in acked])
        imu = [r for r in acked if r.get("imu_t")]
        if imu:
            stats("IMU sample age at frame (frame_t - imu_t)", [f(r, "frame_t") - f(r, "imu_t") for r in imu])
    print("\nnote: frame_t comes from requestVideoFrameCallback; earlier sessions showed it can lag the real\n"
          "capture by ~50-70 ms, so true totals may be larger by about that much (same for both modes).")


if __name__ == "__main__":
    main()
