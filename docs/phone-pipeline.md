# Phone → BLE → belt pipeline (Plan A, measured)

Result of the transport spikes on Oct 24. Summary: **Plan A works.** The phone (Bluefy) runs camera +
IMU + `decide()` and writes motor commands over BLE to the UNO R4. No laptop or network is needed at
runtime.

## Architecture
```
iPhone, Bluefy browser (chest mount)                         UNO R4 WiFi (belt)
camera 640x480 @30 ─┐
IMU ~60 Hz ─────────┴─> decide(pixels, w, h, imu) -> 5 levels ──BLE──> pulse timing per motor -> M1..M5
                                                 <────── ACK(seq) ─────  300 ms watchdog
```

## Measured (iPhone, iOS 18.7, Bluefy 3.9.3)
| | median | p95 | notes |
|---|---|---|---|
| Frame capture -> decide() returns (480x640 getImageData + empty decide) | 17 ms | 26 ms | |
| BLE write -> board (RTT/2) | ~28 ms | ~40 ms | BLE RTT 55 / 72 ms |
| **Frame capture -> board** | **46 ms** | **60 ms** | loss 1/1307; add ~20 ms for rVFC timestamp lag (from recorded clips) |
| Laptop decides instead (frame -> tunnel -> laptop -> tunnel -> phone -> BLE) | 0.5-30 s | | venue Wi-Fi congested; rejected |
| Laptop -> R4 Wi-Fi UDP over iPhone hotspot | ~330 ms RTT | ~850 ms | ~20% loss; rejected |

Budget for the lane algorithm: keep `decide()` under ~20 ms per frame to stay near 70 ms total.

**Open issue: IMU stalls.** With the camera running, iOS sometimes stops delivering `devicemotion`
for up to ~2 s, in bursts (seen after ~80 s of camera use). The page keeps running during the stall
(camera frames and BLE continue), so it is OS/browser-side. Treat IMU as possibly stale: `decide()`
gets the sample with its timestamp (`imu.t`); ignore it when it is older than ~100 ms.
Recorded clips (52 s, 75 s) had no stalls; longer clips should confirm.

## BLE protocol (team plan: 5 motors)
- Service `06096ed2-b23b-4ba5-9737-27dba2053749`, peripheral name `LaneActuator`.
- Command characteristic `14dd8746-dd9e-4308-8f98-08701035fc46` (write without response),
  7 bytes: `[seq lo, seq hi, L1, L2, L3, L4, L5]`.
  - M1 big left, M2 small left, M3 straight heartbeat, M4 small right, M5 big right (pins D3, D5, D6, D9, D10).
  - Level per motor: 0 = off, 255 = continuous, 1..254 = 60 ms pulses, period 1000 ms (level 1) -> 120 ms (level 254).
    The R4 generates the pulse timing; the phone only sends levels.
- ACK characteristic `535f34b1-32f0-4532-97d1-18f2fff4bfbe` (notify): 2-byte seq echo, for RTT.
- Watchdog: no valid command for 300 ms, or disconnect -> all motors off. The phone sends at ~15-20 Hz.
- Assumption to confirm with the team: the level-to-pulse mapping above (constants at the top of
  `firmware/lane_actuator_ble/lane_actuator_ble.ino`).

## Files
| Path | What |
|---|---|
| `web/runner-test.html` | Final-architecture test page: camera + IMU + `decide()` placeholder + BLE with auto-reconnect; shows latency, IMU gaps, BLE stats |
| `web/record.html` | Offline recorder for algorithm development: video + full-rate IMU + per-frame timestamps, saved on the phone (Bluefy works too) |
| `firmware/lane_actuator_ble/` | R4 BLE firmware (5 motors, pulse timing, watchdog). Upload steps (Korean): `firmware/HANDOFF.md` |
| `firmware/lane_actuator/` | Old Wi-Fi/UDP firmware (rejected path, kept for reference) |
| `laptop/` | FastAPI server used for the spikes (`uvicorn server:app --app-dir laptop --host 0.0.0.0 --port 8000`), `/import` for recorded clips, `verify.py` (IMU/frame stats, IMU overlay video), `latency_report.py` |

`decide()` lives in `web/runner-test.html`. It receives RGBA pixels (480x640), width, height and the latest
IMU sample `{t, gx, gy, gz}` and returns 5 levels. The placeholder steps through M1..M5 once per second.

Legacy (not compatible with the 5-motor firmware): `laptop/static/index.html` BLE relay and
`laptop/static/ble.html` still send the old 4-byte `[seq, left, right]` command.

## Recording workflow (for algorithm development)
1. Open `web/record.html` (GitHub Pages URL), Start, Record, tap the phone once (sync marker), run, Stop, Save to phone.
2. At the laptop: open `http://<laptop>/import`, upload the `.mp4` + `.json` -> `data/import-<time>/`.
3. `python laptop/verify.py data/import-<time>` -> IMU plot + overlay video. Video/IMU alignment measured at ~20-30 ms, no drift.

## Hosting
GitHub Pages (personal project site, branch `phone-ble-pipeline`, root): `https://sunuk00.github.io/haptic-lane-guide/web/record.html`
and `.../web/runner-test.html`. HTTPS is required for camera, motion and Web Bluetooth.
