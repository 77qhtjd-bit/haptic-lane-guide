# HACK.md

> Shared plan for the team. Every calhacks-kit skill reads and updates this file.
> Keep it short. If it no longer fits on one screen, the scope is too big.

## Event
- Event: Cal Hacks 13.0, Palace of Fine Arts, San Francisco (Oct 23-25, 2026, 36h hacking)
- Submission deadline: 2026-10-25 11:00 PDT (PROVISIONAL, based on the 2026 AI Hackathon precedent; confirm at opening ceremony / Dashboard and update)
- Judging: expo-style table, judges walk the floor and rotate; finalists pitch on stage. Need a 30s and a 2min demo.
- Judging criteria: Application, Functionality/Quality, Creativity, Technical Complexity
- Rules to respect: no code or hardware built before the event (ideation only); all teammates on Devpost; project image + GitHub repo required; disclose open-source libs used
- Tracks we target (exact text TBA at opening ceremony, copy it here):
  - Best Hardware: <requirement>
  - Microsoft: <requirement>
  - Meta: <requirement>

## Team
| Name | Strengths | Owns |
|------|-----------|------|
| <name> | Hardware design | Haptic belt (motors, driver, 3D-printed phone chest mount, wiring) |
| <name> | Software | iPhone sensor web page, WSS server + HTTPS, laptop -> R4 UDP link, control loop, dashboard |
| <name> | Computer vision | Lane detection, gyro stabilization / filtering, error estimate |

## Problem
- One sentence: Blind runners can't train on a track on their own schedule because staying in a lane requires a sighted guide, and guides are hard to find regularly.
- The user: a blind (B1/B2) runner who wants to do weekly solo laps on a 400m track but can only run when a volunteer guide is free.
- Why it matters: UK study of visually impaired youth: hard to find volunteer guides willing to run regularly, and asking one guide repeatedly feels like a burden. Interview study (10 runners + guides): a runner left a club over an unsafe course; guide entry fees/slots are a barrier at races. Even elite athlete Simran Sharma (India, BBC) struggled to find coaches/guides. <add exact citations/links for the pitch>
- What's different about our approach: existing aids (e.g. Google Project Guideline, glasses-style devices) use audio volume/pitch, which doesn't say "how much" to correct; we use a waist haptic belt where the vibration's position = direction and its intensity = how far off the lane you are.

## MVP: the one journey
1. Runner straps on the haptic belt and the chest-mounted phone, taps one big button; one short buzz confirms "lane locked".
2. Runner moves down the lane; phone camera detects the lane lines, phone gyro + filtering removes running shake, and estimates heading + lateral offset.
3. Runner drifts right; the left-side motor(s) vibrate, stronger the farther they drift (proportional, not on/off).
4. Runner corrects; vibration fades to silence = "you're centered". Moment of value: a blindfolded judge stays inside the lane on our self-built demo track.

Done means: a blindfolded person walks our self-built demo track live, guided only by the belt, while the screen shows the lane overlay and lateral error; shaking the phone does not make the belt chatter.

Proof for judges: lateral-error RMS (cm) on the same lane with (a) no feedback, (b) audio feedback, (c) our belt, as one chart. Belt has an LED next to each motor so judges can see the feedback.

## Cut list
- Pace coaching / pace feedback: separate problem, adds a second feedback channel that confuses the haptic signal.
- Obstacle / other-runner detection: a different CV problem; demo track is empty. STRETCH only after the core loop passes /hack-check: rear-facing HC-SR04 (or TF-Luna ToF) feeding the Safety Supervisor's STOP/warn pattern. Not HC-SR501 PIR: false-triggers when the wearer moves, binary output, ~2-3s retrigger.
- Lap counting, distance, stats, history: not in the journey.
- Audio/voice feedback: our differentiator is haptic; voice only as a one-time "lane locked" at most.
- Glasses / head-mounted form factor: head moves too much; chest mount is the stable choice.
- Raspberry Pi / external display on the device: phone already has camera + gyro + compute.
- Companion app UI, login, settings, onboarding: one start button, fixed tuned parameters.
- Lane-change and track-curve handling beyond gentle curves: demo straight + slight curve only.
- Night / rain / any lighting robustness beyond the demo conditions.
- Google Maps / GPS: GPS error is meters, a lane is 1.22m; useless for lane keeping.
- All AI APIs (Gemini/Claude/Grok), in or out of the control loop: cut from the MVP (team decision, Sat). Revisit only if a track requires it after the core loop works.
- Camera EIS (preferredVideoStabilizationMode): adds latency and breaks camera-IMU geometry; we stabilize the error signal with IMU + Kalman instead.
- ML lane model (UFLD / custom YOLO-seg): only if classic CV fails on the demo track.
- Pivot to crosswalks / general walking: bigger story, more scope; one line in the pitch as "next".
- Recorded-footage replay demo: replaced by a live walk on our own demo track (record a real-track video only as backup).

## Stack
(none from the calhacks-kit catalog fit a hardware build; all picks are "not in catalog, verify")
No Mac on the team (Ubuntu laptops) -> no native iOS app.
DECISION GATE (20-min spike, first thing): Plan A = phone does everything. Open a test page in the Bluefy iOS browser (Web Bluetooth; Safari has none) and check camera (`getUserMedia`) + `DeviceMotionEvent` + BLE write to the R4 (`ArduinoBLE`) all work at once. Page hosted on GitHub Pages (free HTTPS, no mkcert). CV/Kalman/PD in plain JS on a ~160x120 canvas; develop and test the same page in Chrome on Ubuntu (webcam + Web Bluetooth). No Safari remote inspector without a Mac -> on-page log panel. If any of the three fails, use Plan B below.
Plan B (below): the iPhone is only a sensor, through a web page; the laptop decides.
CV library: OpenCV (OpenCV.js in Plan A, opencv-python in Plan B).
- iPhone Safari sensor page (one HTML file served by the laptop): `getUserMedia` camera frames downscaled to ~320x240 JPEG + `DeviceMotionEvent` (rotationRate, accelerationIncludingGravity) + `DeviceOrientationEvent`, timestamped, sent over a WebSocket. Needs HTTPS and `DeviceMotionEvent.requestPermission()` on a button tap.
- HTTPS on the LAN: `mkcert` local CA on the laptop; install its root CA profile on the iPhone and enable full trust (Settings > General > About > Certificate Trust Settings).
- Ubuntu laptop Python server: `websockets` (or FastAPI) receives frames + IMU; OpenCV + NumPy do lane perception (HSV white mask -> ground-plane projection of points with attitude -> RANSAC); hand-written Kalman + PD + haptic mapper. Laptop screen = live dashboard for judges (lane overlay, e_y plot, motor levels) and CSV log for the RMS chart.
- Laptop -> UNO R4 WiFi over Wi-Fi UDP (`WiFiS3` library on the R4), 30Hz packets of [m1..m5] levels. Fallback: BLE via `bleak` on Ubuntu + `ArduinoBLE` on the R4.
- Network: one private network we control, not venue Wi-Fi: iPhone Personal Hotspot with "Maximize Compatibility" on (2.4GHz for the R4), laptop and R4 join it.
- Motors: our own DC motors + 3D-printed eccentric masses (home-made ERM) in printed enclosures, driven by logic-level MOSFETs + flyback diodes on PWM pins 3/5/6/9/10 (ULN2003 only if each motor < ~300mA), separate 5V power bank, LED per motor.
- Riskiest integration: end-to-end loop (iPhone -> Wi-Fi -> laptop CV -> UDP -> motor spin-up) fast and stable enough: e_y jitter < ±10cm while jogging, glass-to-motor latency < ~200ms. Spike result: pending

Architecture:
```
iPhone (Safari, chest mount)                Ubuntu laptop (at the table)                     Belt
camera ~30fps JPEG ─┐                       lane points -> ground plane (attitude)
IMU ~60Hz ──────────┴─ WSS over hotspot ──> -> RANSAC -> Kalman -> PD -> haptic map ── UDP ──> UNO R4 WiFi
                                            dashboard (overlay, e_y, motors) + CSV            -> MOSFETs -> DC motors
                                                                                                 + 3D-printed masses + LEDs
```

## Honesty
Mocked, hardcoded, or seeded parts we will disclose in the demo:
- Demo is walking on a self-built indoor track, not running on a real 400m track.
- Control gains and deadband are hand-tuned for the demo track and walking speed.
- Backup video (if shown) was recorded during the event.

## Pitch
- One-liner: <Product> helps blind runners train alone on a track by turning lane position into a directional, proportional vibration on a waist belt.
- Roles: driver <name>, narrator <name>, tech answers <name>, backup <name>

## Status log
- 16:49 Fri Scoped MVP. Next: confirm deadline + track requirements at opening ceremony, run /hack-stack, spike gyro-stabilized lane detection first.
- 17:20 Fri Chose control pipeline: IMU-compensated bird's-eye lane detection -> Kalman (e_y, e_psi) -> PD + heading + deadband -> haptic mapper (side = direction, pulse rate = magnitude) + safety STOP. LLMs only in slow loop, never in control loop. Reference: PulseWalk (Cal Hacks 11.0 Best Hardware winner). Demo plan deferred; team will build a demo track.
- 18:30 Fri Re-scoped MVP: live walk on self-built demo track + RMS proof chart; cut Maps, in-loop LLM, EIS, ML lane model, pivot. ~40h to provisional deadline. Next: /hack-stack.
- 19:30 Fri Stack chosen: Swift/iPhone (AVFoundation + CoreMotion + CoreBluetooth) -> UNO R4 WiFi (ArduinoBLE) -> ULN2003 -> ERM motors. No API keys in MVP. Next: spike e_y stability (vision) and BLE->motor latency (hardware) in parallel.
- 20:00 Fri Moved HACK.md to new repo haptic-lane-guide. No Mac -> replaced native iOS app with iPhone Safari sensor page -> Ubuntu Python/OpenCV server -> UDP -> UNO R4 WiFi; home-made ERM motors (DC motor + printed mass) via MOSFETs. Next: spike end-to-end latency + e_y jitter, and motor spin-up time.
- Sat BLE image + IMU transfer working. Design: phone runs OpenCV; IMU gravity -> per-frame ground projection; white-pixel lateral histogram in ground frame finds parallel lines; lane LOCK at start keeps the two lines around the runner by Kalman tracking (lane identity by continuity, not re-detection); centerline = mid of L/R (or one line +/- 0.61m). Motors 1/5 = big turn, 2/4 = small turn, 3 = straight heartbeat; R4 runs pulse timing + 300ms watchdog. BLE carries motor commands only. AI APIs cut. Step 1 started: OpenCV.js ground projection + lateral histogram peaks (web/).
- Sat Step 1 code done: web/lane/ground.js (geometry, 8 node tests pass), web/lane/detector.js (OpenCV.js mask -> peaks; synthetic frames at 3 poses recover lines within 0.2cm), web/spike-ground.html (phone spike page with jitter readout). Finding: portrait 1x camera (~53 deg) sees only the runner's own two lines up close (+-0.7m at 1m); outer lanes only beyond ~3m. Next: real-phone jitter test on taped lane, then lane lock + tracking.
