# Feasibility test results

Date: ________   Tester: ________
iPhone model / iOS version: ________________
Laptop / Ubuntu version: ________________
Tunnel used: [ ] cloudflared  [ ] ngrok    Network: ________________ (Wi-Fi / cellular)

## Live mode (Safari)
| Metric | Target | Measured |
|---|---|---|
| IMU (devicemotion) rate, terminal | ≥ 50 Hz | ____ Hz |
| deviceorientation rate | — | ____ Hz |
| Frame rate received at 640x480 (selected: __ fps) | ≥ 10 fps | ____ fps |
| Actual track settings (width x height @ fps) | 640x480 | ________ |
| Upload bandwidth | — | ____ kB/s |
| RTT (typical / worst) | — | ____ / ____ ms |
| Frames skipped by client (encode/backpressure) | — | ____ /s |

### verify.py (live session `data/________`)
| Metric | Value |
|---|---|
| IMU Hz mean / min / max | ____ / ____ / ____ |
| IMU gaps > 50 ms | ____ |
| IMU timestamps monotonic | yes / no |
| Frame fps mean / min / max | ____ / ____ / ____ |
| Frame gaps | ____ |
| Overlay: IMU motion visibly aligned with video motion? | yes / no, offset ≈ ____ ms |

## Record mode (30 s)
| Metric | Value |
|---|---|
| MediaRecorder mimeType | ________ |
| Video size / duration | ____ MB / ____ s |
| Upload succeeded | yes / no (error: ________) |
| IMU samples logged / Hz | ____ / ____ Hz |
| Video frame rate (verify.py) | ____ fps |
| Overlay alignment (onstart-based) | aligned / offset ≈ ____ ms |

## Cameras (enumerateDevices labels)
| # | Label | Settings when selected (w x h @ fps) |
|---|---|---|
| 1 | ________ | ________ |
| 2 | ________ | ________ |
| 3 | ________ | ________ |
| 4 | ________ | ________ |

## API availability
| API | Safari | Bluefy |
|---|---|---|
| getUserMedia | | |
| DeviceMotionEvent | | |
| DeviceMotionEvent.requestPermission | | |
| DeviceOrientationEvent | | |
| wakeLock (works when requested?) | | |
| requestVideoFrameCallback | | |
| MediaRecorder (+ mimeType) | | |
| navigator.bluetooth | | |
| secureContext | | |

## Failures / notes
- ________________________________________________
