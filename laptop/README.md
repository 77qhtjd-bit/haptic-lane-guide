# iPhone → Ubuntu camera + IMU feasibility test

Safari on the iPhone streams rear-camera JPEG frames and IMU data to a Python server on
the Ubuntu laptop over an HTTPS tunnel (same-origin `wss://`). There is also a record mode
that uploads a MediaRecorder video plus a full-rate IMU log.

## Files
- `server.py`: FastAPI server. Serves `/` (phone page), `/viewer` (laptop viewer), `/ws`, `/upload`
- `static/index.html`: phone client (plain JS)
- `static/viewer.html`: optional laptop viewer (latest frame + latest IMU values)
- `environment.yml`: conda environment (pip packages come from `requirements.txt`)
- `verify.py`: offline check of a session (rates, gaps, plot, overlay video)
- `data/<session_id>/`: one directory per WebSocket connection

## 1. One-time setup (Ubuntu)

```bash
cd ~/CalHacks13.0
conda env create -f laptop/environment.yml      # creates env "calhacks-sensor" (Python 3.12, ffmpeg, pip deps)
# later dependency changes: conda env update -f laptop/environment.yml

# cloudflared (quick tunnel, no account needed). Not on pip/conda: install the binary into the env.
curl -fL -o ~/miniconda3/envs/calhacks-sensor/bin/cloudflared \
  https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64
chmod +x ~/miniconda3/envs/calhacks-sensor/bin/cloudflared
```

## 2. Run

Terminal 1, the server:
```bash
cd ~/CalHacks13.0
conda activate calhacks-sensor
uvicorn server:app --app-dir laptop --host 0.0.0.0 --port 8000
```

Terminal 2, the tunnel:
```bash
cloudflared tunnel --url http://localhost:8000
```
It prints a URL like `https://random-words.trycloudflare.com`.

Fallback if cloudflared does not work: `ngrok http 8000` (needs a free ngrok account and authtoken),
then use the `https://...ngrok-free.app` URL. You may have to tap through ngrok's warning page once.

## 3. On the iPhone

1. Open the `https://....trycloudflare.com` URL in **Safari**.
2. Tap **Start**. Allow *Motion & Orientation* and *Camera* when asked.
3. Check the status rows: IMU Hz, Frames sent, WebSocket open, RTT.
4. Optionally pick another camera from the **Camera** dropdown, and change the **Live frame rate** (5/10/15/off).
5. For record mode, tap **Record**, wait about 30 s, then tap **Stop recording**. The page uploads the
   video and then the IMU log, and shows "uploaded ..." when both are done.

Terminal 1 prints one line per second:
```
[20261024-120000-123] IMU  60 Hz | orient  60 Hz | frames 10 fps |   250.3 kB/s | RTT 85 ms
```

Optional: open `http://localhost:8000/viewer` on the laptop to see the latest frame and IMU values.

### iOS notes
- If you denied motion or camera access once, Safari remembers that choice for the origin.
  To reset it: Settings → Apps → Safari → Advanced → Website Data, then delete the trycloudflare entry
  (or start a new tunnel, which gives a new origin).
- Low Power Mode can lower the camera and sensor rates.
- iOS reports `rotationRate` in deg/s. `interval` is logged exactly as reported. Older iOS versions
  reported it in seconds, not ms.
- Each WebSocket (re)connection creates a new session directory.

## 4. Verify a session

```bash
conda activate calhacks-sensor
python laptop/verify.py data/<session_id>
```
It prints the IMU rate (mean/min/max Hz per 1 s window), the number of gaps over 50 ms,
timestamp monotonicity, and the frame rate and gaps. It writes these files to `data/<session_id>/verify/`:
- `live_imu.png` / `rec_<tag>_imu.png`: accelerationIncludingGravity and rotationRate over time (frame times shown as ticks)
- `live_overlay.mp4` / `rec_<tag>_overlay.mp4`: frames with the nearest IMU sample overlaid
  (roll/pitch from gravity, gyro, and the IMU-to-frame time offset). Encoded as `mp4v`. Use VLC or mpv if a browser will not play it.

## Data format
- Clock: every timestamp is `performance.timeOrigin + performance.now()` on the phone (ms, Unix epoch).
- `frames/<t>.jpg`: t is the frame timestamp. It comes from `requestVideoFrameCallback` (`captureTime` if
  available, otherwise the callback's `now`), or from `requestAnimationFrame` as a fallback.
- WebSocket binary frame: 8-byte little-endian float64 timestamp, followed by the JPEG bytes.
- `imu.csv`: one row per sample. `kind` = `motion` or `orientation`, followed by all fields. Fields that do not apply are empty.
- `meta.json`: user agent, API availability, permissions, camera list, track settings, camera switches, recordings.
- Record mode: `rec_<tag>.mp4` (or `.webm`), `rec_<tag>_imu.json` (raw upload), `rec_<tag>_imu.csv`.
  Video time = `rec_start_t` + container timestamp. `rec_start_t` is the MediaRecorder `start` event time,
  or the `start()` call time if the event did not fire. The overlay video shows how good this alignment is.

---

# Actuator link: laptop → Wi-Fi (UDP) → Arduino UNO R4 WiFi

The laptop sends `<seq>,<left>,<right>\n` (0–255) to the board on UDP 4210 at 20 Hz. The board
replies `ACK,<seq>\n`, drives PWM pins 9/10 (future vibration motors) and shows the command on the
12x8 LED matrix. If no valid command arrives for 500 ms, the board sets its outputs to 0 and shows "no signal".
The sketch and the upload instructions (Korean) are in `arduino_handoff/` (also `arduino_handoff.zip`).
The board is flashed from another computer.

Files: `actuator.py` (UDP sender, HELLO listener, `error_to_command()`), `static/control.html` (`/control`),
`latency_report.py`, `arduino_handoff/lane_actuator/lane_actuator.ino`.

## Stage 0: board preparation
- Core `arduino:renesas_uno` (Arduino IDE 2 → Boards Manager → "Arduino UNO R4 Boards", or
  `arduino-cli core install arduino:renesas_uno`). The libraries `WiFiS3` and `Arduino_LED_Matrix` ship with this core.
- Linux serial permission (only on the computer that uploads): `sudo usermod -aG dialout $USER`, then log out and back in.
- Wi-Fi module firmware: at boot the sketch prints `WiFi.firmwareVersion()` and `WIFI_FIRMWARE_LATEST_VERSION`.
  If the installed version is older, update it with Arduino IDE 2 → Tools → Firmware Updater. Do not flash custom firmware to the ESP32-S3.
- Wi-Fi credentials go in `arduino_secrets.h` (copied from `arduino_secrets.h.example`). This file is listed in `.gitignore`.

## LED matrix screens
| Screen | Meaning |
|---|---|
| dotted bottom row only | booting / connecting to Wi-Fi |
| digits + dotted bottom row | no signal. The digits are the last octet of the board IP (e.g. `5` → 172.20.10.5) |
| arrow ◀ / ▶ + bars on the bottom rows | left / right command. Bar length = intensity |
| small square in the center | valid command with left = right = 0 (stop) |

## Run order
1. **Server**: `conda activate calhacks-sensor && cd ~/CalHacks13.0 && uvicorn server:app --app-dir laptop --host 0.0.0.0 --port 8000`
2. **Tunnel**: `cloudflared tunnel --url http://localhost:8000`. Get the current URL with `curl -s http://127.0.0.1:20241/quicktunnel`.
3. **Hotspot**: turn on the iPhone Personal Hotspot with **Maximize Compatibility ON** (the R4 supports 2.4 GHz only),
   then connect the laptop to it. The phone itself still reaches the tunnel over cellular.
4. **Arduino**: power it over USB. Within a few seconds `/control` (`http://localhost:8000/control`) shows the board IP
   (source `hello`). If it does not appear, read the last octet on the matrix, type the full IP
   (hotspot subnet `172.20.10.x`) into the manual IP box, and press Set IP.
5. **Phone**: open the tunnel URL in Safari and tap Start. In `/control`, choose **Tilt demo** and tilt the phone left/right
   (held upright in portrait).

## Stage 3: network checks
- Event Wi-Fi often blocks traffic between clients. Use the iPhone hotspot for the laptop and the board.
- Ping the board: `ping -c 5 <board-ip>`
- Check for HELLO broadcasts (stop the server first, because it owns port 4211):
  `python3 -c "import socket;s=socket.socket(2,2);s.bind(('',4211));print(s.recvfrom(64))"`
- Send one command manually: `echo "1,200,0" | nc -u -w1 <board-ip> 4210`. The reply should be `ACK,1`, and the matrix should show ◀.
- If `ufw` is active, allow HELLO: `sudo ufw allow 4211/udp` (ACK replies are allowed as return traffic).
- `/control` shows ACKs / lost. If lost keeps rising with zero ACKs, UDP is not reaching the board (or the replies are not
  coming back). Report this. Do not work around it.

## Latency report
`python laptop/latency_report.py` (newest `data/actuator/*.csv`) prints the UDP RTT median/p95 and, for tilt mode, these stages:
phone IMU timestamp → server receive → UDP send → board (RTT/2) → ACK. Phone timestamps are converted to the server clock
with the offset `server_time − (phone_send_time + RTT/2)` from the existing ping/echo (median of the last 10 pings).

Tilt demo mapping (`actuator.py`): `roll_from_gravity()` gives degrees, positive = tilted left. `error_to_command()` has a
±8° deadband and 2° hysteresis (on above 8°, off below 6°), and intensity = 255·|error|/30° (capped at 255).
Positive error → LEFT. Replace the roll with the lane offset later. Only `error_to_command()` defines the mapping.

## Stage 4: motor wiring (not implemented yet)
**Vibration module with an onboard driver (3 pins: VCC/GND/SIG):** connect VCC to 5V, GND to GND, and SIG to D9 (left) or D10 (right).

**Bare motor (2 wires): never connect it directly to a GPIO pin.** A pin can supply only a few mA, and the motor's
inductive kick can destroy the pin. Use a transistor or logic-level MOSFET, a flyback diode, and 5 V power, one set per motor:
```
 5V ──────────┬──────────────┐
              │              │
           [MOTOR]       diode 1N4148/1N4007
              │          (cathode/stripe to 5V)
              ├──────────────┘
              │
            Drain (or Collector)
 D9 ──[220Ω]── Gate   MOSFET e.g. AO3400 / IRLZ44N   (NPN 2N2222: base via 1kΩ)
              │  (10kΩ Gate→GND pull-down)
            Source (or Emitter)
              │
 GND ─────────┴──── Arduino GND (common ground)
```
For larger motors use a separate 5 V supply, and connect its GND to the Arduino GND.
