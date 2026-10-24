"""Laptop -> Arduino UNO R4 WiFi actuator link over UDP.

- Sends the latest (left, right) command as "<seq>,<left>,<right>\\n" to the board (UDP 4210) at 20 Hz.
- Logs every ACK with round-trip time and the latency breakdown to data/actuator/<time>.csv.
- Listens for "HELLO,<ip>\\n" broadcasts on UDP 4211 to discover the board's IP.
- Tilt demo: phone roll (from accelerationIncludingGravity) -> command via error_to_command().
"""
import asyncio
import csv
import math
import statistics
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

CMD_PORT = 4210
HELLO_PORT = 4211
SEND_HZ = 20
ACK_TIMEOUT_S = 1.0
IMU_STALE_MS = 500        # tilt mode sends 0/0 if the newest IMU sample is older than this

# error_to_command parameters
DEADBAND_DEG = 8.0
HYSTERESIS_DEG = 2.0
FULL_SCALE_DEG = 30.0     # |error| at which intensity reaches 255

LOG_DIR = Path(__file__).parent.parent / "data" / "actuator"
LOG_COLUMNS = [
    "seq", "mode", "left", "right",
    "imu_t_phone",        # phone timestamp of the IMU sample used (ms, phone clock); tilt mode only
    "clock_offset_ms",    # server clock - phone clock estimate at send time
    "imu_t_server",       # imu_t_phone + clock_offset_ms (ms, server clock)
    "server_recv_t",      # server time the IMU batch containing that sample arrived (ms)
    "udp_send_t",         # server time the UDP command was sent (ms)
    "ack_t",              # server time the ACK arrived (ms), empty if lost
    "rtt_ms",             # ack_t - udp_send_t
]


def now_ms():
    return time.time() * 1000.0


def roll_from_gravity(gx, gy):
    """Left/right tilt of a phone held upright in portrait (degrees, positive = tilted left).

    Measured on iOS Safari: upright portrait gives accg_y ~ -9.8 and tilting the left edge
    down makes accg_x negative. Flip the sign here if the demo reacts the wrong way.
    """
    return math.degrees(math.atan2(-gx, -gy))


def error_to_command(error, active):
    """Map a signed error to a motor command.

    error: degrees (positive = cue LEFT, negative = cue RIGHT). Later this becomes the
           lane offset; only this function needs to change.
    active: side currently on ('left', 'right' or None), for hysteresis.
    Returns (left, right, active).
    Turns on when |error| > DEADBAND, turns off when |error| < DEADBAND - HYSTERESIS.
    Intensity is proportional to |error|, 255 at FULL_SCALE_DEG.
    """
    mag = abs(error)
    side = "left" if error > 0 else "right"
    if active is not None and side != active:
        active = None                          # crossed zero: start over
    if active is None and mag > DEADBAND_DEG:
        active = side
    elif active is not None and mag < DEADBAND_DEG - HYSTERESIS_DEG:
        active = None
    if active is None:
        return 0, 0, None
    v = int(round(min(255.0, 255.0 * mag / FULL_SCALE_DEG)))
    return (v, 0, active) if active == "left" else (0, v, active)


def decide(image, imu):
    """PLACEHOLDER for the decision algorithm (laptop side). No image analysis yet.

    image: decoded BGR frame (numpy array); imu: latest devicemotion sample dict or None.
    Returns (left, right) 0..255. For now: 1 s left / 1 s right so the output is visible.
    Same placeholder as decide() in static/index.html.
    """
    return (150, 0) if int(time.time()) % 2 == 0 else (0, 150)


class _CmdProtocol(asyncio.DatagramProtocol):
    def __init__(self, owner):
        self.owner = owner

    def datagram_received(self, data, addr):
        self.owner._on_ack(data, addr)


class _HelloProtocol(asyncio.DatagramProtocol):
    def __init__(self, owner):
        self.owner = owner

    def datagram_received(self, data, addr):
        self.owner._on_hello(data, addr)


class Actuator:
    def __init__(self):
        self.target_ip = None
        self.target_source = None         # 'hello' or 'manual'
        self.hello_last = None            # (ip, server time ms)
        self.mode = "manual"              # 'manual' | 'tilt'
        self.manual = (0, 0)
        self.left = self.right = 0
        self.tilt_active = None
        self.roll = None
        self.frame_cmd = (0, 0)
        self.seq = 0
        self.phone_seq = 0                # 16-bit seq for the WebSocket -> phone -> BLE path
        self.pending = {}                 # seq -> log row
        self.rtts = deque(maxlen=200)
        self.acks = 0
        self.lost = 0
        # IMU input (read-only hook from server.py)
        self.imu = None                   # (sample dict, server_recv_t)
        self.offsets = deque(maxlen=10)   # recent clock offset estimates
        self._transport = None
        self._log_file = None
        self._log = None
        self.log_path = None

    # ---------- inputs from server.py ----------
    def on_imu(self, samples, recv_t):
        for s in reversed(samples):
            if s.get("k") == "m":
                self.imu = (s, recv_t)
                return

    def on_ping(self, phone_t, last_rtt, recv_t):
        """offset = server_time - (phone send time + RTT/2), median of recent estimates."""
        if phone_t is not None and last_rtt is not None:
            self.offsets.append(recv_t - (phone_t + last_rtt / 2.0))

    def clock_offset(self):
        return statistics.median(self.offsets) if self.offsets else None

    # ---------- control ----------
    def set_target(self, ip, source):
        if ip != self.target_ip:
            print(f"[actuator] target {ip} ({source})", flush=True)
        self.target_ip = ip
        self.target_source = source

    def set_manual(self, left, right):
        self.mode = "manual"
        self.manual = (max(0, min(255, int(left))), max(0, min(255, int(right))))

    def set_mode(self, mode):
        self.mode = mode
        self.tilt_active = None
        self.frame_cmd = (0, 0)

    # ---------- network ----------
    async def start(self):
        loop = asyncio.get_running_loop()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.log_path = LOG_DIR / (datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv")
        self._log_file = open(self.log_path, "w", newline="")
        self._log = csv.DictWriter(self._log_file, fieldnames=LOG_COLUMNS)
        self._log.writeheader()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: _CmdProtocol(self), local_addr=("0.0.0.0", 0))
        try:
            await loop.create_datagram_endpoint(lambda: _HelloProtocol(self), local_addr=("0.0.0.0", HELLO_PORT))
        except OSError as e:
            print(f"[actuator] cannot listen for HELLO on UDP {HELLO_PORT}: {e}", flush=True)
        asyncio.create_task(self._send_loop())
        print(f"[actuator] logging to {self.log_path}", flush=True)

    def _compute(self):
        """Returns (left, right, imu_sample, server_recv_t) for the current mode."""
        if self.mode == "manual":
            return self.manual[0], self.manual[1], None, None
        if self.mode == "frame":
            return self.frame_cmd[0], self.frame_cmd[1], None, None
        if self.imu is None:
            self.roll = None
            return 0, 0, None, None
        s, recv_t = self.imu
        if s.get("gx") is None or s.get("gy") is None or now_ms() - recv_t > IMU_STALE_MS:
            self.roll = None
            return 0, 0, None, None
        self.roll = roll_from_gravity(s["gx"], s["gy"])
        left, right, self.tilt_active = error_to_command(self.roll, self.tilt_active)
        return left, right, s, recv_t

    def decide_frame(self, jpeg):
        """'frame' mode: decode the frame and call decide(). Returns processing time (ms)."""
        t0 = time.perf_counter()
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return None
        self.frame_cmd = decide(img, self.imu[0] if self.imu else None)
        return (time.perf_counter() - t0) * 1000

    def phone_command(self, frame_t, recv_t, jpeg):
        """Command sent back to the phone over the WebSocket when a frame arrives (BLE relay path).

        frame_t: phone capture timestamp of the frame (ms, phone clock); recv_t: server receive time.
        """
        proc_ms = self.decide_frame(jpeg) if self.mode == "frame" else None
        left, right, s, _ = self._compute()
        if s is None and self.imu:
            s = self.imu[0]
        self.phone_seq = (self.phone_seq + 1) & 0xFFFF
        return {"type": "cmd", "seq": self.phone_seq, "mode": self.mode, "left": left, "right": right,
                "proc_ms": proc_ms, "frame_t": frame_t, "imu_t": s["t"] if s else None,
                "srv_recv_t": recv_t, "srv_send_t": now_ms(), "offset": self.clock_offset()}

    async def _send_loop(self):
        period = 1.0 / SEND_HZ
        next_t = time.monotonic()
        while True:
            next_t += period
            self.left, self.right, s, recv_t = self._compute()
            if self.target_ip:
                self._send(s, recv_t)
            self._expire()
            await asyncio.sleep(max(0.0, next_t - time.monotonic()))

    def _send(self, s, recv_t):
        self.seq += 1
        off = self.clock_offset()
        row = {"seq": self.seq, "mode": self.mode, "left": self.left, "right": self.right}
        if s is not None:
            row["imu_t_phone"] = f"{s['t']:.3f}"
            row["server_recv_t"] = f"{recv_t:.3f}"
            if off is not None:
                row["clock_offset_ms"] = f"{off:.1f}"
                row["imu_t_server"] = f"{s['t'] + off:.3f}"
        t = now_ms()
        row["udp_send_t"] = f"{t:.3f}"
        try:
            self._transport.sendto(f"{self.seq},{self.left},{self.right}\n".encode(), (self.target_ip, CMD_PORT))
        except OSError as e:
            print(f"[actuator] send failed: {e}", flush=True)
            return
        self.pending[self.seq] = (t, row)

    def _on_ack(self, data, addr):
        t = now_ms()
        try:
            text = data.decode().strip()
            tag, seq = text.split(",")
            seq = int(seq)
        except ValueError:
            return
        if tag != "ACK" or seq not in self.pending:
            return
        sent, row = self.pending.pop(seq)
        row["ack_t"] = f"{t:.3f}"
        row["rtt_ms"] = f"{t - sent:.2f}"
        self.rtts.append(t - sent)
        self.acks += 1
        self._log.writerow(row)
        self._log_file.flush()

    def _expire(self):
        cutoff = now_ms() - ACK_TIMEOUT_S * 1000
        for seq in [k for k, (sent, _) in self.pending.items() if sent < cutoff]:
            _, row = self.pending.pop(seq)
            self.lost += 1
            self._log.writerow(row)
        self._log_file.flush()

    def _on_hello(self, data, addr):
        try:
            text = data.decode().strip()
        except UnicodeDecodeError:
            return
        if not text.startswith("HELLO,"):
            return
        ip = text.split(",", 1)[1] or addr[0]
        self.hello_last = (ip, now_ms())
        self.set_target(ip, "hello")

    # ---------- status for /control ----------
    def status(self):
        r = sorted(self.rtts)
        def pct(p):
            return r[min(len(r) - 1, int(p * len(r)))] if r else None
        return {
            "target_ip": self.target_ip,
            "target_source": self.target_source,
            "hello_ip": self.hello_last[0] if self.hello_last else None,
            "hello_age_s": (now_ms() - self.hello_last[1]) / 1000 if self.hello_last else None,
            "mode": self.mode,
            "left": self.left,
            "right": self.right,
            "roll_deg": self.roll,
            "acks": self.acks,
            "lost": self.lost,
            "rtt_median_ms": statistics.median(r) if r else None,
            "rtt_p95_ms": pct(0.95),
            "rtt_last_ms": self.rtts[-1] if self.rtts else None,
            "clock_offset_ms": self.clock_offset(),
            "log": str(self.log_path),
        }
