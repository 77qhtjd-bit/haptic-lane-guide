"""Feasibility-test server: receives iPhone camera frames + IMU over WebSocket.

Run from the repo root:  uvicorn server:app --app-dir laptop --host 0.0.0.0 --port 8000
Phone pages in web/ are served under /web/ (the same files are published on GitHub Pages).
"""
import asyncio
import csv
import json
import re
import struct
import time
from datetime import datetime
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, Response
from fastapi.staticfiles import StaticFiles

from actuator import Actuator

ROOT = Path(__file__).parent          # laptop/
STATIC = ROOT / "static"               # laptop-only pages
WEB = ROOT.parent / "web"              # phone pages (also published on GitHub Pages)
DATA = ROOT.parent / "data"            # recorded sessions (gitignored)

# One row per IMU sample. "kind" is "motion" (devicemotion) or "orientation" (deviceorientation).
IMU_COLUMNS = [
    "t", "kind",
    "acc_x", "acc_y", "acc_z",            # acceleration (m/s^2, no gravity)
    "accg_x", "accg_y", "accg_z",         # accelerationIncludingGravity (m/s^2)
    "rot_alpha", "rot_beta", "rot_gamma", # rotationRate (deg/s on iOS)
    "interval",                           # devicemotion interval, raw value as reported
    "ori_alpha", "ori_beta", "ori_gamma", # deviceorientation (deg)
    "ori_absolute",
]
# Short keys used by the client JSON -> CSV column names.
KEYMAP = {
    "ax": "acc_x", "ay": "acc_y", "az": "acc_z",
    "gx": "accg_x", "gy": "accg_y", "gz": "accg_z",
    "ra": "rot_alpha", "rb": "rot_beta", "rg": "rot_gamma",
    "iv": "interval",
    "al": "ori_alpha", "be": "ori_beta", "ga": "ori_gamma", "abs": "ori_absolute",
}
KIND = {"m": "motion", "o": "orientation"}

app = FastAPI()
app.mount("/web", StaticFiles(directory=WEB, html=True), name="web")
actuator = Actuator()

# session_id -> Session
sessions = {}
# Latest data for the /viewer page.
latest = {"frame": None, "frame_t": None, "motion": None, "orientation": None, "session": None}


def sample_to_row(s):
    row = {"t": s.get("t"), "kind": KIND.get(s.get("k"), s.get("k"))}
    for short, col in KEYMAP.items():
        if short in s:
            row[col] = s[short]
    return row


def write_imu_csv(path, samples):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=IMU_COLUMNS)
        w.writeheader()
        for s in samples:
            w.writerow(sample_to_row(s))


class Session:
    def __init__(self, session_id=None):
        self.id = session_id or datetime.now().strftime("%Y%m%d-%H%M%S-%f")[:-3]
        self.dir = DATA / self.id
        (self.dir / "frames").mkdir(parents=True, exist_ok=True)
        self.meta = {"session_id": self.id, "created_server_time": time.time(), "meta_updates": [], "recordings": []}
        self._imu_file = None
        self._imu_writer = None
        self.connected = False
        # Per-second counters for the terminal summary.
        self.cnt = {"motion": 0, "orientation": 0, "frames": 0, "bytes": 0}
        self.last_rtt = None
        sessions[self.id] = self
        self.save_meta()

    def save_meta(self):
        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=2))

    def write_imu(self, samples):
        if self._imu_writer is None:
            self._imu_file = open(self.dir / "imu.csv", "w", newline="")
            self._imu_writer = csv.DictWriter(self._imu_file, fieldnames=IMU_COLUMNS)
            self._imu_writer.writeheader()
        for s in samples:
            row = sample_to_row(s)
            self._imu_writer.writerow(row)
            if row["kind"] in self.cnt:
                self.cnt[row["kind"]] += 1
            latest[row["kind"]] = row
        self._imu_file.flush()

    def write_frame(self, t, jpeg):
        (self.dir / "frames" / f"{t:.3f}.jpg").write_bytes(jpeg)
        self.cnt["frames"] += 1
        latest["frame"] = jpeg
        latest["frame_t"] = t
        latest["session"] = self.id

    def close(self):
        self.connected = False
        if self._imu_file:
            self._imu_file.close()
            self._imu_file = None
            self._imu_writer = None


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})


@app.get("/viewer")
def viewer():
    return FileResponse(STATIC / "viewer.html", headers={"Cache-Control": "no-store"})


@app.get("/latest.jpg")
def latest_jpg():
    if latest["frame"] is None:
        return Response(status_code=404)
    return Response(latest["frame"], media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/latest")
def latest_json():
    return {k: latest[k] for k in ("frame_t", "motion", "orientation", "session")}


@app.get("/control")
def control():
    return FileResponse(STATIC / "control.html", headers={"Cache-Control": "no-store"})


@app.get("/api/actuator")
def actuator_status():
    return actuator.status()


@app.post("/api/actuator")
async def actuator_update(request: Request):
    """Body: {"ip": "..."} and/or {"mode": "manual"|"tilt"} and/or {"left": n, "right": n}."""
    m = await request.json()
    if "ip" in m:
        ip = str(m["ip"]).strip()
        if not re.match(r"^\d{1,3}(\.\d{1,3}){3}$", ip):
            return JSONResponse({"error": "bad ip"}, status_code=400)
        actuator.set_target(ip, "manual")
    if m.get("mode") in ("manual", "tilt", "frame"):
        actuator.set_mode(m["mode"])
    if "left" in m and "right" in m:
        actuator.set_manual(m["left"], m["right"])
    return actuator.status()


@app.get("/record")
def record_page():
    return RedirectResponse("/web/record.html")


@app.get("/import")
def import_page():
    return FileResponse(STATIC / "import.html", headers={"Cache-Control": "no-store"})


IMPORT_NAME = re.compile(r"^(rec_[0-9]{8}-[0-9]{6})\.(mp4|webm|json)$")


def build_import_meta(d, base):
    """Once both files of a /record clip are present, write meta.json + IMU CSV in the layout verify.py reads."""
    videos = [p for p in (d / f"{base}.mp4", d / f"{base}.webm") if p.exists()]
    jpath = d / f"{base}.json"
    if not videos or not jpath.exists():
        return False
    rec = json.loads(jpath.read_text())
    write_imu_csv(d / f"{base}_imu.csv", rec.get("samples", []))
    info = {k: v for k, v in rec.items() if k not in ("samples", "frames", "imu_gaps", "events")}
    info.update({"tag": base.removeprefix("rec_"), "video_file": videos[0].name, "imu_csv": f"{base}_imu.csv",
                 "n_samples": len(rec.get("samples", [])), "n_frames": len(rec.get("frames", [])),
                 "n_imu_gaps": len(rec.get("imu_gaps", []))})
    meta = {"session_id": d.name, "source": "/record import", "imported_server_time": time.time(), "recordings": [info]}
    (d / "meta.json").write_text(json.dumps(meta, indent=2))
    return True


@app.post("/api/import")
async def import_file(request: Request, name: str):
    """Upload one file saved by /record. Files of a clip go to data/import-<clip>/."""
    m = IMPORT_NAME.match(name)
    if not m:
        return JSONResponse({"error": "expected rec_YYYYMMDD-HHMMSS.mp4/.json from /record"}, status_code=400)
    base = m.group(1)
    d = DATA / f"import-{base[4:]}"
    (d / "frames").mkdir(parents=True, exist_ok=True)   # verify.py expects the folder (empty for recordings)
    n = 0
    with open(d / name, "wb") as f:
        async for chunk in request.stream():
            f.write(chunk)
            n += len(chunk)
    if m.group(2) == "json":
        try:
            json.loads((d / name).read_text())
        except ValueError:
            return JSONResponse({"error": "invalid JSON"}, status_code=400)
    complete = build_import_meta(d, base)
    print(f"[import] {name}: {n / 1e6:.1f} MB -> {d}" + (" (clip complete)" if complete else ""), flush=True)
    msg = f"saved {n / 1e6:.1f} MB"
    if complete:
        msg += f" - clip complete: python verify.py data/{d.name}"
    return {"ok": True, "message": msg, "dir": f"data/{d.name}", "complete": complete}


@app.get("/imutest")
def imutest_page():
    return RedirectResponse("/web/runner-test.html")


@app.post("/api/imutest")
async def imutest_result(request: Request, run: str):
    if not re.match(r"^[0-9]{1,20}$", run):
        return JSONResponse({"error": "bad run"}, status_code=400)
    out = DATA / "imutest"
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{run}.json").write_bytes(await request.body())
    return {"ok": True}


@app.get("/ble")
def ble_page():
    return FileResponse(STATIC / "ble.html", headers={"Cache-Control": "no-store"})




@app.post("/api/ble_log")
async def ble_log(request: Request):
    """Saves the /ble page's command log (phone clock) to data/ble/<time>.csv."""
    m = await request.json()
    out = DATA / "ble"
    out.mkdir(parents=True, exist_ok=True)
    name = datetime.now().strftime("%Y%m%d-%H%M%S")
    rows = m.get("rows", [])
    columns = list(dict.fromkeys(k for r in rows for k in r))   # union of keys, first-seen order
    with open(out / f"{name}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        w.writerows(rows)
    (out / f"{name}.json").write_text(json.dumps({k: v for k, v in m.items() if k != "rows"}, indent=2))
    print(f"[ble] saved {len(m.get('rows', []))} rows to data/ble/{name}.csv", flush=True)
    return {"ok": True, "file": f"data/ble/{name}.csv"}


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    sess = Session()
    sess.connected = True
    print(f"[ws] connected, session {sess.id}", flush=True)
    await ws.send_text(json.dumps({"type": "session", "id": sess.id}))
    try:
        while True:
            msg = await ws.receive()
            if msg["type"] == "websocket.disconnect":
                break
            if msg.get("bytes") is not None:
                data = msg["bytes"]
                if len(data) < 9:
                    continue
                # 8-byte little-endian float64 timestamp header + JPEG bytes.
                (t,) = struct.unpack("<d", data[:8])
                recv_t = time.time() * 1000
                sess.write_frame(t, data[8:])
                sess.cnt["bytes"] += len(data)
                # Integrated path: decide per frame, reply on the same socket (phone relays over BLE).
                await ws.send_text(json.dumps(actuator.phone_command(t, recv_t, data[8:])))
                continue
            text = msg.get("text")
            if text is None:
                continue
            sess.cnt["bytes"] += len(text)
            m = json.loads(text)
            typ = m.get("type")
            if typ == "imu":
                sess.write_imu(m.get("samples", []))
                actuator.on_imu(m.get("samples", []), time.time() * 1000)   # read-only
            elif typ == "ping":
                actuator.on_ping(m.get("t"), m.get("last_rtt"), time.time() * 1000)
                if m.get("last_rtt") is not None:
                    sess.last_rtt = m["last_rtt"]
                await ws.send_text(json.dumps({"type": "pong", "id": m.get("id"), "t": m.get("t"),
                                               "server_time": time.time() * 1000}))
            elif typ == "hello":
                sess.meta["client"] = m.get("meta", {})
                sess.save_meta()
            elif typ == "meta":
                sess.meta["meta_updates"].append({"server_time": time.time(), **m.get("meta", {})})
                sess.save_meta()
    except WebSocketDisconnect:
        pass
    finally:
        sess.close()
        print(f"[ws] disconnected, session {sess.id}", flush=True)


SAFE_TAG = re.compile(r"^[0-9]{1,20}$")
SAFE_EXT = re.compile(r"^[a-z0-9]{1,5}$")


@app.post("/upload")
async def upload(request: Request, kind: str, tag: str, session: str = "", ext: str = "mp4"):
    """Record-mode upload. Client sends kind=imu first (JSON), then kind=video (raw bytes)."""
    if not SAFE_TAG.match(tag) or not SAFE_EXT.match(ext):
        return JSONResponse({"error": "bad tag/ext"}, status_code=400)
    sess = sessions.get(session) if session else None
    if sess is None:
        # Unknown/empty session (e.g. WebSocket was not connected): start a new one.
        sess = Session()
    if kind == "imu":
        body = await request.body()
        rec = json.loads(body)
        (sess.dir / f"rec_{tag}_imu.json").write_bytes(body)
        write_imu_csv(sess.dir / f"rec_{tag}_imu.csv", rec.get("samples", []))
        info = {k: v for k, v in rec.items() if k != "samples"}
        info.update({"tag": tag, "imu_csv": f"rec_{tag}_imu.csv", "n_samples": len(rec.get("samples", []))})
        sess.meta["recordings"].append(info)
        sess.save_meta()
        print(f"[upload] session {sess.id}: rec {tag} IMU, {info['n_samples']} samples", flush=True)
        return {"ok": True, "session": sess.id}
    if kind == "video":
        name = f"rec_{tag}.{ext}"
        n = 0
        with open(sess.dir / name, "wb") as f:
            async for chunk in request.stream():
                f.write(chunk)
                n += len(chunk)
        for r in sess.meta["recordings"]:
            if r.get("tag") == tag:
                r["video_file"] = name
                r["video_bytes"] = n
        sess.save_meta()
        print(f"[upload] session {sess.id}: rec {tag} video {name}, {n / 1e6:.1f} MB", flush=True)
        return {"ok": True, "session": sess.id, "bytes": n}
    return JSONResponse({"error": "bad kind"}, status_code=400)


async def summary_loop():
    while True:
        await asyncio.sleep(1.0)
        for s in list(sessions.values()):
            if not s.connected:
                continue
            c = s.cnt
            rtt = f"{s.last_rtt:.0f} ms" if s.last_rtt is not None else "-"
            print(f"[{s.id}] IMU {c['motion']:3d} Hz | orient {c['orientation']:3d} Hz | "
                  f"frames {c['frames']:2d} fps | {c['bytes'] / 1024:7.1f} kB/s | RTT {rtt}", flush=True)
            s.cnt = {k: 0 for k in c}


@app.on_event("startup")
async def startup():
    DATA.mkdir(exist_ok=True)
    asyncio.create_task(summary_loop())
    await actuator.start()
