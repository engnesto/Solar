"""
PowerGray - local bridge between the MPS 5KVA-60 inverter and dashboard.html

Inverter protocol: Voltronic "PI30" ASCII (QPIGS etc.) at 2400 8N1 on the RJ45 COM port.
That port is RS232. We are currently talking to it through a USB-RS485 adapter
(CH340), which inverts the signal and shifts it by one bit. Commands still get
through, and replies can be recovered with:   char = (~byte & 0xFF) >> 1
The first ~6 characters of every reply are lost while the adapter turns around,
so QPIGS field 1 (grid voltage) is unreliable and is estimated instead.

With a real USB-RS232 cable, start with --rs232 and everything decodes cleanly.

Run:   python inverter_server.py            (COM3, port 8080)
       python inverter_server.py --port COM5 --http 8080 --rs232
Open:  http://localhost:8080
"""
import argparse, csv, json, os, re, shutil, threading, time
from collections import deque
from datetime import datetime
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
# Logs live outside the project folder so editor live-reload doesn't refresh the page
LOG_DIR = os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "PowerGray", "logs")


def migrate_old_logs():
    """Move CSVs from the old <project>/logs folder to LOG_DIR (one-time)."""
    old = os.path.join(HERE, "logs")
    if not os.path.isdir(old):
        return
    os.makedirs(LOG_DIR, exist_ok=True)
    for name in os.listdir(old):
        src, dst = os.path.join(old, name), os.path.join(LOG_DIR, name)
        if not name.endswith(".csv"):
            continue
        try:
            if os.path.exists(dst):
                with open(src) as a, open(dst, "a") as b:
                    b.writelines(a.readlines()[1:])  # skip header
                os.remove(src)
            else:
                shutil.move(src, dst)  # works across drives (D: -> C:)
        except OSError as e:
            print(f"Could not move old log {src}: {e} (continuing)")
    try:
        os.rmdir(old)
    except OSError:
        pass

# ---------------------------------------------------------------- protocol


def crc_xmodem(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pi30_frame(cmd: str) -> bytes:
    c = crc_xmodem(cmd.encode())
    hi, lo = c >> 8, c & 0xFF
    if hi in (0x28, 0x0D, 0x0A): hi += 1
    if lo in (0x28, 0x0D, 0x0A): lo += 1
    return cmd.encode() + bytes([hi, lo]) + b"\r"


def decode_rs485_hack(raw: bytes) -> str:
    return "".join(chr(((~b) & 0xFF) >> 1) for b in raw)


# QPIGS fields 2..21 (field 1 may be garbled, so it is optional in the match)
QPIGS_RE = re.compile(
    r"(?:(\d{3}\.\d) )?"            # 1 grid V (often lost)
    r"(\d{2}\.\d) (\d{3}\.\d) (\d{2}\.\d) "   # 2 grid Hz, 3 out V, 4 out Hz
    r"(\d{4}) (\d{4}) (\d{3}) (\d{3}) "       # 5 VA, 6 W, 7 load %, 8 bus V
    r"(\d{2}\.\d{2}) (\d{3}) (\d{3}) (\d{4}) " # 9 batt V, 10 chg A, 11 SOC, 12 temp
    r"(\d{4}) (\d{3}\.\d) (\d{2}\.\d{2}) (\d{5}) "  # 13 PV A, 14 PV V, 15 SCC V, 16 dischg A
    r"([01]{8})(?: (\d{2}) (\d{2}) (\d{5}) ([01]{3}))?"  # 17 status, 18-21 optional
)


def parse_qpigs(text: str):
    m = QPIGS_RE.search(text)
    if not m:
        return None
    g = m.groups()
    f = lambda i: float(g[i]) if g[i] is not None else None
    status = g[16]
    d = {
        "gridVoltageRaw": f(0),
        "gridFrequency": f(1),
        "outputVoltage": f(2),
        "outputFrequency": f(3),
        "apparentPower": int(g[4]),
        "loadPower": int(g[5]),
        "loadPercent": int(g[6]),
        "busVoltage": int(g[7]),
        "batteryVoltage": f(8),
        "batteryChargeCurrent": int(g[9]),
        "batterySoc": int(g[10]),
        "invTemp": int(g[11]),
        "pvCurrent": float(g[12]),
        "pvVoltage": f(13),
        "sccBatteryVoltage": f(14),
        "batteryDischargeCurrentRaw": int(g[15]),
        "statusBits": status,
        "fanOffset": int(g[17]) if g[17] else None,
        "eepromVersion": g[18],
        "pvPowerReported": int(g[19]) if g[19] else None,
        "statusBits2": g[20],
    }
    d["loadOn"] = status[3] == "1"
    d["charging"] = status[5] == "1"
    d["sccCharging"] = status[6] == "1"
    d["acCharging"] = status[7] == "1"
    return d


# QPGS0 (parallel-info query; works on single units too). Mode letter sits far enough
# into the reply to survive the RS485 garbling, and it carries a clean grid voltage.
QPGS_RE = re.compile(
    r" ([PSLBFHD]) (\d{2}) (\d{3}\.\d) (\d{2}\.\d{2}) (\d{3}\.\d) (\d{2}\.\d{2}) "  # mode fault gridV gridHz outV outHz
    r"(\d{4}) (\d{4}) (\d{3}) (\d{2}\.\d) (\d{3}) (\d{3}) (\d{3}\.\d) (\d{3}) "      # VA W load% battV chgA SOC pvV totChgA
    r"(\d{5}) (\d{5}) (\d{3}) ([01]{8}) (\d) (\d) (\d{3}) (\d{3}) (\d{2})"           # totVA totW tot% status outMode chgPrio maxChg maxRange maxAC
    r"(?: (\d{2}) (\d{3}))?"                                                          # pvA dischgA (newer firmware)
)
WORK_MODES = {"P": "POWER ON", "S": "STANDBY", "L": "LINE", "B": "BATTERY",
              "F": "FAULT", "H": "POWER SAVING", "D": "SHUTDOWN"}
# Standard Voltronic/Axpert fault codes (shown on the LCD as the number)
FAULT_CODES = {
    1: "Fan locked while inverter is off",
    2: "Over temperature",
    3: "Battery voltage too high",
    4: "Battery voltage too low",
    5: "Output short circuit or internal over-temperature",
    6: "Output voltage too high",
    7: "Overload time-out",
    8: "Bus voltage too high",
    9: "Bus soft start failed",
    51: "Over current or surge",
    52: "Bus voltage too low",
    53: "Inverter soft start failed",
    55: "DC voltage on AC output (over DC voltage)",
    56: "Battery connection open",
    57: "Current sensor failed",
    58: "Output voltage too low",
}
CHARGER_PRIORITY = {"0": "Utility first", "1": "Solar first", "2": "Solar + Utility", "3": "Only solar"}


def parse_qpgs(text: str):
    m = QPGS_RE.search(text)
    if not m:
        return None
    g = m.groups()
    st = g[17]
    d = {
        "workMode": g[0],
        "faultCode": int(g[1]),
        "gridVoltage": float(g[2]),
        "gridFrequency": float(g[3]),
        "outputVoltage": float(g[4]),
        "chargerPriority": CHARGER_PRIORITY.get(g[19], g[19]),
        "pgsStatusBits": st,
        "sccOk": st[0] == "1",
        "acCharging": st[1] == "1",
        "sccCharging": st[2] == "1",
        "lineLoss": st[5] == "1",
        "loadOn": st[6] == "1",
    }
    if not (0 <= d["gridVoltage"] <= 280 and 0 <= d["gridFrequency"] <= 70):
        return None
    return d


def sanity_ok(d) -> bool:
    """Reject frames that decoded but contain bit errors."""
    return (
        0 <= d["gridFrequency"] <= 70
        and 0 <= d["outputVoltage"] <= 280
        and 0 <= d["outputFrequency"] <= 70
        and 20 <= d["batteryVoltage"] <= 70
        and 0 <= d["batterySoc"] <= 100
        and 0 <= d["pvVoltage"] <= 160
        and 0 <= d["loadPower"] <= 6000
        and 0 <= d["batteryChargeCurrent"] <= 120
        and 0 <= d["batteryDischargeCurrentRaw"] <= 200
        and 0 <= d["invTemp"] <= 120
    )


def derive(d, pgs=None):
    """Turn raw QPIGS (+ optional QPGS0) into the metrics dashboard.html expects."""
    pv_w = d["pvPowerReported"]
    if pv_w is None:
        pv_w = round(d["pvCurrent"] * d["sccBatteryVoltage"])
    load = d["loadPower"]
    bv = d["batteryVoltage"]
    chg_a = d["batteryChargeCurrent"]
    dis_a = d["batteryDischargeCurrentRaw"]
    batt_net_a = chg_a - dis_a  # + charging, - discharging
    batt_w = round(batt_net_a * bv)
    grid_present = d["gridFrequency"] > 40
    grid_v = d["gridVoltageRaw"]
    grid_v_est = False
    if pgs:
        grid_v = pgs["gridVoltage"]
        grid_present = pgs["gridVoltage"] > 100 and not pgs["lineLoss"]
    elif grid_v is None or not (0 <= grid_v <= 280):
        grid_v = d["outputVoltage"] if grid_present else 0.0
        grid_v_est = True

    batt_out = max(0, -batt_w)
    batt_in = max(0, batt_w)
    work = pgs["workMode"] if pgs else None
    ac_charging = pgs["acCharging"] if pgs else d["acCharging"]

    if work == "L":
        # Line mode: the grid feeds the load through the bypass; PV only charges the battery
        solar_to_load, batt_to_load, grid_to_load = 0, 0, load
        grid_w = load + (max(0, batt_in - pv_w) if ac_charging else 0)
    else:
        # Inverter mode (or mode unknown): energy balance PV + battery (+grid) -> load
        solar_to_load = min(load, pv_w)
        batt_to_load = min(load - solar_to_load, batt_out)
        grid_to_load = max(0, load - solar_to_load - batt_to_load) if (grid_present and work is None) else 0
        grid_w = (max(0, batt_in - pv_w) if ac_charging else 0) + grid_to_load

    charge_txt = "BATTERY CHARGING" if batt_net_a > 0 else "BATTERY DISCHARGING" if batt_net_a < 0 else ""
    if work == "L":
        mode = "LINE MODE" + (f" · {charge_txt}" if charge_txt else "")
    elif work == "B":
        mode = "BATTERY MODE" + (" · SOLAR" if pv_w > 5 else "")
    elif work == "F":
        fc = pgs["faultCode"]
        mode = f"FAULT {fc:02d} · {FAULT_CODES.get(fc, 'see manual')}"
    elif work:
        mode = WORK_MODES.get(work, work)
    elif grid_present and grid_to_load > 0:
        mode = "LINE (est.)"
    else:
        mode = "INVERTER (est.)"

    return {
        "pvPower": pv_w,
        "pvVoltage": d["pvVoltage"],
        "pvCurrent": d["pvCurrent"],
        "loadPower": load,
        "loadPercent": d["loadPercent"],
        "outputVoltage": d["outputVoltage"],
        "outputFrequency": d["outputFrequency"],
        "apparentPower": d["apparentPower"],
        "batteryPower": batt_w,
        "batteryVoltage": bv,
        "batterySoc": d["batterySoc"],
        "batteryDischargeCurrent": float(batt_net_a),  # name kept for the UI: + = charging
        "batteryChargeCurrent": chg_a,
        "batteryTemp": None,
        "gridPresent": grid_present,
        "gridPower": float(grid_w),
        "gridVoltage": float(grid_v),
        "gridVoltageEstimated": grid_v_est,
        "gridFrequency": d["gridFrequency"],
        "gridImport": grid_w,
        "gridExport": 0,
        "solarToLoad": solar_to_load,
        "batteryToLoad": batt_to_load,
        "gridToLoad": grid_to_load,
        "invTemp": d["invTemp"],
        "busVoltage": d["busVoltage"],
        "efficiency": None,
        "fanSpeed": None,
        "invStatus": mode,
        "workMode": work,  # L=line, B=battery, ... (None = unknown)
        "faultCode": pgs["faultCode"] if pgs else None,
        "faultText": FAULT_CODES.get(pgs["faultCode"], "Unknown code – see manual")
        if pgs and pgs["faultCode"] else None,
        "chargerPriority": pgs["chargerPriority"] if pgs else None,
        "charging": d["charging"],
        "sccCharging": d["sccCharging"],
        "acCharging": ac_charging,
        "statusBits": d["statusBits"],
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }


# ---------------------------------------------------------------- poller


class Inverter:
    def __init__(self, port, rs232=False, interval=3.0):
        self.port, self.rs232, self.interval = port, rs232, interval
        self.lock = threading.Lock()
        self.new_data = threading.Condition()  # wakes /api/stream clients
        self.seq = 0
        self.pending = []  # diagnostic queries: [cmd, threading.Event, result]
        self.qpgs_supported = True
        self.last_pgs, self.last_pgs_at = None, 0.0
        self.latest = None
        self.last_raw = ""
        self.last_error = None
        self.ok = self.bad = 0
        self.history = deque(maxlen=2880)  # 24 h at 30 s
        self._last_hist = 0
        self.raw_log = deque(maxlen=30)

    def query(self, ser, cmd, wait=2.2):
        ser.reset_input_buffer()
        ser.write(pi30_frame(cmd))
        ser.flush()
        buf, end = b"", time.time() + wait
        while time.time() < end:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                # Clean RS232 replies end in CR; with the RS485 hack, wait for the line to go quiet
                if self.rs232 and buf.endswith(b"\r") and len(buf) > 20:
                    break
            elif buf and len(buf) > 100:
                break
        return buf

    def decode(self, raw):
        return raw.decode("ascii", "replace") if self.rs232 else decode_rs485_hack(raw)

    def run(self):
        while True:
            try:
                with serial.Serial(self.port, 2400, timeout=0.15) as ser:
                    self.last_error = None
                    while True:
                        raw = self.query(ser, "QPIGS")
                        # Auto-detect cable: try clean RS232 ASCII first, then the RS485 decode
                        text = raw.decode("ascii", "replace")
                        if parse_qpigs(text):
                            self.rs232 = True
                        else:
                            hacked = decode_rs485_hack(raw)
                            if parse_qpigs(hacked):
                                self.rs232 = False
                            elif self.rs232:
                                self.rs232 = False  # nothing parsed: retry with RS485 read timing
                            text = hacked
                        self.last_raw = text
                        self.raw_log.appendleft(
                            f"{datetime.now():%H:%M:%S} RX {len(raw)}B  {''.join(c if 32 <= ord(c) < 127 else '·' for c in text)}"
                        )
                        d = parse_qpigs(text)
                        # Second query for the real work mode + clean grid voltage
                        if self.qpgs_supported:
                            r2 = self.query(ser, "QPGS0", wait=3.0)
                            t2 = r2.decode("ascii", "replace") if self.rs232 else decode_rs485_hack(r2)
                            p = parse_qpgs(t2)
                            if p:
                                self.last_pgs, self.last_pgs_at = p, time.time()
                        pgs = self.last_pgs if time.time() - self.last_pgs_at < 30 else None
                        if d and sanity_ok(d):
                            m = derive(d, pgs)
                            with self.lock:
                                self.latest = m
                                self.ok += 1
                                now = time.time()
                                if now - self._last_hist >= 30:
                                    self._last_hist = now
                                    self.history.append(m)
                                    self.log_csv(m)
                        else:
                            self.bad += 1
                        with self.new_data:
                            self.seq += 1
                            self.new_data.notify_all()
                        # Run any diagnostic queries between polls (serial port is single-user)
                        while self.pending:
                            job = self.pending.pop(0)
                            r = self.query(ser, job[0], wait=3.0)
                            job[2] = {"hex": r.hex(" "), "decoded": decode_rs485_hack(r),
                                      "ascii": r.decode("ascii", "replace")}
                            job[1].set()
                        time.sleep(self.interval)
            except serial.SerialException as e:
                self.last_error = str(e)
                print("serial error:", e)
                time.sleep(5)

    def log_csv(self, m):
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"{datetime.now():%Y-%m-%d}.csv")
        new = not os.path.exists(path)
        keys = ["timestamp", "pvPower", "pvVoltage", "pvCurrent", "loadPower", "apparentPower",
                "outputVoltage", "outputFrequency", "gridVoltage", "gridFrequency", "gridPower",
                "batteryVoltage", "batterySoc", "batteryPower", "batteryDischargeCurrent",
                "invTemp", "busVoltage", "invStatus", "statusBits"]
        with open(path, "a", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
            if new:
                w.writeheader()
            w.writerow(m)

    def status(self):
        with self.lock:
            return {
                "connected": self.latest is not None and self.last_error is None,
                "port": self.port,
                "mode": "rs232" if self.rs232 else "rs485-hack",
                "goodFrames": self.ok,
                "badFrames": self.bad,
                "error": self.last_error,
                "metrics": self.latest,
            }


# ---------------------------------------------------------------- http


def make_handler(inv: Inverter):
    class H(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=HERE, **kw)

        def log_message(self, *a):
            pass

        def _json(self, obj):
            body = json.dumps(obj).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            """Server-Sent Events: push every new reading to the browser."""
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            seen = -1
            try:
                while True:
                    with inv.new_data:
                        inv.new_data.wait_for(lambda: inv.seq != seen, timeout=15)
                        changed = inv.seq != seen
                        seen = inv.seq
                    if changed:
                        st = inv.status()
                        st["raw"] = list(inv.raw_log)[:30]
                        self.wfile.write(b"retry: 3000\ndata: " + json.dumps(st).encode() + b"\n\n")
                    else:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/api/status":
                return self._json(inv.status())
            if p == "/api/history":
                with inv.lock:
                    return self._json(list(inv.history))
            if p == "/api/raw":
                return self._json({"last": inv.last_raw, "log": list(inv.raw_log)})
            if p == "/api/stream":
                return self._stream()
            if p == "/api/days":
                return self._json(daily_energy(7))
            if p == "/api/query":
                # Read-only diagnostics, e.g. /api/query?cmd=QPGS0 (only Q... query commands)
                from urllib.parse import parse_qs, urlparse
                cmd = parse_qs(urlparse(self.path).query).get("cmd", [""])[0].upper()
                if not re.fullmatch(r"Q[A-Z0-9]{1,8}", cmd):
                    return self._json({"error": "only read-only Q... commands allowed"})
                job = [cmd, threading.Event(), None]
                inv.pending.append(job)
                if not job[1].wait(20):
                    return self._json({"error": "timeout"})
                return self._json({"cmd": cmd, **job[2]})
            if p == "/":
                self.path = "/dashboard.html"
            return super().do_GET()

    return H


def daily_energy(days=7):
    """Per-day kWh totals from the CSV logs (integrates each power column over time)."""
    from datetime import timedelta
    out = []
    today = datetime.now().date()
    for i in range(days - 1, -1, -1):
        day = today - timedelta(days=i)
        path = os.path.join(LOG_DIR, f"{day:%Y-%m-%d}.csv")
        if not os.path.exists(path):
            continue
        tot = {"solar": 0.0, "load": 0.0, "solarToLoad": 0.0, "battToLoad": 0.0, "gridToLoad": 0.0}
        prev = None
        with open(path, newline="") as fh:
            for row in csv.DictReader(fh):
                try:
                    t = datetime.fromisoformat(row["timestamp"])
                    load, pv = float(row["loadPower"]), float(row["pvPower"])
                    bp, gp = float(row["batteryPower"]), float(row.get("gridPower") or 0)
                except (KeyError, ValueError, TypeError):
                    continue
                dt = 30.0 if prev is None else min(120.0, max(0.0, (t - prev).total_seconds()))
                prev = t
                h = dt / 3600.0 / 1000.0  # W·s -> kWh
                s2l = min(load, pv)
                b2l = min(load - s2l, max(0.0, -bp))
                g2l = max(0.0, load - s2l - b2l) if gp > 0 else 0.0
                tot["solar"] += pv * h
                tot["load"] += load * h
                tot["solarToLoad"] += s2l * h
                tot["battToLoad"] += b2l * h
                tot["gridToLoad"] += g2l * h
        out.append({"day": f"{day:%a %d %b}", "date": f"{day:%Y-%m-%d}",
                    **{k: round(v, 2) for k, v in tot.items()}})
    return out


def load_today_history(inv: Inverter):
    path = os.path.join(LOG_DIR, f"{datetime.now():%Y-%m-%d}.csv")
    if not os.path.exists(path):
        return
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            for k, v in row.items():
                try:
                    row[k] = float(v)
                except (TypeError, ValueError):
                    pass
            load, pv, bp = row.get("loadPower", 0), row.get("pvPower", 0), row.get("batteryPower", 0)
            s2l = min(load, pv)
            b2l = min(load - s2l, max(0, -bp))
            row.update(solarToLoad=s2l, batteryToLoad=b2l, gridToLoad=max(0, load - s2l - b2l))
            inv.history.append(row)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--http", type=int, default=8080)
    ap.add_argument("--interval", type=float, default=3.0)
    ap.add_argument("--rs232", action="store_true", help="optional hint; cable type is auto-detected")
    a = ap.parse_args()

    migrate_old_logs()
    inv = Inverter(a.port, a.rs232, a.interval)
    load_today_history(inv)
    threading.Thread(target=inv.run, daemon=True).start()
    ThreadingHTTPServer.daemon_threads = True  # open live streams don't block Ctrl+C
    srv = ThreadingHTTPServer(("0.0.0.0", a.http), make_handler(inv))
    print(f"PowerGray: reading {a.port} -> open http://localhost:{a.http}")
    print(f"Logs: {LOG_DIR}")
    srv.serve_forever()
