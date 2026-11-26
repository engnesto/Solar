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
    # Status b7..b0 (string index 0 = b7)
    d["sbuPriorityVersion"] = status[0] == "1"
    d["configChanged"] = status[1] == "1"
    d["sccFirmwareUpdated"] = status[2] == "1"
    d["loadOn"] = status[3] == "1"
    d["batteryVoltageSteady"] = status[4] == "1"  # charger holding a steady (CV) voltage
    d["charging"] = status[5] == "1"
    d["sccCharging"] = status[6] == "1"
    d["acCharging"] = status[7] == "1"
    # Second status field b10 b9 b8 (newer firmware only)
    s2 = g[20]
    d["floatCharging"] = s2[0] == "1" if s2 else None
    d["switchOn"] = s2[1] == "1" if s2 else None
    d["dustproofInstalled"] = s2[2] == "1" if s2 else None
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
OUTPUT_MODES = {"0": "Single unit", "1": "Parallel", "2": "3-phase: phase 1", "3": "3-phase: phase 2",
                "4": "3-phase: phase 3"}
BATTERY_STATUS = {"00": "Normal", "01": "Under voltage", "10": "Open (not connected)"}


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
        "outputMode": OUTPUT_MODES.get(g[18], g[18]),
        "totalChargeCurrent": int(g[13]),
        "maxChargeCurrent": int(g[20]),
        "maxAcChargeCurrent": int(g[22]),
        # Status b7..b0: SCC ok, AC charging, SCC charging, battery (2 bits), line loss, load on, config changed
        "pgsStatusBits": st,
        "sccOk": st[0] == "1",
        "acCharging": st[1] == "1",
        "sccCharging": st[2] == "1",
        "batteryStatus": BATTERY_STATUS.get(st[3:5], st[3:5]),
        "lineLoss": st[5] == "1",
        "loadOn": st[6] == "1",
        "configChanged": st[7] == "1",
    }
    if not (0 <= d["gridVoltage"] <= 280 and 0 <= d["gridFrequency"] <= 70):
        return None
    return d


# QPIWS warning-status bits a0..a31. kind: "fault" (inverter stops), "warning" (keeps running,
# blinking warning icon on the LCD), "info" (normal condition worth noting). code = number on the LCD.
# Bits marked * are faults when a1 (inverter fault) is also set, otherwise warnings.
QPIWS_DEFS = {
    1: ("inverterFault", "Inverter fault", "fault", None),
    2: ("busOver", "Bus voltage too high", "fault", "08"),
    3: ("busUnder", "Bus voltage too low", "fault", "52"),
    4: ("busSoftFail", "Bus soft start failed", "fault", "09"),
    5: ("lineFail", "Grid lost / out of range", "info", None),
    6: ("opvShort", "Output short circuit", "warning", None),
    7: ("invVoltLow", "Inverter output voltage too low", "fault", "58"),
    8: ("invVoltHigh", "Inverter output voltage too high", "fault", "06"),
    9: ("overTemp", "Over temperature", "warning*", "02"),
    10: ("fanLocked", "Fan locked", "warning*", "01"),
    11: ("battHigh", "Battery voltage too high", "warning*", "03"),
    12: ("battLow", "Low battery", "warning", "04"),
    13: ("overcharge", "Battery overcharge", "warning", None),
    14: ("battUnderShutdown", "Battery under-voltage shutdown", "warning", None),
    15: ("battDerating", "Battery derating", "warning", None),
    16: ("overload", "Overload", "warning*", "07"),
    17: ("eepromFault", "EEPROM fault", "warning", None),
    18: ("invOverCurrent", "Inverter over current", "fault", "51"),
    19: ("invSoftFail", "Inverter soft start failed", "fault", "53"),
    20: ("selfTestFail", "Self test failed", "fault", None),
    21: ("opDcOver", "DC voltage on AC output", "fault", "55"),
    22: ("battOpen", "Battery not connected", "fault", "56"),
    23: ("currentSensorFail", "Current sensor failed", "fault", "57"),
    24: ("battShort", "Battery short circuit", "fault", None),
    25: ("powerLimit", "Output power derating", "warning", "10"),
    26: ("pvHigh", "PV voltage too high", "warning", None),
    27: ("mpptOverloadFault", "MPPT overload fault", "warning", None),
    28: ("mpptOverload", "MPPT overload", "warning", None),
    29: ("battTooLowToCharge", "Battery too low to charge", "warning", None),
}
QPIWS_LEN = 32  # bits in the reply; updated automatically from any clean reply


def parse_qpiws(text: str, length: int):
    """Return (bits, first_known_index, length) or None.

    Clean reply: "(" + bits. With the RS485 decode the first ~6 characters are lost, so
    the surviving run of 0/1 is the *tail* of the bit string and is aligned from the end;
    the lost leading bits come back as None (unknown)."""
    m = re.search(r"\(([01]{32,40})", text)
    if m:
        bits = m.group(1)
        return [int(c) for c in bits], 0, len(bits)
    runs = re.findall(r"[01]{20,}", text)
    if not runs:
        return None
    run = max(runs, key=len)
    if len(run) >= length:  # only the "(" was lost
        return [int(c) for c in run[:length]], 0, length
    start = length - len(run)
    return [None] * start + [int(c) for c in run], start, length


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


def split_load(load, pv_w, batt_w, mode):
    """Split the load into (solar, battery, grid) parts that always add up to the load.
    mode: "line" = grid feeds the load through the bypass; "inverter" = PV + battery feed it;
    "unknown" = mode not reported but the grid is up (PV, then the measured battery, then grid)."""
    if mode == "line":
        return 0, 0, load
    solar = min(load, pv_w)
    if mode == "unknown":
        batt = min(load - solar, max(0, -batt_w))
        return solar, batt, load - solar - batt
    # Inverter mode: the battery is the only other source. Its current is only reported in
    # whole amps (~55 W steps), so take the rest of the load as battery instead of measuring it.
    return solar, load - solar, 0


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

    batt_in = max(0, batt_w)
    work = pgs["workMode"] if pgs else None
    ac_charging = pgs["acCharging"] if pgs else d["acCharging"]

    if work == "L":
        # Line mode: the grid feeds the load through the bypass; PV only charges the battery
        solar_to_load, batt_to_load, grid_to_load = split_load(load, pv_w, batt_w, "line")
        grid_w = load + (max(0, batt_in - pv_w) if ac_charging else 0)
    else:
        # Inverter mode (or mode unknown): energy balance PV + battery (+grid) -> load
        solar_to_load, batt_to_load, grid_to_load = split_load(
            load, pv_w, batt_w, "unknown" if (grid_present and work is None) else "inverter")
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
        # Inverter reports PV current in whole amps; derive a 0.1 A value from PV watts / PV volts
        "pvCurrent": (round(d["pvPowerReported"] / d["pvVoltage"], 1)
                      if d["pvPowerReported"] is not None and d["pvVoltage"] and d["pvVoltage"] > 5
                      else d["pvCurrent"]),
        "pvCurrentRaw": d["pvCurrent"],
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
        # extra QPIGS details
        "sccBatteryVoltage": d["sccBatteryVoltage"],
        "fanOffset": d["fanOffset"],  # battery-voltage offset for fan-on, in 10 mV units
        "eepromVersion": d["eepromVersion"],
        "floatCharging": d["floatCharging"],
        "batteryVoltageSteady": d["batteryVoltageSteady"],
        "switchOn": d["switchOn"],
        "dustproofInstalled": d["dustproofInstalled"],
        "sbuPriorityVersion": d["sbuPriorityVersion"],
        "sccFirmwareUpdated": d["sccFirmwareUpdated"],
        "configChanged": d["configChanged"] or bool(pgs and pgs["configChanged"]),
        # extra QPGS0 details (None when QPGS0 isn't available)
        "outputMode": pgs["outputMode"] if pgs else None,
        "maxChargeCurrent": pgs["maxChargeCurrent"] if pgs else None,
        "maxAcChargeCurrent": pgs["maxAcChargeCurrent"] if pgs else None,
        "totalChargeCurrent": pgs["totalChargeCurrent"] if pgs else None,
        "batteryStatus": pgs["batteryStatus"] if pgs else None,
        "sccOk": pgs["sccOk"] if pgs else None,
        # include the UTC offset so browsers in another timezone don't see the reading as hours old
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
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
        # Warnings (QPIWS) + event log
        self.cycle = 0
        self.qpiws_len = QPIWS_LEN
        self.ws_prev = None      # previous parsed bit list (for debouncing)
        self.ws_state = {}       # bit index -> confirmed 0/1
        self.ws_at = 0.0         # last successful QPIWS parse
        self.ws_bits = None      # last raw bit string (? = lost in the RS485 decode)
        self.ws_known_from = None
        self.active_fault = 0    # last QPGS0 fault code
        self.events = deque(load_events(200), maxlen=200)  # newest first

    def query(self, ser, cmd, wait=2.2, quiet_len=100):
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
            elif buf and len(buf) > quiet_len:
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
                        if pgs:
                            self.track_fault(pgs["faultCode"])
                        # Warning bits: not every cycle (each query costs ~2 s on the RS485 cable)
                        self.cycle += 1
                        if self.cycle % 3 == 1:
                            r3 = self.query(ser, "QPIWS", wait=2.5, quiet_len=25)
                            t3 = self.decode(r3)
                            self.raw_log.appendleft(
                                f"{datetime.now():%H:%M:%S} WS {len(r3)}B  {''.join(c if 32 <= ord(c) < 127 else '·' for c in t3)}"
                            )
                            self.update_warnings(parse_qpiws(t3, self.qpiws_len))
                        if d and sanity_ok(d):
                            m = derive(d, pgs)
                            m.update(self.warning_fields())
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

    # ---------------- warnings / events

    def update_warnings(self, parsed):
        """Debounce: a bit only changes state after two identical readings in a row
        (filters single-frame bit errors from the RS485 decode)."""
        if not parsed:
            return
        bits, known_from, length = parsed
        if known_from == 0:
            self.qpiws_len = length
        self.ws_at = time.time()
        self.ws_known_from = known_from
        self.ws_bits = "".join("?" if b is None else str(b) for b in bits)
        prev, self.ws_prev = self.ws_prev, bits
        if not prev or len(prev) != len(bits):
            return
        for i, b in enumerate(bits):
            if b is None or prev[i] != b or i not in QPIWS_DEFS:
                continue
            old = self.ws_state.get(i, 0)
            self.ws_state[i] = b
            if b != old:
                key, text, kind, code = QPIWS_DEFS[i]
                self.add_event("start" if b else "clear", self.kind_of(i), code, key, text)

    def kind_of(self, i):
        kind = QPIWS_DEFS[i][2]
        if kind == "warning*":
            return "fault" if self.ws_state.get(1) == 1 else "warning"
        return kind

    def track_fault(self, code):
        if code == self.active_fault:
            return
        if self.active_fault:
            self.add_event("clear", "fault", f"{self.active_fault:02d}", f"fault{self.active_fault}",
                           FAULT_CODES.get(self.active_fault, "Unknown fault"))
        if code:
            self.add_event("start", "fault", f"{code:02d}", f"fault{code}",
                           FAULT_CODES.get(code, "Unknown fault - see manual"))
        self.active_fault = code

    def add_event(self, event, kind, code, key, text):
        ev = {"timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
              "event": event, "kind": kind, "code": code or "", "key": key, "text": text}
        with self.lock:
            self.events.appendleft(ev)
        print(f"{ev['timestamp']} {event.upper():5} {kind:7} {code or '--'} {text}")
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            path = os.path.join(LOG_DIR, EVENTS_FILE)
            new = not os.path.exists(path)
            with open(path, "a", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=EVENT_KEYS)
                if new:
                    w.writeheader()
                w.writerow(ev)
        except OSError as e:
            print("could not write event log:", e)

    def warning_fields(self):
        fresh = bool(self.ws_at) and time.time() - self.ws_at < 120
        active = [
            {"bit": i, "key": QPIWS_DEFS[i][0], "text": QPIWS_DEFS[i][1],
             "kind": self.kind_of(i), "code": QPIWS_DEFS[i][3]}
            for i, v in sorted(self.ws_state.items()) if v and i in QPIWS_DEFS
        ] if fresh else []
        return {
            "warnings": active,
            "warningsAvailable": fresh,
            "warningBits": self.ws_bits,
            "warningBitsKnownFrom": self.ws_known_from,
        }

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
                "events": list(self.events)[:20],
            }


EVENTS_FILE = "events.csv"
EVENT_KEYS = ["timestamp", "event", "kind", "code", "key", "text"]


def load_events(n):
    """Last n warning/fault events from the log, newest first."""
    path = os.path.join(LOG_DIR, EVENTS_FILE)
    if not os.path.exists(path):
        return []
    try:
        with open(path, newline="") as fh:
            rows = list(csv.DictReader(fh))
    except OSError:
        return []
    return rows[::-1][:n]


# ---------------------------------------------------------------- http


def make_handler(inv: Inverter):
    class H(SimpleHTTPRequestHandler):
        # Don't trust the OS registry (Windows sometimes maps .js to text/plain)
        extensions_map = {**SimpleHTTPRequestHandler.extensions_map,
                          ".js": "text/javascript", ".css": "text/css", ".svg": "image/svg+xml",
                          ".woff2": "font/woff2", ".ttf": "font/ttf"}

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
            if p == "/api/events":
                with inv.lock:
                    return self._json(list(inv.events))
            if p == "/api/days":
                return self._json(daily_energy(7))
            if p == "/api/day":
                # One logged day's readings, e.g. /api/day?date=2026-11-25
                from urllib.parse import parse_qs, urlparse
                date = parse_qs(urlparse(self.path).query).get("date", [""])[0]
                if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", date):
                    return self._json({"error": "date must be YYYY-MM-DD"})
                return self._json(day_rows(date))
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
                    # drop the offset: older rows have none, and naive/aware can't be subtracted
                    t = datetime.fromisoformat(row["timestamp"]).replace(tzinfo=None)
                    load, pv = float(row["loadPower"]), float(row["pvPower"])
                    bp, gp = float(row["batteryPower"]), float(row.get("gridPower") or 0)
                except (KeyError, ValueError, TypeError):
                    continue
                s2l, b2l, g2l = split_load(load, pv, bp, csv_mode(row.get("invStatus"), gp))
                dt = 30.0 if prev is None else min(120.0, max(0.0, (t - prev).total_seconds()))
                prev = t
                h = dt / 3600.0 / 1000.0  # W·s -> kWh
                tot["solar"] += pv * h
                tot["load"] += load * h
                tot["solarToLoad"] += s2l * h
                tot["battToLoad"] += b2l * h
                tot["gridToLoad"] += g2l * h
        out.append({"day": f"{day:%a %d %b}", "date": f"{day:%Y-%m-%d}",
                    **{k: round(v, 2) for k, v in tot.items()}})
    return out


def csv_mode(status, grid_w):
    """Recover split_load's mode from a logged row (the CSV keeps invStatus, not workMode)."""
    status = str(status or "")
    if status.startswith("LINE MODE"):
        return "line"
    if status.endswith("(est.)") and isinstance(grid_w, (int, float)) and grid_w > 0:
        return "unknown"
    return "inverter"


def day_rows(date_str):
    """All logged readings of one day (YYYY-MM-DD), with the load split added."""
    path = os.path.join(LOG_DIR, f"{date_str}.csv")
    if not os.path.exists(path):
        return []
    rows = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            for k, v in row.items():
                try:
                    row[k] = float(v)
                except (TypeError, ValueError):
                    pass
            load, pv, bp = row.get("loadPower", 0), row.get("pvPower", 0), row.get("batteryPower", 0)
            gp = row.get("gridPower", 0)
            s2l, b2l, g2l = split_load(load, pv, bp, csv_mode(row.get("invStatus"), gp))
            row.update(solarToLoad=s2l, batteryToLoad=b2l, gridToLoad=g2l)
            rows.append(row)
    return rows


def load_today_history(inv: Inverter):
    inv.history.extend(day_rows(f"{datetime.now():%Y-%m-%d}"))


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
