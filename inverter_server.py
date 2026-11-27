"""
PowerGray - local bridge between the MPS 5KVA-60 inverter and dashboard.html

Inverter protocol: Voltronic "PI30" ASCII (QPIGS etc.) at 2400 8N1 on the RJ45 COM port.
That port is RS232. We are currently talking to it through a USB-RS485 adapter
(CH340), which inverts the signal and shifts it by one bit. Commands still get
through, and replies can be recovered with:   char = (~byte & 0xFF) >> 1
The first ~6 characters of every reply are lost while the adapter turns around,
so QPIGS field 1 (grid voltage) is unreliable and is estimated instead.

With a real USB-RS232 cable (the preferred setup) every reply is CRC-checked, warnings
are read every cycle, and inverter settings can be read and changed. The cable type is
auto-detected. Wiring that works on this unit: RJ45 pin 2 -> DB9 2, pin 1 -> DB9 3, pin 8 -> DB9 5.

Run:   python inverter_server.py --port COM4        (Windows)
       python inverter_server.py --port /dev/serial/by-id/usb-FTDI_...   (Linux)
       python inverter_server.py --set-password     (password that unlocks settings changes)
Open:  http://localhost:8080
"""
import argparse, csv, hashlib, hmac, json, os, re, secrets, shutil, threading, time
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


def day_log_path(day):
    """Daily log file, grouped as <LOG_DIR>/<year>/<month>/<YYYY-MM-DD>.csv.
    `day` is a date/datetime or a 'YYYY-MM-DD' string."""
    s = day if isinstance(day, str) else f"{day:%Y-%m-%d}"
    return os.path.join(LOG_DIR, s[:4], s[5:7], f"{s}.csv")


def file_logs_by_month():
    """Move flat <LOG_DIR>/YYYY-MM-DD.csv files into their year/month folders (one-time)."""
    if not os.path.isdir(LOG_DIR):
        return
    for name in os.listdir(LOG_DIR):
        if not re.fullmatch(r"\d{4}-\d{2}-\d{2}\.csv", name):
            continue
        src, dst = os.path.join(LOG_DIR, name), day_log_path(name[:-4])
        try:
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if os.path.exists(dst):
                with open(src) as a, open(dst, "a") as b:
                    b.writelines(a.readlines()[1:])  # skip header
                os.remove(src)
            else:
                os.replace(src, dst)
        except OSError as e:
            print(f"Could not file log {src}: {e} (continuing)")

# ---------------------------------------------------------------- protocol


def crc_xmodem(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pi30_crc(data: bytes) -> bytes:
    c = crc_xmodem(data)
    hi, lo = c >> 8, c & 0xFF
    # Voltronic never sends "(", CR or LF as a CRC byte
    if hi in (0x28, 0x0D, 0x0A): hi += 1
    if lo in (0x28, 0x0D, 0x0A): lo += 1
    return bytes([hi, lo])


def pi30_frame(cmd: str) -> bytes:
    return cmd.encode() + pi30_crc(cmd.encode()) + b"\r"


def pi30_reply(raw: bytes):
    """A clean RS232 reply "(...<crc><cr>" -> its text "(..." , or None if the CRC is wrong."""
    if len(raw) < 4 or raw[:1] != b"(" or not raw.endswith(b"\r"):
        return None
    body = raw[:-3]
    return body.decode("ascii", "replace") if pi30_crc(body) == raw[-3:-1] else None


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


# ---------------------------------------------------------------- settings (RS232 only)

OUTPUT_PRIORITY = {"0": "Utility first", "1": "Solar first", "2": "SBU (solar → battery → utility)"}
BATTERY_TYPES = {"0": "AGM", "1": "Flooded", "2": "User-defined"}
INPUT_RANGES = {"0": "Appliance (90–280 V)", "1": "UPS (170–280 V)"}
# QFLAG letters -> (key, label). Changed with PE<letter> (enable) / PD<letter> (disable).
FLAG_DEFS = {
    "a": ("buzzer", "Buzzer"),
    "x": ("backlight", "LCD backlight"),
    "k": ("lcdReturnHome", "LCD returns to main screen"),
    "j": ("powerSaving", "Power saving mode"),
    "b": ("overloadBypass", "Overload → bypass to grid"),
    "u": ("overloadRestart", "Auto-restart after overload"),
    "v": ("overTempRestart", "Auto-restart after over-temperature"),
    "y": ("sourceAlarm", "Beep when the main source is lost"),
    "z": ("faultRecord", "Record fault codes"),
}


def parse_qpiri(text: str):
    """QPIRI: rated values + the current settings."""
    f = text.lstrip("(").split()
    if len(f) < 18:
        return None
    try:
        return {
            "ratedVA": int(f[5]),
            "ratedW": int(f[6]),
            "batteryRatedV": float(f[7]),
            "batteryRechargeV": float(f[8]),     # back to grid below this (SBU / solar first)
            "batteryCutoffV": float(f[9]),       # low-battery shutdown
            "batteryBulkV": float(f[10]),        # bulk / absorption (CV) voltage
            "batteryFloatV": float(f[11]),
            "batteryType": f[12],
            "maxAcChargeCurrent": int(f[13]),
            "maxChargeCurrent": int(f[14]),
            "inputRange": f[15],
            "outputPriority": f[16],
            "chargerPriority": f[17],
            "batteryRedischargeV": float(f[22]) if len(f) > 22 else None,  # back to battery above this
        }
    except ValueError:
        return None


def parse_qflag(text: str):
    """QFLAG "(EuvyDabjkxz": letters after E are enabled, after D disabled."""
    m = re.fullmatch(r"\(E([a-z]*)D([a-z]*)", text.strip())
    if not m:
        return None
    on, off = m.groups()
    return {FLAG_DEFS[c][0]: c in on for c in on + off if c in FLAG_DEFS}


def parse_amp_list(text: str):
    """QMCHGCR / QMUCHGCR: "(010 020 030 ..." -> [10, 20, 30, ...]"""
    vals = re.findall(r"\d{2,3}", text)
    return [int(v) for v in vals] or None


def volt_cmd(prefix):
    return lambda v: f"{prefix}{v:04.1f}"


# Everything the dashboard may change. Each entry: label, group, kind, command builder, limits.
# Voltage limits are for a 48 V battery bank; the inverter still has the final say (it answers NAK).
SETTING_DEFS = {
    "outputPriority": {"label": "Output source priority", "group": "Priorities", "kind": "choice",
                       "options": OUTPUT_PRIORITY, "cmd": lambda v: f"POP0{v}"},
    "chargerPriority": {"label": "Charger source priority", "group": "Priorities", "kind": "choice",
                        "options": CHARGER_PRIORITY, "cmd": lambda v: f"PCP0{v}"},
    "maxChargeCurrent": {"label": "Max charging current (solar + grid)", "group": "Charging", "kind": "amps",
                         "list": "chargeOptions",
                         "cmd": lambda v: f"MCHGC0{v:02d}" if v < 100 else f"MNCHGC0{v:03d}"},
    "maxAcChargeCurrent": {"label": "Max grid charging current", "group": "Charging", "kind": "amps",
                           "list": "acChargeOptions", "cmd": lambda v: f"MUCHGC{v:03d}"},
    "batteryRechargeV": {"label": "Back to grid below", "group": "Battery voltages", "kind": "volts",
                         "min": 44.0, "max": 51.0, "cmd": volt_cmd("PBCV")},
    "batteryRedischargeV": {"label": "Back to battery above (0 = when full)", "group": "Battery voltages",
                            "kind": "volts", "min": 48.0, "max": 58.0, "allowZero": True, "cmd": volt_cmd("PBDV")},
    "batteryCutoffV": {"label": "Low-battery cut-off", "group": "Battery voltages", "kind": "volts",
                       "min": 40.0, "max": 48.0, "cmd": volt_cmd("PSDV")},
    "batteryBulkV": {"label": "Bulk (absorption) voltage", "group": "Battery voltages", "kind": "volts",
                     "min": 48.0, "max": 58.4, "cmd": volt_cmd("PCVV")},
    "batteryFloatV": {"label": "Float voltage", "group": "Battery voltages", "kind": "volts",
                      "min": 48.0, "max": 58.4, "cmd": volt_cmd("PBFT")},
    **{key: {"label": label, "group": "Options", "kind": "flag",
             "cmd": (lambda c: lambda v: f"{'PE' if v else 'PD'}{c}")(c)}
       for c, (key, label) in FLAG_DEFS.items()},
}


def build_setting_cmd(key, value, current):
    """Validate a requested change against the limits and the current settings.
    Returns (command, cleaned value) or raises ValueError with a message for the user."""
    d = SETTING_DEFS.get(key)
    if not d:
        raise ValueError("unknown setting")
    kind = d["kind"]
    if kind == "choice":
        v = str(value)
        if v not in d["options"]:
            raise ValueError("not one of the allowed options")
    elif kind == "amps":
        v = int(value)
        allowed = current.get(d["list"]) or []
        if v not in allowed:
            raise ValueError(f"allowed values: {', '.join(map(str, allowed))} A")
    elif kind == "volts":
        v = round(float(value), 1)
        if not (d.get("allowZero") and v == 0) and not d["min"] <= v <= d["max"]:
            raise ValueError(f"must be between {d['min']} and {d['max']} V")
        s = {**current, key: v}
        if s.get("batteryFloatV") and s.get("batteryBulkV") and s["batteryFloatV"] > s["batteryBulkV"]:
            raise ValueError("float voltage can't be above the bulk voltage")
        if s.get("batteryCutoffV") and s.get("batteryRechargeV") and s["batteryCutoffV"] >= s["batteryRechargeV"]:
            raise ValueError("cut-off must be below the back-to-grid voltage")
        if s.get("batteryRedischargeV") and s.get("batteryRechargeV") and s["batteryRedischargeV"] <= s["batteryRechargeV"]:
            raise ValueError("back-to-battery must be above back-to-grid (or 0)")
    elif kind == "flag":
        if not isinstance(value, bool):
            raise ValueError("must be true or false")
        v = value
    else:
        raise ValueError("unsupported setting")
    return d["cmd"](v), v


def setting_defs_public():
    """SETTING_DEFS without the command builders, for the dashboard."""
    return {k: {f: x for f, x in d.items() if f != "cmd"} for k, d in SETTING_DEFS.items()}


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
        # Settings (QPIRI + QFLAG + allowed current lists); RS232 only
        self.settings = None
        self.settings_at = 0.0
        self.settings_stale = True
        self.firmware = None

    def query(self, ser, cmd, wait=2.2, quiet_len=100):
        ser.reset_input_buffer()
        ser.write(pi30_frame(cmd))
        ser.flush()
        buf, end = b"", time.time() + wait
        while time.time() < end:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                # Clean RS232 replies end in CR (CRC bytes never are CR); with the RS485 hack,
                # wait for the line to go quiet
                if self.rs232 and buf.endswith(b"\r") and len(buf) >= 4:
                    break
            elif buf and len(buf) > quiet_len:
                break
        return buf

    def decode(self, raw):
        """RS232: CRC-checked text ("" if the CRC is wrong). RS485 hack: best-effort decode."""
        return (pi30_reply(raw) or "") if self.rs232 else decode_rs485_hack(raw)

    def read_settings(self, ser):
        """QPIRI + QFLAG (+ the allowed current lists and firmware, once). RS232 only."""
        s = dict(self.settings or {})
        p = parse_qpiri(self.decode(self.query(ser, "QPIRI")))
        fl = parse_qflag(self.decode(self.query(ser, "QFLAG")))
        if not p or fl is None:
            return False
        s.update(p)
        s.update(fl)
        if not s.get("chargeOptions"):
            s["chargeOptions"] = parse_amp_list(self.decode(self.query(ser, "QMCHGCR")))
        if not s.get("acChargeOptions"):
            s["acChargeOptions"] = parse_amp_list(self.decode(self.query(ser, "QMUCHGCR")))
        if not self.firmware:
            fw = self.decode(self.query(ser, "QVFW"))
            m = re.search(r"VERFW:([\d.]+)", fw)
            self.firmware = m.group(1).lstrip("0") if m else None
        with self.lock:
            self.settings, self.settings_at, self.settings_stale = s, time.time(), False
        return True

    def run_job(self, ser, job):
        """A queued request from the web side (the serial port has one user: this thread)."""
        if job["kind"] == "query":
            r = self.query(ser, job["cmd"], wait=3.0)
            job["result"] = {"hex": r.hex(" "), "decoded": decode_rs485_hack(r),
                             "ascii": r.decode("ascii", "replace")}
        elif job["kind"] == "set":
            r = self.query(ser, job["cmd"], wait=4.0)
            reply = pi30_reply(r)
            job["result"] = {"reply": reply, "ack": reply == "(ACK"}
            self.settings_stale = True
            self.read_settings(ser)  # read back what the inverter now reports
        job["done"].set()

    def run(self):
        while True:
            try:
                with serial.Serial(self.port, 2400, timeout=0.15) as ser:
                    self.last_error = None
                    silent = 0
                    while True:
                        raw = self.query(ser, "QPIGS")
                        # An unplugged USB adapter often doesn't raise on Windows: the old handle
                        # just reads nothing forever, and a re-plugged adapter is never picked up.
                        # A few silent polls in a row -> drop the handle and reopen the port.
                        if not raw:
                            silent += 1
                            if silent >= 3:
                                raise serial.SerialException("no reply from inverter (adapter unplugged?)")
                            self.bad += 1
                            continue  # skip QPGS0/QPIWS: they'd only time out too
                        silent = 0
                        # Auto-detect cable: try clean RS232 ASCII first, then the RS485 decode
                        text = raw.decode("ascii", "replace")
                        crc_bad = False
                        if parse_qpigs(text):
                            self.rs232 = True
                            crc_bad = pi30_reply(raw) is None  # parsed, but damaged on the way
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
                        d = None if crc_bad else parse_qpigs(text)
                        # Second query for the real work mode + clean grid voltage
                        if self.qpgs_supported:
                            r2 = self.query(ser, "QPGS0", wait=3.0)
                            p = parse_qpgs(self.decode(r2))
                            if p:
                                self.last_pgs, self.last_pgs_at = p, time.time()
                        pgs = self.last_pgs if time.time() - self.last_pgs_at < 30 else None
                        if pgs:
                            self.track_fault(pgs["faultCode"])
                        # Warning bits: every cycle on RS232 (a reply takes ~0.2 s); every 3rd on
                        # the RS485 cable, where each query costs ~2 s
                        self.cycle += 1
                        if self.rs232 or self.cycle % 3 == 1:
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
                        # Settings: on start, every 5 min, and right after a change (RS232 only)
                        if self.rs232 and (self.settings_stale or time.time() - self.settings_at > 300):
                            self.read_settings(ser)
                        # Run queued web requests between polls (serial port is single-user)
                        while self.pending:
                            self.run_job(ser, self.pending.pop(0))
                        time.sleep(self.interval)
            except (serial.SerialException, OSError) as e:
                # OSError: a yanked adapter can surface as a raw Windows/Linux I/O error
                if self.last_error != str(e):
                    print("serial error:", e)  # print once, not every retry
                self.last_error = str(e)
                # Fail queued web requests now, so a settings change made while the port is
                # down doesn't run by surprise after the adapter is plugged back in
                while self.pending:
                    job = self.pending.pop(0)
                    job["result"] = {"error": "inverter not connected"}
                    job["done"].set()
                time.sleep(2)

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
        path = day_log_path(datetime.now())
        os.makedirs(os.path.dirname(path), exist_ok=True)
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


# ---------------------------------------------------------------- auth (settings changes only)

# Next to the logs folder: %LOCALAPPDATA%\PowerGray\auth.json or ~/PowerGray/auth.json
AUTH_FILE = os.path.join(os.path.dirname(LOG_DIR), "auth.json")
SESSION_IDLE = 30 * 60       # signed out after 30 min without activity
LOCK_AFTER, LOCK_FOR = 5, 300  # 5 wrong passwords -> no tries for 5 min (per client address)


def hash_password(pw: str, salt: bytes, rounds: int) -> str:
    return hashlib.pbkdf2_hmac("sha256", pw.encode(), salt, rounds).hex()


def set_password_interactive():
    import getpass
    pw = getpass.getpass("New settings password (min 8 characters): ")
    if len(pw) < 8:
        raise SystemExit("Too short - nothing saved.")
    if getpass.getpass("Repeat it: ") != pw:
        raise SystemExit("The two entries differ - nothing saved.")
    salt, rounds = os.urandom(16), 300_000
    os.makedirs(os.path.dirname(AUTH_FILE), exist_ok=True)
    with open(AUTH_FILE, "w") as fh:
        json.dump({"salt": salt.hex(), "rounds": rounds, "hash": hash_password(pw, salt, rounds)}, fh)
    try:
        os.chmod(AUTH_FILE, 0o600)  # owner-only on Linux; no effect on Windows
    except OSError:
        pass
    print(f"Saved to {AUTH_FILE}. Restart the server to use it.")


class Auth:
    def __init__(self):
        self.lock = threading.Lock()
        self.sessions = {}   # token -> last activity
        self.fails = {}      # client ip -> (count, locked_until)
        try:
            with open(AUTH_FILE) as fh:
                self.cfg = json.load(fh)
        except (OSError, ValueError):
            self.cfg = None

    @property
    def password_set(self):
        return bool(self.cfg)

    def login(self, ip, pw):
        """-> (token or None, error message or None)"""
        if not self.cfg:
            return None, "No password is set on the server yet"
        now = time.time()
        with self.lock:
            n, until = self.fails.get(ip, (0, 0))
            if now < until:
                return None, f"Too many wrong passwords - try again in {(int(until - now) + 59) // 60} min"
        good = hmac.compare_digest(
            hash_password(str(pw), bytes.fromhex(self.cfg["salt"]), self.cfg["rounds"]), self.cfg["hash"])
        with self.lock:
            if not good:
                n = 1 if until else n + 1  # a served lock-out starts the count again
                self.fails[ip] = (n, now + LOCK_FOR if n >= LOCK_AFTER else 0)
                return None, "Wrong password"
            self.fails.pop(ip, None)
            token = secrets.token_urlsafe(32)
            self.sessions = {t: a for t, a in self.sessions.items() if now - a < SESSION_IDLE}
            self.sessions[token] = now
            return token, None

    def check(self, token):
        """Valid session? Also extends it (sliding expiry)."""
        if not token:
            return False
        now = time.time()
        with self.lock:
            last = self.sessions.get(token)
            if last is None or now - last >= SESSION_IDLE:
                self.sessions.pop(token, None)
                return False
            self.sessions[token] = now
            return True

    def logout(self, token):
        with self.lock:
            self.sessions.pop(token, None)


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


def make_handler(inv: Inverter, auth: Auth):
    class H(SimpleHTTPRequestHandler):
        # Don't trust the OS registry (Windows sometimes maps .js to text/plain)
        extensions_map = {**SimpleHTTPRequestHandler.extensions_map,
                          ".js": "text/javascript", ".css": "text/css", ".svg": "image/svg+xml",
                          ".woff2": "font/woff2", ".ttf": "font/ttf"}

        def __init__(self, *a, **kw):
            super().__init__(*a, directory=HERE, **kw)

        def log_message(self, *a):
            pass

        def _json(self, obj, code=200, cors=True, cookie=None):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            if cors:  # read-only data may be fetched from a page served elsewhere
                self.send_header("Access-Control-Allow-Origin", "*")
            if cookie is not None:
                self.send_header("Set-Cookie", cookie)
            self.end_headers()
            self.wfile.write(body)

        # ---- session helpers (settings changes only)
        def _token(self):
            from http.cookies import SimpleCookie
            c = SimpleCookie()
            try:
                c.load(self.headers.get("Cookie", ""))
            except Exception:
                return None
            return c["pg_session"].value if "pg_session" in c else None

        def _same_origin(self):
            """Changes are only accepted from pages served by this server (blocks other sites
            from posting with the browser's cookie)."""
            origin = self.headers.get("Origin")
            if origin is None:
                return True  # non-browser client; it still needs the session cookie
            from urllib.parse import urlparse
            # Behind an HTTPS proxy (tailscale serve) the browser's host may arrive as
            # X-Forwarded-Host. Another site can't forge that header: a cross-site request with
            # a custom header needs a CORS preflight, which this server never approves.
            hosts = {self.headers.get("Host", ""), self.headers.get("X-Forwarded-Host", "")}
            return urlparse(origin).netloc in hosts - {""}

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n > 4096 or "application/json" not in self.headers.get("Content-Type", ""):
                return None
            try:
                return json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return None

        def _settings_view(self):
            with inv.lock:
                s = dict(inv.settings) if inv.settings else None
                read_at = inv.settings_at
            if not inv.rs232:
                why = "Settings need the RS232 cable (the server is reading through RS485)."
            elif not s:
                why = "Reading the settings from the inverter…"
            else:
                why = None
            return {
                "available": why is None,
                "reason": why,
                "passwordSet": auth.password_set,
                "authenticated": auth.check(self._token()),
                "values": s,
                "readAt": datetime.fromtimestamp(read_at).astimezone().isoformat(timespec="seconds")
                if read_at else None,
                "firmware": inv.firmware,
                "batteryTypes": BATTERY_TYPES,
                "inputRanges": INPUT_RANGES,
                "defs": setting_defs_public(),
            }

        def do_POST(self):
            p = self.path.split("?")[0]
            if not self._same_origin():
                return self._json({"error": "cross-site request refused"}, 403, cors=False)
            body = self._body()
            if body is None:
                return self._json({"error": "expected a small JSON body"}, 400, cors=False)
            if p == "/api/login":
                token, err = auth.login(self.client_address[0], body.get("password", ""))
                if not token:
                    return self._json({"error": err}, 401, cors=False)
                return self._json({"ok": True}, cors=False,
                                  cookie=f"pg_session={token}; HttpOnly; SameSite=Strict; Path=/; Max-Age={SESSION_IDLE}")
            if p == "/api/logout":
                auth.logout(self._token())
                return self._json({"ok": True}, cors=False,
                                  cookie="pg_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0")
            if p == "/api/settings":
                if not auth.check(self._token()):
                    return self._json({"error": "Please unlock settings first"}, 401, cors=False)
                if not inv.rs232 or not inv.settings:
                    return self._json({"error": "Settings need the RS232 cable and a settings read"}, 409, cors=False)
                key, value = body.get("key"), body.get("value")
                with inv.lock:
                    current = dict(inv.settings)
                try:
                    cmd, value = build_setting_cmd(key, value, current)
                except (ValueError, TypeError) as e:
                    return self._json({"error": str(e) or "invalid value"}, 400, cors=False)
                old = current.get(key)
                job = {"kind": "set", "cmd": cmd, "done": threading.Event(), "result": None}
                inv.pending.append(job)
                if not job["done"].wait(30):
                    return self._json({"error": "The inverter did not answer in time"}, 504, cors=False)
                res = job["result"]
                if "error" in res:
                    return self._json({"error": "The inverter is not connected — nothing was changed"}, 503, cors=False)
                label = SETTING_DEFS[key]["label"]
                if res["ack"]:
                    inv.add_event("start", "setting", None, key, f"{label}: {old} → {value}")
                print(f"setting {key} {old!r} -> {value!r} ({cmd}) from {self.client_address[0]}: {res['reply']}")
                return self._json({"ok": res["ack"], "reply": res["reply"], "command": cmd,
                                   "settings": self._settings_view()}, cors=False)
            return self._json({"error": "not found"}, 404, cors=False)

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
                job = {"kind": "query", "cmd": cmd, "done": threading.Event(), "result": None}
                inv.pending.append(job)
                if not job["done"].wait(20):
                    return self._json({"error": "timeout"})
                return self._json({"cmd": cmd, **job["result"]})
            if p == "/api/settings":
                return self._json(self._settings_view(), cors=False)
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
        path = day_log_path(day)
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
    path = day_log_path(date_str)
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
    ap.add_argument("--set-password", action="store_true",
                    help="set the password that unlocks inverter settings in the dashboard, then exit")
    a = ap.parse_args()
    if a.set_password:
        set_password_interactive()
        raise SystemExit(0)

    migrate_old_logs()
    file_logs_by_month()
    inv = Inverter(a.port, a.rs232, a.interval)
    load_today_history(inv)
    threading.Thread(target=inv.run, daemon=True).start()
    ThreadingHTTPServer.daemon_threads = True  # open live streams don't block Ctrl+C
    auth = Auth()
    if not auth.password_set:
        print("Settings are locked: no password yet (run with --set-password to create one)")
    srv = ThreadingHTTPServer(("0.0.0.0", a.http), make_handler(inv, auth))
    print(f"PowerGray: reading {a.port} -> open http://localhost:{a.http}")
    print(f"Logs: {LOG_DIR}")
    srv.serve_forever()
