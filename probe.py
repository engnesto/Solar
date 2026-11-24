"""Probe the inverter on a serial port with Voltronic PI30 ASCII and Modbus RTU."""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"


def crc_xmodem(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def pi30_frame(cmd: str) -> bytes:
    c = crc_xmodem(cmd.encode())
    hi, lo = c >> 8, c & 0xFF
    # Voltronic avoids 0x28 '(' 0x0d CR 0x0a LF in CRC bytes
    if hi in (0x28, 0x0D, 0x0A): hi += 1
    if lo in (0x28, 0x0D, 0x0A): lo += 1
    return cmd.encode() + bytes([hi, lo]) + b"\r"


def crc_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


def modbus_read(slave, fn, reg, count) -> bytes:
    body = bytes([slave, fn, reg >> 8, reg & 0xFF, count >> 8, count & 0xFF])
    return body + crc_modbus(body)


def xfer(ser, data, wait=1.5):
    ser.reset_input_buffer()
    ser.write(data)
    ser.flush()
    end = time.time() + wait
    buf = b""
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if buf.endswith(b"\r"):
                break
    return buf


def show(label, tx, rx):
    print(f"{label:40} TX={tx.hex(' ')}")
    print(f"{'':40} RX({len(rx)})={rx!r}")


mode = sys.argv[2] if len(sys.argv) > 2 else "all"

if mode in ("all", "pi30"):
    for baud in (2400, 9600):
        with serial.Serial(PORT, baud, timeout=0.2) as ser:
            for cmd in ("QPI", "QID", "QPIGS"):
                tx = pi30_frame(cmd)
                show(f"PI30 {cmd} @{baud}", tx, xfer(ser, tx, 2.0))

if mode in ("all", "modbus"):
    for baud in (9600, 19200, 2400):
        with serial.Serial(PORT, baud, timeout=0.1) as ser:
            for slave in (1, 4):
                for fn, reg in ((3, 0), (4, 0), (3, 100), (3, 4501), (3, 0x1000)):
                    tx = modbus_read(slave, fn, reg, 10)
                    show(f"MB s{slave} f{fn} r{reg} @{baud}", tx, xfer(ser, tx, 0.6))
