# PowerGray – live dashboard for an MPS 5KVA-60 solar inverter

A local web dashboard that reads live data from a Voltronic-family hybrid inverter
(MPS 5KVA-60, also sold as Axpert-style units) over its RJ45 **COM** port.

## Quick start

```
pip install pyserial
python inverter_server.py            # defaults: COM3, http port 8080
```

Then open **http://localhost:8080**, or double-click `start_dashboard.bat`.

Options: `--port COM5`, `--http 8080`, `--interval 3`.

## Files

| File | Purpose |
|---|---|
| `inverter_server.py` | Polls the inverter, decodes replies, logs CSV, serves the dashboard and a JSON/SSE API |
| `dashboard.html` | The dashboard (live diagram, cards, charts) – updates via server push |
| `start_dashboard.bat` | Starts the server and opens the browser |
| `probe.py` | Diagnostic: tries PI30 and Modbus at several baud rates on a port |

## How it talks to the inverter

- **Protocol:** Voltronic **PI30** ASCII at **2400 8N1** (not Modbus). Commands used:
  - `QPIGS` – live values (PV, load, battery, temperatures, status bits)
  - `QPGS0` – work mode (Line / Battery / Fault), fault code, clean grid voltage, charger priority
- **Cable:** the COM port is **RS232** (RJ45 pin 1 TX, pin 2 RX, pin 8 GND).
  - With a proper USB-RS232 cable, replies are clean ASCII.
  - With a **USB-RS485** adapter (CH340) on pins 1/8, commands still get through, but replies
    arrive inverted and shifted by one bit. They are recovered with `char = (~byte & 0xFF) >> 1`.
    The first ~6 characters of each reply are lost, which is why `QPGS0` is used for mode and grid voltage.
  - The server detects which cable is in use automatically.

## API

| Endpoint | Returns |
|---|---|
| `GET /api/status` | Latest decoded metrics + link stats |
| `GET /api/stream` | Server-Sent Events: pushes every new reading |
| `GET /api/history` | Today's 30-second samples |
| `GET /api/raw` | Recent decoded raw replies |
| `GET /api/query?cmd=QXXX` | Diagnostic: send a read-only `Q…` command (setting commands are refused) |

## Logs

One CSV per day (one row every 30 s) in `%LOCALAPPDATA%\PowerGray\logs`, kept outside the project
folder so editor live-reload doesn't refresh the page.

## Notes

- The server listens on all interfaces, so the dashboard is reachable from your phone at
  `http://<pc-ip>:8080`. It has no authentication, so keep it on your home network.
- Only one program can use the COM port at a time.
