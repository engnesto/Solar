@echo off
cd /d "%~dp0"
start "" http://localhost:8080
python inverter_server.py --port COM3 --http 8080
pause
