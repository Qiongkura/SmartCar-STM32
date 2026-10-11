from pathlib import Path
import serial, time, sys
root = Path(__file__).resolve().parents[1]
port = sys.argv[1] if len(sys.argv) > 1 else 'COM13'
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 20
s = serial.Serial()
s.port = port; s.baudrate = 115200; s.timeout = 0.5
s.dtr = False; s.rts = False
s.open()
end = time.monotonic()+duration
with (root/'runtime.log').open('a', encoding='utf-8') as log:
    while time.monotonic() < end:
        line = s.readline().decode('utf-8', errors='replace').strip()
        if line:
            log.write(line+'\n'); log.flush()
            if 'LINK=' in line or 'CONNECTED' in line or 'CAR BRIDGE' in line or 'CAR_BLE' in line or 'error' in line.lower() or 'panic' in line.lower():
                print(line, flush=True)
s.close()
