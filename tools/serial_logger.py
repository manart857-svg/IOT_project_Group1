import argparse, time, csv, serial, sys
from pathlib import Path


parser = argparse.ArgumentParser(description="Log CSV from ESP32 over serial")
parser.add_argument("port", help="Serial port, e.g., COM3 or /dev/ttyUSB0")
parser.add_argument("outfile", help="Output CSV path")
parser.add_argument("--baud", type=int, default=9600)
args = parser.parse_args()


out_path = Path(args.outfile)
ser = serial.Serial(args.port, args.baud, timeout=2)
print(f"Connected to {args.port} @ {args.baud}. Waiting for header...")


# Open CSV and write header
f = out_path.open("w", newline="")
writer = csv.writer(f)
writer.writerow(["t_ms", "duration_us", "dist_cm"]) # ensure header exists


print("Logging... Press Ctrl+C to stop.")
try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        # Expect CSV lines: t_ms,duration_us,dist_cm
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 3:
            continue
        writer.writerow(parts)
        f.flush()
        sys.stdout.write("\rLast: " + line + " " )
        sys.stdout.flush()
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    ser.close()
    f.close()
    print(f"Saved to {out_path.resolve()}")