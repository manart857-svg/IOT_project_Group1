import argparse, csv, serial, sys, threading, datetime
from pathlib import Path

parser = argparse.ArgumentParser(description="Log CSV from ESP32 over serial")
parser.add_argument("port", help="Serial port, e.g., COM3 or /dev/ttyUSB0")
parser.add_argument("outfile", help="Output CSV path")
parser.add_argument("--baud", type=int, default=9600)
args = parser.parse_args()

out_path = Path(args.outfile)
ser = serial.Serial(args.port, args.baud, timeout=2)
print(f"Connected to {args.port} @ {args.baud}. Waiting for data...")

state = {
    "paused": False,
    "true_distance_cm": 10,
    "quit": False,
}

def command_loop():
    print("\nCommands: dist <cm> | pause | resume | quit")
    while True:
        try:
            cmd = input().strip()
        except EOFError:
            break
        if not cmd:
            continue
        parts = cmd.split()
        head = parts[0].lower()

        if head == "quit":
            state["quit"] = True
            break
        elif head == "pause":
            state["paused"] = True
            print("[paused]")
        elif head == "resume":
            state["paused"] = False
            print("[resumed]")
        elif head == "dist" and len(parts) >= 2:
            try:
                val = float(parts[1])
                state["true_distance_cm"] = val
                print(f"[true_distance_cm set to {val}]")
            except ValueError:
                print("[error] dist expects a number, e.g., 'dist 50'")
        else:
            print("[unknown] Commands: dist <cm> | pause | resume | quit")

import threading
cmd_thread = threading.Thread(target=command_loop, daemon=True)
cmd_thread.start()

f = out_path.open("w", newline="")
writer = csv.writer(f)
writer.writerow(["t_ms", "host_time_iso", "true_distance_cm", "measured_dist_cm"])

print("Logging... Press Ctrl+C or type 'quit' to stop.")

try:
    while not state["quit"]:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 3:
            continue

        if state["paused"]:
            sys.stdout.write("\r[paused] Last raw: " + line + " ")
            sys.stdout.flush()
            continue

        t_ms = parts[0]               
        measured = parts[2]           
        true_dist = state["true_distance_cm"]

        host_time_iso = datetime.datetime.now().isoformat(timespec="milliseconds")

        row = [t_ms, host_time_iso, true_dist if true_dist is not None else "", measured]
        writer.writerow(row)
        f.flush()

        sys.stdout.write("\rLast: " + ",".join(map(str, row)) + " ")
        sys.stdout.flush()

except KeyboardInterrupt:
    print("\nStopped by keyboard.")
finally:
    state["quit"] = True
    try:
        ser.close()
    except Exception:
        pass
    f.close()
    print(f"\nSaved to {out_path.resolve()}")
