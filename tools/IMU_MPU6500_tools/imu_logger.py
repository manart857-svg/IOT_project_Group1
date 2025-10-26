#!/usr/bin/env python3
"""
IMU Serial Logger
-----------------
Logs IMU CSV data streamed from ESP32/Arduino running the Madgwick + Filters sketch.

Features:
- Auto-detect or force CSV header
- Adds host timestamp and optional label (activity, etc.)
- Pause/resume/quit commands
- Works for headers like:
  time_ms,ax,ay,az,gx,gy,gz,ax_ema,ay_ema,az_ema,...
"""

import argparse, csv, serial, sys, threading, datetime, re
from pathlib import Path

# === Helper functions ===
def looks_like_int(s: str) -> bool:
    try:
        int(s)
        return True
    except Exception:
        return False

def clean_line(s: str) -> str:
    return s.replace("\x00", "").strip()

parser = argparse.ArgumentParser(description="Log IMU CSV data over serial (Madgwick + Filters)")
parser.add_argument("port", help="Serial port, e.g., COM3 or /dev/ttyUSB0")
parser.add_argument("outfile", help="Output CSV file path")
parser.add_argument("--baud", type=int, default=115200)
parser.add_argument("--echo", action="store_true", help="Print each parsed row to console")
parser.add_argument("--force-cols", type=str, default="", help="Comma list of columns (after time_ms)")
args = parser.parse_args()

out_path = Path(args.outfile)
ser = serial.Serial(args.port, args.baud, timeout=2)
print(f"Connected to {args.port} @ {args.baud}. Waiting for header from device...")

# === Runtime state ===
state = {
    "paused": False,
    "label": "unknown_activity",
    "quit": False,
}

# === Commands thread ===
def command_loop():
    print("\nCommands: label <name> | pause | resume | quit")
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
        elif head == "label" and len(parts) >= 2:
            label = "_".join(parts[1:])
            state["label"] = label
            print(f"[label set to '{label}']")
        else:
            print("[unknown] Commands: label <name> | pause | resume | quit")

threading.Thread(target=command_loop, daemon=True).start()

# === Header inference ===
IMU_DEFAULT = [
    "time_ms","ax","ay","az","gx","gy","gz",
    "ax_ema","ay_ema","az_ema","gx_ema","gy_ema","gz_ema",
    "ax_median","ay_median","az_median","gx_median","gy_median","gz_median",
    "ax_lp","ay_lp","az_lp","gx_lp","gy_lp","gz_lp",
    "ax_outlier","ay_outlier","az_outlier","gx_outlier","gy_outlier","gz_outlier",
    "q0","q1","q2","q3"
]

def parse_forced_columns(arg: str):
    if not arg:
        return None
    cols = [c.strip() for c in arg.split(",") if c.strip()]
    return ["time_ms"] + cols if cols else None

forced_header = parse_forced_columns(args.force_cols)
header = None

while not state["quit"]:
    line = clean_line(ser.readline().decode(errors="ignore"))
    if not line:
        continue

    if forced_header and header is None:
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 2 and looks_like_int(re.sub(r"\D", "", parts[0]) or "0"):
            header = forced_header
            print(f"[forced header] {header}")
            first_data_line = parts
            break

    if line.lower().startswith("time_ms"):
        header = [p.strip() for p in line.split(",")]
        print(f"[header detected] {header}")
        break

    parts = [p.strip() for p in line.split(",")]
    if len(parts) >= 2 and looks_like_int(re.sub(r"\D", "", parts[0]) or "0"):
        header = IMU_DEFAULT[:len(parts)] if len(parts) <= len(IMU_DEFAULT) else ["time_ms"] + [f"col{i}" for i in range(1, len(parts))]
        print(f"[inferred header] {header} (from data)")
        first_data_line = parts
        break

if header is None:
    print("\n[error] No header or data detected. Exiting.")
    sys.exit(1)

out_header = ["time_ms","host_time_iso","label"] + header[1:]
f = out_path.open("w", newline="")
writer = csv.writer(f)
writer.writerow(out_header)
print("\nLogging... Press Ctrl+C or type 'quit' to stop.")

# === Write loop ===
def handle_data(parts, header_len):
    if len(parts) != header_len:
        return False
    if state["paused"]:
        sys.stdout.write(f"\r[paused] {','.join(parts[:8])[:100]}   ")
        sys.stdout.flush()
        return False
    t_ms = parts[0]
    host_time = datetime.datetime.now().isoformat(timespec="milliseconds")
    label = state["label"]
    row = [t_ms, host_time, label] + parts[1:]
    writer.writerow(row)
    f.flush()
    if args.echo:
        sys.stdout.write(f"\r{','.join(map(str, row))[:140]}   ")
        sys.stdout.flush()
    return True

try:
    if "first_data_line" in locals():
        handle_data(first_data_line, len(header))
    while not state["quit"]:
        line = clean_line(ser.readline().decode(errors="ignore"))
        if not line:
            continue
        if line.lower().startswith("time_ms"):
            new_header = [p.strip() for p in line.split(",")]
            if new_header != header:
                print(f"\n[notice] header changed on device → {new_header}")
                header = new_header
                out_header = ["time_ms","host_time_iso","label"] + header[1:]
                writer.writerow(out_header)
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != len(header):
            continue
        handle_data(parts, len(header))
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
