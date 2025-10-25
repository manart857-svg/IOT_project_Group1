import argparse, csv, serial, sys, threading, datetime, re
from pathlib import Path

def looks_like_int(s: str) -> bool:
    try:
        int(s)
        return True
    except Exception:
        return False

def looks_like_float(s: str) -> bool:
    try:
        float(s)
        return True
    except Exception:
        return False

def clean_line(s: str) -> str:
    return s.replace("\x00", "").strip()

parser = argparse.ArgumentParser(description="Log CSV from ESP32 (raw + filters) over serial")
parser.add_argument("port", help="Serial port, e.g., COM3 or /dev/ttyUSB0")
parser.add_argument("outfile", help="Output CSV path")
parser.add_argument("--baud", type=int, default=9600)
parser.add_argument("--echo", action="store_true", help="Print each parsed row to console")
parser.add_argument("--force-cols", type=str, default="", help="Comma list of columns after t_ms, e.g. 'raw_cm,ema_cm,lp_cm'")
args = parser.parse_args()

out_path = Path(args.outfile)
ser = serial.Serial(args.port, args.baud, timeout=2)
print(f"Connected to {args.port} @ {args.baud}. Waiting for header from device...")

# Shared state
state = {
    "paused": False,
    "true_distance_cm": 20.0,  # default label; update with 'dist <cm>'
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

cmd_thread = threading.Thread(target=command_loop, daemon=True)
cmd_thread.start()

PREFERRED = ["raw_cm", "ema_cm", "lp_cm", "median_cm", "outlier_cm", "kalman_cm"]

def infer_header_from_data(parts):
    """
    parts: tokens of a data-like line. parts[0] should be t_ms.
    We'll map remaining columns to PREFERRED (truncate as needed).
    """
    n = len(parts)
    if n < 2:
        return None
    k = n - 1
    cols = PREFERRED[:k]
    if len(cols) < k:  # shouldn't happen, but just in case
        cols += [f"col{i}" for i in range(len(cols)+1, k+1)]
    return ["t_ms"] + cols

def parse_forced_columns(arg: str):
    if not arg:
        return None
    cols = [c.strip() for c in arg.split(",") if c.strip()]
    if not cols:
        return None
    return ["t_ms"] + cols

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

    if line.lower().lstrip().startswith("t_ms"):
        header = [p.strip() for p in line.split(",")]
        print(f"[header detected] {header}")
        break

    parts = [p.strip() for p in line.split(",")]
    if len(parts) >= 2 and looks_like_int(re.sub(r"\D", "", parts[0]) or "0"):
        header = infer_header_from_data(parts)
        print(f"[inferred header] {header} (from first data line)")
        first_data_line = parts
        break

    sys.stdout.write("\rWaiting header... last seen: " + line[:80] + "   ")
    sys.stdout.flush()

if header is None:
    print("\n[error] No header or data detected. Exiting.")
    sys.exit(1)

out_header = ["t_ms", "host_time_iso", "true_distance_cm"] + header[1:]
f = out_path.open("w", newline="")
writer = csv.writer(f)
writer.writerow(out_header)
print("\nLogging... Press Ctrl+C or type 'quit' to stop.")

def handle_data_parts(parts, header_len):
    if len(parts) != header_len:
        return False
    if state["paused"]:
        sys.stdout.write("\r[paused] Last raw: " + ",".join(parts)[:100] + "   ")
        sys.stdout.flush()
        return False
    t_ms = parts[0]
    host_time_iso = datetime.datetime.now().isoformat(timespec="milliseconds")
    true_dist = state["true_distance_cm"]
    rest = parts[1:]
    row = [t_ms, host_time_iso, true_dist] + rest
    writer.writerow(row)
    f.flush()
    if args.echo:
        sys.stdout.write("\rLast: " + ",".join(map(str, row))[:140] + "   ")
        sys.stdout.flush()
    return True

try:
    if 'first_data_line' in locals():
        handle_data_parts(first_data_line, len(header))

    while not state["quit"]:
        line = clean_line(ser.readline().decode(errors="ignore"))
        if not line:
            continue

        if line.lower().lstrip().startswith("t_ms"):
            new_header = [p.strip() for p in line.split(",")]
            if new_header != header:
                print(f"\n[notice] Header changed on device: {new_header}")
                header = new_header
                out_header = ["t_ms", "host_time_iso", "true_distance_cm"] + header[1:]
                writer.writerow(out_header)
            continue

        parts = [p.strip() for p in line.split(",")]
        if len(parts) != len(header):
            if len(parts) >= 2 and looks_like_int(re.sub(r"\D", "", parts[0]) or "0"):
                new_header = infer_header_from_data(parts) if not forced_header else forced_header
                if new_header and len(new_header) == len(parts):
                    print(f"\n[auto re-sync] columns changed → {new_header}")
                    header = new_header
                    out_header = ["t_ms", "host_time_iso", "true_distance_cm"] + header[1:]
                    writer.writerow(out_header)
                else:
                    continue
            else:
                continue

        handle_data_parts(parts, len(header))

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
