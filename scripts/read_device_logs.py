#!/usr/bin/env python3
"""Read diagnostic logs (and crash reports) from a device over USB.

The diagnostic log lives in the SPIFFS partition, the crash report in the
"coredump" partition. Both are read with esptool (read only, nothing on the
device changes) and the log files are unpacked with mkspiffs.

Usage (board connected to this machine):
  python scripts/read_device_logs.py sender --out logs/
Board connected to another Linux host reachable by SSH (e.g. the kalipi):
  python scripts/read_device_logs.py sender --remote kalipi --out logs/
Devices still on the old layout (before V2.1.0-beta.3):
  python scripts/read_device_logs.py display --layout legacy --remote kalipi --out logs/

Logs may contain the VIN - keep them out of the repository.
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TARGETS = {
    # CH340 (sender) is unreliable above 115200 for long reads.
    "sender": {"chip": "esp32", "usb": "1a86:7523", "csv": "sender_4mb.csv", "baud": "115200"},
    "display": {"chip": "esp32s3", "usb": "303a:1001", "csv": "display_16mb.csv", "baud": "460800"},
}
# Layout of V2.0 - V2.1.0-beta.2 (partitions/ota_4mb.csv, both devices).
LEGACY = {"spiffs": (0x3B0000, 0x50000)}

REMOTE_SCRIPT = r'''
set -euo pipefail
E=""; for c in "$HOME/.venv-esptool/bin/esptool.py" "$(command -v esptool.py || true)" "$(command -v esptool || true)"; do
  [[ -n "$c" && -x "$c" ]] && { E=$c; break; }; done
[[ -n "$E" ]] || { echo "esptool fehlt" >&2; exit 1; }
P=""; for d in /dev/ttyUSB* /dev/ttyACM*; do [[ -e "$d" ]] || continue
  v=$(udevadm info -q property -n "$d" | sed -n 's/^ID_VENDOR_ID=//p'); p=$(udevadm info -q property -n "$d" | sed -n 's/^ID_MODEL_ID=//p')
  [[ "${v,,}:${p,,}" == "$USBID" ]] && { P=$d; break; }; done
[[ -n "$P" ]] || { echo "Board $USBID nicht gefunden" >&2; exit 1; }
"$E" --chip "$CHIP" --port "$P" --baud "$BAUD" read_flash "$OFFSET" "$SIZE" "$OUT" >&2
'''


def partition(csv_name: str, subtype: str) -> tuple[int, int]:
    for line in (ROOT / "partitions" / csv_name).read_text().splitlines():
        fields = [f.strip() for f in line.split(",")]
        if len(fields) >= 5 and not fields[0].startswith("#") and fields[2] == subtype:
            return int(fields[3], 16), int(fields[4], 16)
    raise SystemExit(f"error: no '{subtype}' partition in partitions/{csv_name}")


def read_region(target: str, offset: int, size: int, dest: Path, remote: str | None) -> None:
    env = {"USBID": TARGETS[target]["usb"], "CHIP": TARGETS[target]["chip"],
           "OFFSET": hex(offset), "SIZE": hex(size), "BAUD": TARGETS[target]["baud"]}
    if remote:
        remote_file = f"/tmp/canobd2-read-{os.getpid()}.bin"
        assignments = " ".join(f"{k}={shlex.quote(v)}" for k, v in {**env, "OUT": remote_file}.items())
        subprocess.run(["ssh", remote, f"{assignments} bash -s"], input=REMOTE_SCRIPT, text=True, check=True)
        try:
            subprocess.run(["scp", "-q", f"{remote}:{remote_file}", str(dest)], check=True)
        finally:
            subprocess.run(["ssh", remote, f"rm -f {shlex.quote(remote_file)}"], check=False)
    else:
        subprocess.run(["bash", "-s"], input=REMOTE_SCRIPT, text=True, check=True,
                       env={**os.environ, **env, "OUT": str(dest)})


def find_mkspiffs() -> Path:
    core = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    tool = core / "packages" / "tool-mkspiffs" / "mkspiffs_espressif32_arduino"
    if not tool.exists():
        raise SystemExit(f"error: {tool} missing - pio pkg install -g -t platformio/tool-mkspiffs")
    return tool


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("target", choices=TARGETS)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--remote", help="SSH host the board is connected to")
    parser.add_argument("--layout", choices=["current", "legacy"], default="current")
    args = parser.parse_args(argv)

    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True, mode=0o700)
    if args.layout == "legacy":
        spiffs = LEGACY["spiffs"]
        coredump = None
    else:
        spiffs = partition(TARGETS[args.target]["csv"], "spiffs")
        coredump = partition(TARGETS[args.target]["csv"], "coredump")

    image = out / f"{args.target}_spiffs.bin"
    read_region(args.target, *spiffs, image, args.remote)
    files = out / f"{args.target}_log"
    files.mkdir(exist_ok=True)
    subprocess.run([str(find_mkspiffs()), "-u", str(files), "-b", "4096", "-p", "256", "-s", str(spiffs[1]),
                    str(image)], check=True, stdout=subprocess.DEVNULL)
    print(f"Logdateien: {files}")
    for path in sorted(files.iterdir()):
        print(f"  {path.name}: {path.stat().st_size} Bytes")

    if coredump:
        dump = out / f"{args.target}_coredump_partition.bin"
        read_region(args.target, *coredump, dump, args.remote)
        data = dump.read_bytes()
        empty = all(b == 0xFF for b in data[:64])
        print(f"Coredump-Bereich: {'leer' if empty else 'enthält einen Absturzbericht'} ({dump})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
