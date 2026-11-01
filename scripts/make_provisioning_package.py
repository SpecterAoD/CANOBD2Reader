#!/usr/bin/env python3
"""Build a USB provisioning package for sender and display.

Builds both firmwares WITH the local include/secrets.h and collects
bootloader, partition table, otadata and application plus a flash script
(flash.sh, Linux, needs only `esptool`). The flash script checks the MAC of
the connected board against secrets.h before erasing anything.

The package contains the credentials - keep it private and delete it after
flashing (flash.sh aufraeumen).

Usage:
  python scripts/make_provisioning_package.py --out ../CANOBD2Reader-provisioning
  python scripts/make_provisioning_package.py --out DIR --skip-build   # vorhandene Builds verwenden
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SECRETS = ROOT / "include" / "secrets.h"

# chip, bootloader offset, USB VID:PID, baud
TARGETS = {
    "sender": {"chip": "esp32", "bootloader": 0x1000, "usb": "1a86:7523", "baud": 460800, "mac": "SenderAllowedMac",
               "partitions": "sender_4mb.csv"},
    "display": {"chip": "esp32s3", "bootloader": 0x0, "usb": "303a:1001", "baud": 460800, "mac": "DisplayPeerMac",
                "partitions": "display_16mb.csv"},
}
PARTITIONS_OFFSET = 0x8000


def read_secrets() -> dict[str, str]:
    if not SECRETS.exists():
        raise SystemExit(f"error: {SECRETS} missing (scripts/make_secrets.py)")
    text = SECRETS.read_text(encoding="utf-8")
    values = dict(re.findall(r'constexpr const char\* (\w+) = "([^"]*)";', text))
    for name in ("SenderAllowedMac", "DisplayPeerMac"):
        match = re.search(name + r"\[6\] = \{([^}]*)\}", text)
        parts = re.findall(r"0x([0-9A-Fa-f]{2})", match.group(1)) if match else []
        if len(parts) != 6 or all(p == "00" for p in parts):
            raise SystemExit(f"error: {name} in secrets.h is missing or zero")
        values[name] = ":".join(p.lower() for p in parts)
    for name in ("SenderWebPassword", "DisplayWebPassword", "WebPassword"):
        if values.get(name, "").startswith("change-me") or len(values.get(name, "")) < 8:
            raise SystemExit(f"error: {name} in secrets.h is a placeholder or shorter than 8")
    if values.get("ApiToken", "").startswith("change-me") or len(values.get("ApiToken", "")) < 16:
        raise SystemExit("error: ApiToken in secrets.h is a placeholder or shorter than 16")
    return values


def otadata_and_app_offsets(csv_name: str) -> tuple[int, int]:
    otadata = app = None
    for line in (ROOT / "partitions" / csv_name).read_text().splitlines():
        fields = [f.strip() for f in line.split(",")]
        if len(fields) >= 4 and not fields[0].startswith("#"):
            if fields[2] == "ota":
                otadata = int(fields[3], 16)
            if fields[2] == "ota_0":
                app = int(fields[3], 16)
    if otadata is None or app is None:
        raise SystemExit(f"error: partitions/{csv_name} without otadata/ota_0")
    return otadata, app


def find_boot_app0() -> Path:
    core = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    path = core / "packages" / "framework-arduinoespressif32" / "tools" / "partitions" / "boot_app0.bin"
    if not path.exists():
        raise SystemExit(f"error: {path} not found (set PLATFORMIO_CORE_DIR)")
    return path


FLASH_SH = r'''#!/usr/bin/env bash
# CANOBD2Reader – Provisionierung per USB (erzeugt von make_provisioning_package.py)
#
#   ./flash.sh pruefen sender|display   Board finden und MAC prüfen (ändert nichts)
#   ./flash.sh sender|display           Flash komplett löschen und neu beschreiben (Provisionierung)
#   ./flash.sh layout sender|display    Partitionstabelle + Firmware schreiben, Zugangsdaten (NVS) bleiben
#   ./flash.sh aufraeumen               Paket (enthält Zugangsdaten) löschen
#
# Ausgabe zusätzlich in /tmp/canobd2-flash.log
set -Eeuo pipefail   # jeder Fehler bricht ab
cd "$(dirname "$(readlink -f "$0")")"
LOG=/tmp/canobd2-flash.log
exec > >(tee -a "$LOG") 2>&1
trap 'chmod 644 "$LOG" 2>/dev/null || true' EXIT
trap 'echo "!! ABBRUCH in Zeile $LINENO – Gerät nicht fertig geflasht, bitte Ausgabe prüfen."' ERR
# Debian/Kali-Pakete ("dfsg") enthalten keine Flash-Stubs -> offizielles esptool
# aus einer venv bevorzugen: python3 -m venv ~/.venv-esptool && ~/.venv-esptool/bin/pip install esptool
ESPTOOL=""
for c in "$HOME/.venv-esptool/bin/esptool.py" "$(command -v esptool.py || true)" "$(command -v esptool || true)"; do
  [[ -n "$c" && -x "$c" ]] && { ESPTOOL=$c; break; }
done
[[ -n "$ESPTOOL" ]] || { echo "esptool fehlt (siehe Kommentar oben)"; exit 1; }
echo "esptool: $ESPTOOL"

declare -A CHIP=(%(chip)s) USB=(%(usb)s) MAC=(%(mac)s) BAUD=(%(baud)s) BOOT=(%(boot)s)
declare -A OTADATA=(%(otadata)s) APP=(%(app)s); PART=%(part)s

write_images() {  # $1 = target: bootloader, partition table, boot selection, firmware
  "$ESPTOOL" --chip "${CHIP[$1]}" --port "$PORT" --baud "${BAUD[$1]}" write_flash -z \
    "${BOOT[$1]}" "$1/bootloader.bin" "$PART" "$1/partitions.bin" "${OTADATA[$1]}" boot_app0.bin \
    "${APP[$1]}" "$1/firmware.bin"
}

find_port() {  # $1 = target; erstes serielles Gerät mit passender VID:PID
  local dev vid pid
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [[ -e "$dev" ]] || continue
    vid=$(udevadm info -q property -n "$dev" | sed -n 's/^ID_VENDOR_ID=//p')
    pid=$(udevadm info -q property -n "$dev" | sed -n 's/^ID_MODEL_ID=//p')
    [[ "${vid,,}:${pid,,}" == "${USB[$1]}" ]] && { echo "$dev"; return 0; }
  done
  return 1
}

check_board() {  # $1 = target -> setzt PORT
  PORT=$(find_port "$1") || { echo "!! Kein ${1}-Board gefunden (USB ${USB[$1]}). Angesteckt? Display: ggf. BOOT halten und RESET drücken."; exit 1; }
  echo "==> $1: Port $PORT"
  local mac
  mac=$("$ESPTOOL" --chip "${CHIP[$1]}" --port "$PORT" read_mac 2>&1 | sed -n 's/^MAC: *//p' | head -1 | tr 'A-F' 'a-f')
  [[ -n "$mac" ]] || { echo "!! MAC nicht lesbar – Board im Download-Modus starten und erneut versuchen."; exit 1; }
  echo "    MAC $mac, erwartet ${MAC[$1]}"
  if [[ "$mac" != "${MAC[$1]}" ]]; then
    echo "!! MAC passt nicht zu secrets.h. Falsches Board, oder secrets.h mit dieser MAC neu erzeugen und Paket neu bauen."
    exit 1
  fi
}

case "${1:-}" in
  pruefen)
    [[ -n "${CHIP[${2:-x}]:-}" ]] || { echo "Aufruf: $0 pruefen sender|display"; exit 1; }
    check_board "$2"; echo "OK – Board passt, nichts verändert." ;;
  sender|display)
    T=$1; check_board "$T"
    echo "==> $T: Flash löschen (alte Firmware, Einstellungen, Logs)"
    "$ESPTOOL" --chip "${CHIP[$T]}" --port "$PORT" --baud "${BAUD[$T]}" erase_flash
    echo "==> $T: schreiben"
    write_images "$T"
    echo "==> $T fertig. Das Gerät startet neu und übernimmt die Zugangsdaten ins NVS." ;;
  layout)
    # NVS (0x9000) is not touched: credentials stay. The log file system is
    # re-created at the new position on first boot (old log is lost).
    T=${2:-x}; [[ -n "${CHIP[$T]:-}" ]] || { echo "Aufruf: $0 layout sender|display"; exit 1; }
    check_board "$T"
    echo "==> $T: neue Partitionstabelle + Firmware schreiben (ohne Löschen, Zugangsdaten bleiben)"
    write_images "$T"
    echo "==> $T fertig. Erster Start legt das Log-Dateisystem neu an (dauert einige Sekunden)." ;;
  aufraeumen)
    read -r -p "Paket $(pwd) mit Zugangsdaten löschen? [j/N] " a; [[ "$a" =~ ^[jJ]$ ]] || exit 0
    d=$(pwd); cd / && rm -r -- "${d:?}" && echo "gelöscht: $d" ;;
  *) sed -n '2,9p' "$0"; exit 1 ;;
esac
'''


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--pio", default=os.environ.get("PIO", "pio"))
    args = parser.parse_args(argv)

    secrets = read_secrets()
    offsets = {t: otadata_and_app_offsets(c["partitions"]) for t, c in TARGETS.items()}
    if not args.skip_build:
        subprocess.run([args.pio, "run", "-e", "sender", "-e", "display"], cwd=ROOT, check=True)

    out = args.out.resolve()
    if ROOT in out.parents or out == ROOT:
        raise SystemExit("error: the package contains credentials - choose a directory outside the repository")
    if out.exists():
        raise SystemExit(f"error: {out} exists - delete it first")
    out.mkdir(parents=True, mode=0o700)
    for target in TARGETS:
        (out / target).mkdir(mode=0o700)
        for name in ("bootloader.bin", "partitions.bin", "firmware.bin"):
            shutil.copy2(ROOT / ".pio" / "build" / target / name, out / target / name)
    shutil.copy2(find_boot_app0(), out / "boot_app0.bin")

    def assoc(key):
        return " ".join(f"[{t}]={key(t, c)}" for t, c in TARGETS.items())

    script = FLASH_SH % {
        "chip": assoc(lambda t, c: c["chip"]),
        "usb": assoc(lambda t, c: c["usb"]),
        "mac": assoc(lambda t, c: secrets[c["mac"]]),
        "baud": assoc(lambda t, c: c["baud"]),
        "boot": assoc(lambda t, c: hex(c["bootloader"])),
        "otadata": assoc(lambda t, c: hex(offsets[t][0])),
        "app": assoc(lambda t, c: hex(offsets[t][1])),
        "part": hex(PARTITIONS_OFFSET),
    }
    (out / "flash.sh").write_text(script, encoding="utf-8")
    (out / "flash.sh").chmod(0o700)
    version = (ROOT / "VERSION.txt").read_text().strip()
    (out / "INFO.txt").write_text(
        f"CANOBD2Reader Provisionierung {version}\n"
        f"Sender-MAC {secrets['SenderAllowedMac']}, Display-MAC {secrets['DisplayPeerMac']}\n"
        "Enthält Zugangsdaten - nach dem Flashen mit ./flash.sh aufraeumen löschen.\n", encoding="utf-8")
    print(f"Paket: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
