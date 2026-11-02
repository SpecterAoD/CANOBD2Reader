#!/usr/bin/env python3
"""Archive exact firmware, ELF, MAP and non-secret build metadata for coredumps."""
import argparse
import hashlib
import importlib.metadata
import json
import os
import platform
import subprocess
import zipfile
from pathlib import Path


def archive_build(root, out, environments, version):
    packages = []
    core = Path(os.environ.get("PLATFORMIO_CORE_DIR", str(Path.home() / ".platformio")))
    for kind in ("packages", "platforms"):
        for path in sorted((core / kind).glob("*/package.json")):
            data = json.loads(path.read_text())
            packages.append({"name": data.get("name"), "version": data.get("version"), "kind": kind})
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root))
    out.mkdir(parents=True, exist_ok=True)
    for environment in environments:
        files = {name: root / ".pio/build" / environment / name for name in
                 ("firmware.bin", "firmware.elf", "firmware.map")}
        files.update({name: root / name for name in ("platformio.ini", "requirements-dev.txt")})
        hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in files.items()}
        metadata = {"commit": commit, "trackedChanges": dirty, "version": version,
                    "environment": environment, "python": platform.python_version(),
                    "platformio": importlib.metadata.version("platformio"),
                    "packages": packages, "sha256": hashes}
        with zipfile.ZipFile(out / f"debug-{environment}.zip", "w", zipfile.ZIP_DEFLATED) as bundle:
            for name, path in files.items():
                bundle.write(path, name)
            bundle.writestr("build.json", json.dumps(metadata, indent=2) + "\n")
            bundle.writestr("SHA256SUMS", "".join(f"{digest}  {name}\n" for name, digest in hashes.items()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("environments", nargs="+", choices=("sender", "display", "sender_dev", "display_dev"))
    args = parser.parse_args()
    archive_build(Path(__file__).resolve().parents[1], args.out, args.environments, args.version)


if __name__ == "__main__":
    main()
