from __future__ import annotations

import argparse
from pathlib import Path

from repo_utils import repo_root, sha256_file, version_tag, write_generated


def firmware_row(target: str) -> list[str]:
    root = repo_root()
    firmware = root / ".pio" / "build" / target / "firmware.bin"
    elf = root / ".pio" / "build" / target / "firmware.elf"
    if not firmware.exists():
        return [target, "missing", "-", "-", "-"]
    return [
        target,
        "present",
        str(firmware.stat().st_size),
        sha256_file(firmware),
        str(elf.stat().st_size) if elf.exists() else "-",
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=["sender", "display", "all"], default="all")
    parser.add_argument("--output", default="docs/AUTO_FIRMWARE_ARTIFACTS.md")
    args = parser.parse_args()

    targets = ["sender", "display"] if args.target == "all" else [args.target]
    lines = [
        f"- Version: `{version_tag()}`",
        "",
        "| Target | State | BIN bytes | BIN SHA256 | ELF bytes |",
        "|---|---:|---:|---|---:|",
    ]
    for target in targets:
        lines.append("| " + " | ".join(f"`{cell}`" for cell in firmware_row(target)) + " |")

    write_generated(repo_root() / args.output, "AUTO Firmware Artifacts", "\n".join(lines))
    print(f"Generated {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

