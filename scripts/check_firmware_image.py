#!/usr/bin/env python3
"""Check that a built firmware image carries the OTA metadata record.

Devices reject OTA images without a valid record, so a release must never
contain an image where the linker dropped it.

Usage: python scripts/check_firmware_image.py .pio/build/sender/firmware.bin sender [--version V2.1.0]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

RECORD = re.compile(rb"CANOBD2_FW_METADATA_BEGIN;([\x20-\x7e]{1,320}?);CANOBD2_FW_METADATA_END")


def read_record(data: bytes) -> dict[str, str] | None:
    match = RECORD.search(data)
    if not match:
        return None
    fields = {}
    for item in match.group(1).decode().split(";"):
        key, _, value = item.partition("=")
        fields[key] = value
    return fields


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", type=Path)
    parser.add_argument("target", choices=["sender", "display"])
    parser.add_argument("--version")
    args = parser.parse_args(argv)

    record = read_record(args.image.read_bytes())
    if record is None:
        print(f"error: {args.image}: no CANOBD2 metadata record", file=sys.stderr)
        return 1
    if record.get("target") != args.target:
        print(f"error: {args.image}: target is {record.get('target')!r}, expected {args.target!r}", file=sys.stderr)
        return 1
    if args.version and record.get("version") != args.version:
        print(f"error: {args.image}: version {record.get('version')!r} != {args.version!r}", file=sys.stderr)
        return 1
    print(f"OK: {args.image}: {record}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
