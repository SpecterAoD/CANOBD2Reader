#!/usr/bin/env python3
"""Create or extend the GitHub update manifest (update_manifest.json).

The devices read this index from the release "firmware-index". Each release
adds its entries in front of the existing ones, so older versions stay
available for a manual rollback (at most --keep entries per target).

Usage:
  python scripts/release_manifest.py \
      --previous previous.json --out output/update_manifest.json \
      --version 2.1.0 --channel stable --protocol 3 \
      --base-url https://github.com/OWNER/REPO/releases/download/firmware-V2.1.0 \
      --sender output/sender.bin --display output/display.bin

  python scripts/release_manifest.py --validate output/update_manifest.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

MANIFEST_VERSION = 1
CHANNELS = ("stable", "beta", "development")
SEMVER = re.compile(r"^\d+\.\d+\.\d+(-[0-9A-Za-z]+(\.[0-9A-Za-z]+)*)?$")


class ManifestError(Exception):
    pass


def normalize_version(raw: str) -> str:
    version = raw.strip().lstrip("vV")
    if not SEMVER.match(version):
        raise ManifestError(f"Version '{raw}' is not SemVer (e.g. 2.1.0 or 2.1.0-beta.1)")
    return version


def sha256_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_previous(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (ValueError, OSError):
        return {}
    return data if isinstance(data, dict) else {}


def build_manifest(previous: dict, version: str, channel: str, protocol: int, base_url: str,
                   files: dict[str, Path], keep: int, created_at: str) -> dict:
    if channel not in CHANNELS:
        raise ManifestError(f"Unknown channel '{channel}'")
    tag = "V" + version
    manifest = {
        "manifestVersion": MANIFEST_VERSION,
        "project": "CANOBD2Reader",
        "generatedAt": created_at,
        "sender": [],
        "display": [],
    }
    for target in ("sender", "display"):
        old = previous.get(target, [])
        old = [e for e in old if isinstance(e, dict) and e.get("version", "").lstrip("vV") != version]
        entries = []
        if target in files:
            entries.append({
                "version": tag,
                "channel": channel,
                "target": target,
                "protocol": protocol,
                "url": f"{base_url.rstrip('/')}/{target}.bin",
                "sha256": sha256_of(files[target]),
                "signed": True,
                "createdAt": created_at,
            })
        manifest[target] = (entries + old)[:keep]
    return manifest


def validate(manifest: dict) -> list[str]:
    problems = []
    if manifest.get("manifestVersion") != MANIFEST_VERSION:
        problems.append("manifestVersion must be 1")
    for target in ("sender", "display"):
        for index, entry in enumerate(manifest.get(target, [])):
            where = f"{target}[{index}]"
            for key in ("version", "channel", "url", "protocol"):
                if key not in entry:
                    problems.append(f"{where}: missing {key}")
            if entry.get("target", target) != target:
                problems.append(f"{where}: target mismatch")
            if entry.get("channel") not in CHANNELS:
                problems.append(f"{where}: invalid channel")
            if not str(entry.get("url", "")).startswith("https://"):
                problems.append(f"{where}: url must be https")
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--validate", metavar="FILE")
    parser.add_argument("--previous", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--version")
    parser.add_argument("--channel", default="stable")
    parser.add_argument("--protocol", type=int)
    parser.add_argument("--base-url")
    parser.add_argument("--sender", type=Path)
    parser.add_argument("--display", type=Path)
    parser.add_argument("--keep", type=int, default=20)
    args = parser.parse_args(argv)

    try:
        if args.validate:
            problems = validate(json.loads(Path(args.validate).read_text(encoding="utf-8")))
            for problem in problems:
                print(f"error: {problem}", file=sys.stderr)
            return 1 if problems else 0

        if not (args.out and args.version and args.protocol and args.base_url):
            raise ManifestError("--out, --version, --protocol and --base-url are required")
        files = {name: path for name, path in (("sender", args.sender), ("display", args.display)) if path}
        if not files:
            raise ManifestError("at least one of --sender/--display is required")
        manifest = build_manifest(
            load_previous(args.previous), normalize_version(args.version), args.channel, args.protocol,
            args.base_url, files, args.keep, datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"))
        problems = validate(manifest)
        if problems:
            raise ManifestError("; ".join(problems))
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(f"Wrote {args.out}: sender={len(manifest['sender'])} display={len(manifest['display'])} entries")
        return 0
    except (ManifestError, OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
