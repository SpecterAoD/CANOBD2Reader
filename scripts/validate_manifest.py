from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from repo_utils import repo_root, sha256_file


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", nargs="?", default="output/firmware_manifest.json")
    args = parser.parse_args()
    root = repo_root()
    manifest_path = root / args.manifest
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    base = manifest_path.parent
    errors: list[str] = []

    for target in ("sender", "display"):
        stable = manifest.get(f"{target}_bin_stable") or ""
        expected = manifest.get(f"{target}_sha256") or ""
        if not stable:
            continue
        path = base / stable
        if not path.exists():
            errors.append(f"{stable} is missing")
            continue
        actual = sha256_file(path)
        if expected and expected.lower() != actual.lower():
            errors.append(f"{stable} SHA256 mismatch: {actual} != {expected}")

    if errors:
        print("Manifest validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    print("Manifest validation OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())

