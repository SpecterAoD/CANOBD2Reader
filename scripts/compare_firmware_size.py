from __future__ import annotations

import json
from pathlib import Path

from repo_utils import repo_root, write_generated


def size_for(target: str) -> int:
    path = repo_root() / ".pio" / "build" / target / "firmware.bin"
    return path.stat().st_size if path.exists() else 0


def main() -> int:
    root = repo_root()
    state_path = root / "build" / "firmware_size_baseline.json"
    current = {"sender": size_for("sender"), "display": size_for("display")}
    previous = json.loads(state_path.read_text()) if state_path.exists() else {}
    state_path.parent.mkdir(exist_ok=True)
    state_path.write_text(json.dumps(current, indent=2), encoding="utf-8")

    lines = [
        "| Target | Current bytes | Previous bytes | Delta |",
        "|---|---:|---:|---:|",
    ]
    for target, size in current.items():
        old = int(previous.get(target, 0) or 0)
        lines.append(f"| `{target}` | {size} | {old} | {size - old:+d} |")
    write_generated(root / "docs" / "AUTO_FIRMWARE_SIZE.md", "AUTO Firmware Size Report", "\n".join(lines))
    print("Generated docs/AUTO_FIRMWARE_SIZE.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

