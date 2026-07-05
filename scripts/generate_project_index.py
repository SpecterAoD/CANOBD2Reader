from __future__ import annotations

from pathlib import Path

from repo_utils import repo_root, write_generated


IMPORTANT_DIRS = [
    "include/config",
    "lib/can_router",
    "lib/isotp",
    "lib/obd",
    "lib/uds",
    "lib/telemetry",
    "lib/transport",
    "lib/runtime",
    "lib/web",
    "lib/update",
    "lib/network",
    "lib/power",
    "lib/simulation",
    "src/sender",
    "src/display",
    "test",
    ".github/workflows",
    ".github/actions",
    "scripts",
]


def tree(root: Path, max_depth: int = 3) -> str:
    lines: list[str] = []
    skip = {".git", ".pio", ".vscode", "build", "__pycache__"}
    for path in sorted(root.rglob("*")):
        rel = path.relative_to(root)
        if any(part in skip for part in rel.parts):
            continue
        if len(rel.parts) > max_depth:
            continue
        prefix = "  " * (len(rel.parts) - 1)
        marker = "/" if path.is_dir() else ""
        lines.append(f"{prefix}- {rel.name}{marker}")
    return "\n".join(lines)


def main() -> int:
    root = repo_root()
    sections = ["## Repository tree\n", "```text", tree(root), "```", "\n## Module overview\n"]
    for directory in IMPORTANT_DIRS:
        path = root / directory
        if not path.exists():
            continue
        files = sorted(p.name for p in path.iterdir() if p.is_file())
        sections.append(f"### `{directory}`\n")
        if files:
            sections.extend(f"- `{name}`" for name in files)
        else:
            sections.append("- No direct files.")
        sections.append("")
    write_generated(root / "docs" / "AUTO_PROJECT_INDEX.md", "AUTO Project Index", "\n".join(sections))
    print("Generated docs/AUTO_PROJECT_INDEX.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

