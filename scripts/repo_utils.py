from __future__ import annotations

import hashlib
import os
import re
from pathlib import Path


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def write_generated(path: Path, title: str, body: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        f"# {title}\n\n"
        "> AUTO-GENERATED FILE. Do not edit manually.\n"
        "> Regenerate with the corresponding script in `scripts/`.\n\n"
    )
    path.write_text(header + body.rstrip() + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def version_tag() -> str:
    version_file = repo_root() / "VERSION.txt"
    version = read_text(version_file).strip() if version_file.exists() else "V0.0.0-dev"
    return version if version.startswith("V") else f"V{version}"


def iter_repo_files() -> list[Path]:
    root = repo_root()
    ignored_parts = {".git", ".pio", ".vscode", "build", "node_modules", "__pycache__"}
    files: list[Path] = []
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(root)
        if any(part in ignored_parts for part in rel.parts):
            continue
        files.append(path)
    return files


def markdown_links(markdown: str) -> list[str]:
    return re.findall(r"\[[^\]]+\]\(([^)#][^)]+)\)", markdown)

