from __future__ import annotations

import sys

from repo_utils import iter_repo_files, repo_root


TEXT_SUFFIXES = {".h", ".hpp", ".cpp", ".c", ".py", ".ps1", ".md", ".ini", ".yml", ".yaml", ".json", ".txt"}


def main() -> int:
    root = repo_root()
    warnings: list[str] = []
    for path in iter_repo_files():
        if path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        data = path.read_bytes()
        if b"\r\n" in data and b"\n" in data.replace(b"\r\n", b""):
            warnings.append(path.relative_to(root).as_posix())
    if warnings:
        print("Mixed line endings detected:")
        for item in warnings:
            print(f"- {item}")
        return 1
    print("Line ending check OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
