from __future__ import annotations

import re
import sys
from pathlib import Path

from repo_utils import iter_repo_files, read_text, repo_root


ALLOWLIST = {
    "change-me",
    "CANOBD2_Sender_Setup",
    "CANOBD2_Display_Setup",
    "example",
    "placeholder",
}

SECRET_PATTERNS = [
    re.compile(r"BEGIN (RSA |EC |OPENSSH |DSA )?PRIVATE KEY"),
    re.compile(r"AKIA[0-9A-Z]{16}"),
    re.compile(r"ghp_[A-Za-z0-9_]{30,}"),
    re.compile(r"github_pat_[A-Za-z0-9_]{30,}"),
    re.compile(r"AIza[0-9A-Za-z_-]{25,}"),
    re.compile(r"(?i)(password|passwd|api[_-]?token|secret|ssid)\s*=\s*['\"]([^'\"]{8,})['\"]"),
    re.compile(r"(?i)constexpr\s+const\s+char\*\s+\w*(password|token|secret|ssid)\w*\s*=\s*\"([^\"]{8,})\""),
]


def allowed(value: str) -> bool:
    lower = value.lower()
    return any(item.lower() in lower for item in ALLOWLIST) or value.strip() == ""


def main() -> int:
    root = repo_root()
    errors: list[str] = []

    if (root / "include" / "secrets.h").exists():
        # secrets.h may exist locally for development, but it must not be tracked.
        import subprocess

        tracked = subprocess.run(
            ["git", "ls-files", "--error-unmatch", "include/secrets.h"],
            cwd=root,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if tracked.returncode == 0:
            errors.append("include/secrets.h is tracked by Git. Remove it from the index.")

    for path in iter_repo_files():
        rel = path.relative_to(root).as_posix()
        if rel.endswith((".png", ".jpg", ".jpeg", ".gif", ".bin", ".elf", ".zip", ".pdf")):
            continue
        if rel == "include/secrets.example.h":
            continue
        text = read_text(path)
        for pattern in SECRET_PATTERNS:
            for match in pattern.finditer(text):
                value = match.group(match.lastindex or 0)
                snippet = match.group(0)
                if "encodeURIComponent" in snippet:
                    continue
                if not allowed(value):
                    errors.append(f"{rel}: possible secret: {snippet[:90]}")

    if errors:
        print("Secret check failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("Secret check OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
