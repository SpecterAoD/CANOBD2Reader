#!/usr/bin/env python3
"""Fail if credentials or private keys are about to be committed.

Checks every file Git would track (tracked + untracked, honouring
.gitignore) for private keys, tokens and hard-coded passwords, and makes sure
include/secrets.h is not tracked.

Usage: python scripts/check_secrets.py
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Values that are documented placeholders, not secrets.
ALLOWLIST = ("change-me", "canobd2_sender", "canobd2_display", "example", "placeholder")

SECRET_PATTERNS = [
    re.compile(r"BEGIN (RSA |EC |OPENSSH |DSA |ENCRYPTED )?PRIVATE KEY"),
    re.compile(r"AKIA[0-9A-Z]{16}"),
    re.compile(r"ghp_[A-Za-z0-9_]{30,}"),
    re.compile(r"github_pat_[A-Za-z0-9_]{30,}"),
    re.compile(r"AIza[0-9A-Za-z_-]{25,}"),
    re.compile(r"(?i)(password|passwd|api[_-]?token|secret|ssid)\s*=\s*['\"]([^'\"]{8,})['\"]"),
    re.compile(r"(?i)constexpr\s+const\s+char\*\s+\w*(password|token|secret|ssid)\w*\s*=\s*\"([^\"]{8,})\""),
]

SKIP_SUFFIXES = (".png", ".jpg", ".jpeg", ".gif", ".bin", ".elf", ".zip", ".pdf")
# Files that legitimately contain the patterns as text.
SKIP_FILES = {"include/secrets.example.h", "scripts/check_secrets.py", "scripts/sign_firmware.py",
              "test/python/test_sign_firmware.py"}


def git_files() -> list[str]:
    result = subprocess.run(["git", "ls-files", "-co", "--exclude-standard"], cwd=ROOT,
                            capture_output=True, text=True, check=True)
    return [line for line in result.stdout.splitlines() if line]


def allowed(value: str) -> bool:
    lower = value.lower()
    return value.strip() == "" or any(item in lower for item in ALLOWLIST)


def main() -> int:
    errors: list[str] = []
    tracked = subprocess.run(["git", "ls-files", "--error-unmatch", "include/secrets.h"], cwd=ROOT,
                             capture_output=True)
    if tracked.returncode == 0:
        errors.append("include/secrets.h is tracked by Git - remove it from the index")

    for rel in git_files():
        if rel in SKIP_FILES or rel.endswith(SKIP_SUFFIXES) or rel == "include/secrets.h":
            continue
        path = ROOT / rel
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern in SECRET_PATTERNS:
            for match in pattern.finditer(text):
                value = match.group(match.lastindex or 0)
                if "encodeURIComponent" in match.group(0) or allowed(value):
                    continue
                errors.append(f"{rel}: possible secret: {match.group(0)[:90]}")

    if errors:
        print("Secret check failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    print("Secret check OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
