from __future__ import annotations

import argparse
import subprocess

from repo_utils import repo_root


def git_commits(limit: int) -> list[str]:
    result = subprocess.run(
        ["git", "log", f"--max-count={limit}", "--pretty=format:- `%h` %s"],
        cwd=repo_root(),
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout.splitlines() if result.returncode == 0 else []


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", required=True)
    parser.add_argument("--target", default="all")
    parser.add_argument("--channel", default="test")
    parser.add_argument("--commits", type=int, default=20)
    parser.add_argument("--output", default="RELEASE_NOTES.md")
    args = parser.parse_args()

    lines = [
        f"# CAN OBD2 Firmware {args.version}",
        "",
        f"- Channel: `{args.channel}`",
        f"- Target: `{args.target}`",
        "- Stable release: only use the release workflow for production firmware.",
        "- Test firmware: use test-build artifacts; do not publish as a normal release.",
        "",
        "## Recent commits",
        "",
        *(git_commits(args.commits) or ["- No Git history available."]),
        "",
    ]
    out = repo_root() / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"Generated {out.relative_to(repo_root())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

