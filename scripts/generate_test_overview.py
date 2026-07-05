from __future__ import annotations

from repo_utils import repo_root, write_generated


def main() -> int:
    root = repo_root()
    lines = [
        "## Native Unity test suites\n",
        "| Suite | Files | Focus |",
        "|---|---:|---|",
    ]
    for directory in sorted((root / "test").glob("test_*")):
        if not directory.is_dir():
            continue
        files = sorted(directory.glob("*.cpp"))
        focus = directory.name.removeprefix("test_").replace("_", " ")
        lines.append(f"| `{directory.name}` | {len(files)} | {focus} |")

    lines.extend(
        [
            "",
            "## Commands\n",
            "```powershell",
            "platformio test -e native",
            "platformio test -e native -f test_update_manifest",
            "```",
        ]
    )
    write_generated(root / "docs" / "AUTO_TEST_OVERVIEW.md", "AUTO Test Overview", "\n".join(lines))
    print("Generated docs/AUTO_TEST_OVERVIEW.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

