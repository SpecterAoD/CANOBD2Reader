from __future__ import annotations

import re

from repo_utils import read_text, repo_root, write_generated


CONST_RE = re.compile(r"constexpr\s+(?:inline\s+)?(?:const\s+)?([^=;]+?)\s+([A-Za-z_][A-Za-z0-9_]*)\s*=")


def main() -> int:
    root = repo_root()
    lines = ["## Config headers\n"]
    for path in sorted((root / "include" / "config").glob("*.h")):
        lines.append(f"### `{path.name}`\n")
        matches = CONST_RE.findall(read_text(path))
        if not matches:
            lines.append("- No constexpr values detected.\n")
            continue
        lines.append("| Type | Name |")
        lines.append("|---|---|")
        for type_name, name in matches:
            lines.append(f"| `{type_name.strip()}` | `{name}` |")
        lines.append("")
    write_generated(root / "docs" / "AUTO_CONFIG_REFERENCE.md", "AUTO Config Reference", "\n".join(lines))
    print("Generated docs/AUTO_CONFIG_REFERENCE.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

