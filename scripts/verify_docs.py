from __future__ import annotations

import sys
from pathlib import Path

from repo_utils import markdown_links, read_text, repo_root


REQUIRED_DOCS = [
    "docs/README.md",
    "docs/00_Master_Design_Document.md",
    "docs/01_Architecture.md",
    "docs/16_OTA.md",
    "docs/18_Testing.md",
    "docs/19_GitHub_Actions.md",
    "README.md",
]


def main() -> int:
    root = repo_root()
    errors: list[str] = []
    for rel in REQUIRED_DOCS:
        if not (root / rel).exists():
            errors.append(f"missing required document: {rel}")

    for path in list((root / "docs").glob("*.md")) + [root / "README.md"]:
        text = read_text(path)
        if not text.lstrip().startswith("#"):
            errors.append(f"{path.relative_to(root)} does not start with a Markdown heading")
        for link in markdown_links(text):
            if "://" in link or link.startswith("mailto:") or link.startswith("#"):
                continue
            target = (path.parent / link.split("#", 1)[0]).resolve()
            try:
                target.relative_to(root.resolve())
            except ValueError:
                errors.append(f"{path.relative_to(root)} links outside repo: {link}")
                continue
            if not target.exists():
                errors.append(f"{path.relative_to(root)} broken link: {link}")

    if errors:
        print("Documentation verification failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    print("Documentation verification OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())

