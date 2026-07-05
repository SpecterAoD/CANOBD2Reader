from __future__ import annotations

import configparser
from io import StringIO

from repo_utils import read_text, repo_root, write_generated


def main() -> int:
    root = repo_root()
    text = read_text(root / "platformio.ini")
    parser = configparser.ConfigParser(interpolation=None, allow_no_value=True)
    parser.optionxform = str
    parser.read_file(StringIO(text))

    lines = ["## PlatformIO sections\n"]
    for section in parser.sections():
        lines.append(f"### `[{section}]`\n")
        for key, value in parser.items(section):
            value = "" if value is None else value.strip()
            if "\n" in value:
                lines.append(f"- `{key}`:\n\n```text\n{value}\n```\n")
            else:
                lines.append(f"- `{key}`: `{value}`")
        lines.append("")

    lines.append("## Common commands\n")
    lines.extend(
        [
            "```powershell",
            "platformio run -e sender",
            "platformio run -e display",
            "platformio test -e native",
            "platformio check -e sender",
            "platformio check -e display",
            "```",
        ]
    )
    write_generated(root / "docs" / "AUTO_BUILD_FLAGS.md", "AUTO Build Flags", "\n".join(lines))
    print("Generated docs/AUTO_BUILD_FLAGS.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

