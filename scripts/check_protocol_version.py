from __future__ import annotations

import re
import sys

from repo_utils import read_text, repo_root


def main() -> int:
    root = repo_root()
    platformio = read_text(root / "platformio.ini")
    config = read_text(root / "include" / "config" / "ProjectConfig.h")
    pio_versions = set(re.findall(r"CANOBD2_PROTOCOL_VERSION=(\d+)", platformio))
    cfg_versions = set(re.findall(r"ProtocolVersion\s*=\s*(\d+)", config))
    all_versions = pio_versions | cfg_versions
    if len(all_versions) != 1:
        print(f"Protocol version mismatch: platformio={pio_versions}, config={cfg_versions}")
        return 1
    print(f"Protocol version OK: {next(iter(all_versions))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

