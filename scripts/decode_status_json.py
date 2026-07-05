from __future__ import annotations

import argparse
import json
import urllib.request


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("url", help="Device /status or /api/... JSON URL")
    args = parser.parse_args()
    with urllib.request.urlopen(args.url, timeout=5) as response:
        data = json.loads(response.read().decode("utf-8"))
    for key in sorted(data):
        print(f"{key:28} {data[key]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

