#!/usr/bin/env python3
"""Fetch a validated index; never replace a transient download failure with {}."""
import argparse
import json
import os
import time
import urllib.error
import urllib.request
from pathlib import Path

from release_manifest import ManifestError, validate


def fetch(url, initialize=False, opener=urllib.request.urlopen, sleep=time.sleep):
    if not url.startswith("https://"):
        raise ManifestError("Index URL must use HTTPS")
    for attempt in range(3):
        try:
            with opener(url, timeout=30) as response:
                body = response.read(1024 * 1024 + 1)
            if len(body) > 1024 * 1024:
                raise ManifestError("Index exceeds 1 MiB")
            data = json.loads(body)
            problems = validate(data)
            if problems:
                raise ManifestError("Invalid existing index: " + "; ".join(problems))
            return body
        except urllib.error.HTTPError as error:
            if error.code == 404:
                if initialize:
                    return b"{}\n"
                raise ManifestError("Index missing; explicit initialize_index required") from error
            if error.code not in (408, 429, 500, 502, 503, 504):
                raise ManifestError(f"Index HTTP {error.code}") from error
            if attempt == 2:
                raise ManifestError(f"Index HTTP {error.code} after three attempts") from error
        except (urllib.error.URLError, TimeoutError) as error:
            if attempt == 2:
                raise ManifestError("Index download failed after three attempts") from error
        sleep(2 ** attempt)
    raise ManifestError("Index download failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("url")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--initialize", action="store_true")
    args = parser.parse_args()
    try:
        body = fetch(args.url, args.initialize)
        temporary = args.out.with_suffix(args.out.suffix + ".tmp")
        temporary.write_bytes(body)
        os.replace(temporary, args.out)
    except (ManifestError, ValueError, OSError) as error:
        parser.exit(1, f"Index fetch failed: {error}\n")


if __name__ == "__main__":
    main()
