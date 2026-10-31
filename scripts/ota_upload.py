#!/usr/bin/env python3
"""Upload a signed firmware image to a device over its access point.

Connect your computer to the device AP (CANOBD2_Sender / CANOBD2_Display),
then run for example:

  python scripts/ota_upload.py --host 192.168.4.1 --user admin output/sender.bin

The password is asked interactively (or taken from env CANOBD2_WEB_PASSWORD).
Unsigned images are refused locally; use --sign-key to sign a local build
first (scripts/sign_firmware.py). The device verifies the signature again.
"""

from __future__ import annotations

import argparse
import base64
import getpass
import os
import sys
import urllib.error
import urllib.request
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sign_firmware  # noqa: E402


def multipart(field: str, filename: str, data: bytes) -> tuple[bytes, str]:
    boundary = uuid.uuid4().hex
    head = (f"--{boundary}\r\nContent-Disposition: form-data; name=\"{field}\"; filename=\"{filename}\"\r\n"
            "Content-Type: application/octet-stream\r\n\r\n").encode()
    tail = f"\r\n--{boundary}--\r\n".encode()
    return head + data + tail, f"multipart/form-data; boundary={boundary}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", type=Path)
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--user", default="admin")
    parser.add_argument("--sign-key", help="sign the image with this private key before upload")
    parser.add_argument("--dry-run", action="store_true", help="only check/sign, do not upload")
    args = parser.parse_args(argv)

    data = args.image.read_bytes()
    try:
        if args.sign_key and not sign_firmware.is_signed(data):
            data = sign_firmware.sign_image(data, sign_firmware.load_private_key(args.sign_key))
        sign_firmware.verify_image(data, sign_firmware.parse_header(sign_firmware.DEFAULT_HEADER))
    except sign_firmware.SigningError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if args.dry_run:
        print(f"OK: {args.image} is signed ({len(data)} bytes), upload skipped")
        return 0

    password = os.environ.get("CANOBD2_WEB_PASSWORD") or getpass.getpass(f"Web-Passwort fuer {args.user}: ")
    body, content_type = multipart("firmware", args.image.name, data)
    request = urllib.request.Request(f"http://{args.host}/update", data=body, method="POST")
    request.add_header("Content-Type", content_type)
    token = base64.b64encode(f"{args.user}:{password}".encode()).decode()
    request.add_header("Authorization", f"Basic {token}")
    try:
        with urllib.request.urlopen(request, timeout=180) as response:
            print(response.read().decode(errors="replace"))
            return 0
    except urllib.error.HTTPError as exc:
        print(f"error: HTTP {exc.code}: {exc.read().decode(errors='replace')}", file=sys.stderr)
    except urllib.error.URLError as exc:
        print(f"error: {exc.reason}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
