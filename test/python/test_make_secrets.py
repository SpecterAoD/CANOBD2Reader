"""Tests for scripts/make_secrets.py."""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import make_secrets as ms  # noqa: E402


def test_generated_values_are_strong_and_random(tmp_path):
    out = tmp_path / "secrets.h"
    assert ms.main(["--sender-mac", "8C:4B:14:00:00:01", "--display-mac", "f0-f5-bd-00-00-02", "--out", str(out)]) == 0
    text = out.read_text()
    passwords = re.findall(r'Password = "([^"]*)"', text)
    assert len([p for p in passwords if p]) == 3 and all(len(p) == 20 for p in passwords if p)
    assert len(re.search(r'ApiToken = "([^"]*)"', text).group(1)) == 32
    assert "0x8C, 0x4B, 0x14, 0x00, 0x00, 0x01" in text and "0xF0, 0xF5, 0xBD, 0x00, 0x00, 0x02" in text
    assert "change-me" not in text
    other = tmp_path / "b.h"
    ms.main(["--sender-mac", "8C:4B:14:00:00:01", "--display-mac", "F0:F5:BD:00:00:02", "--out", str(other)])
    assert out.read_text() != other.read_text()


def test_existing_file_is_not_overwritten_and_bad_mac_rejected(tmp_path):
    out = tmp_path / "secrets.h"
    out.write_text("keep")
    assert ms.main(["--sender-mac", "8C:4B:14:00:00:01", "--display-mac", "F0:F5:BD:00:00:02", "--out", str(out)]) == 1
    assert out.read_text() == "keep"
    assert ms.main(["--sender-mac", "nope", "--display-mac", "F0:F5:BD:00:00:02", "--out", str(tmp_path / "x")]) == 1
