"""Tests for scripts/release_manifest.py."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import release_manifest as rm  # noqa: E402

BASE = "https://github.com/o/r/releases/download/firmware-V2.1.0"


def write_bin(tmp_path, name):
    path = tmp_path / name
    path.write_bytes(b"\xe9" + name.encode())
    return path


def test_new_entry_is_first_and_old_versions_stay(tmp_path):
    previous = {"sender": [{"version": "V2.0.0", "channel": "stable", "target": "sender", "protocol": 2,
                            "url": "https://x/sender.bin"}], "display": []}
    files = {"sender": write_bin(tmp_path, "sender.bin")}
    manifest = rm.build_manifest(previous, "2.1.0", "stable", 3, BASE, files, 20, "now")
    assert [e["version"] for e in manifest["sender"]] == ["V2.1.0", "V2.0.0"]
    assert manifest["sender"][0]["url"] == BASE + "/sender.bin"
    assert manifest["sender"][0]["signed"] is True
    assert rm.validate(manifest) == []


def test_rerelease_replaces_same_version_and_keep_limit(tmp_path):
    previous = {"sender": [{"version": f"V1.0.{i}", "channel": "stable", "target": "sender", "protocol": 2,
                            "url": "https://x"} for i in range(30)] + [
        {"version": "V2.1.0", "channel": "beta", "target": "sender", "protocol": 3, "url": "https://old"}]}
    manifest = rm.build_manifest(previous, "2.1.0", "stable", 3, BASE, {"sender": write_bin(tmp_path, "s.bin")},
                                 5, "now")
    versions = [e["version"] for e in manifest["sender"]]
    assert versions.count("V2.1.0") == 1 and len(versions) == 5


def test_version_must_be_semver():
    assert rm.normalize_version("V2.1.0-beta.1") == "2.1.0-beta.1"
    with pytest.raises(rm.ManifestError):
        rm.normalize_version("2.0.0.b6")


def test_validate_finds_problems():
    problems = rm.validate({"manifestVersion": 1, "sender": [{"version": "V1.0.0", "channel": "nightly",
                                                              "url": "http://x", "protocol": 3}]})
    assert any("channel" in p for p in problems) and any("https" in p for p in problems)


def test_cli_roundtrip(tmp_path):
    out = tmp_path / "m.json"
    args = ["--out", str(out), "--version", "2.1.0", "--protocol", "3", "--base-url", BASE,
            "--display", str(write_bin(tmp_path, "display.bin"))]
    assert rm.main(args) == 0
    assert json.loads(out.read_text())["display"][0]["target"] == "display"
    assert rm.main(["--validate", str(out)]) == 0
