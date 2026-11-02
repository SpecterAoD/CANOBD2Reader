import hashlib
import json
import sys
import zipfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import archive_build


def test_debug_archive_contains_exact_files_and_matching_hashes(tmp_path, monkeypatch):
    build = tmp_path / ".pio/build/sender"
    build.mkdir(parents=True)
    for name in ("firmware.bin", "firmware.elf", "firmware.map"):
        (build / name).write_bytes(name.encode())
    for name in ("platformio.ini", "requirements-dev.txt"):
        (tmp_path / name).write_text(name)
    monkeypatch.setenv("PLATFORMIO_CORE_DIR", str(tmp_path / "core"))
    monkeypatch.setattr(archive_build.subprocess, "check_output",
                        lambda cmd, **kwargs: "a" * 40 if "rev-parse" in cmd else b"")
    archive_build.archive_build(tmp_path, tmp_path / "out", ["sender"], "V2.1.1-beta.4")
    with zipfile.ZipFile(tmp_path / "out/debug-sender.zip") as bundle:
        metadata = json.loads(bundle.read("build.json"))
        assert metadata["version"] == "V2.1.1-beta.4"
        assert metadata["trackedChanges"] is False
        for name, digest in metadata["sha256"].items():
            assert hashlib.sha256(bundle.read(name)).hexdigest() == digest
        assert bundle.read("firmware.elf") == b"firmware.elf"


def test_missing_elf_stops_archiving(tmp_path, monkeypatch):
    monkeypatch.setenv("PLATFORMIO_CORE_DIR", str(tmp_path / "core"))
    monkeypatch.setattr(archive_build.subprocess, "check_output", lambda *a, **k: "abc")
    with pytest.raises(FileNotFoundError):
        archive_build.archive_build(tmp_path, tmp_path / "out", ["sender"], "V2.1.1-beta.4")
