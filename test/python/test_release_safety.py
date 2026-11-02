import io
import json
import sys
import urllib.error
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import fetch_previous_manifest as download
import release_manifest as manifest
import check_secrets

URL = "https://example.org/index.json"


def test_download_retries_then_preserves_exact_valid_index():
    body = b'{"manifestVersion":1,"sender":[],"display":[]}'
    attempts = []

    def opener(url, timeout):
        attempts.append(url)
        if len(attempts) < 3:
            raise urllib.error.HTTPError(url, 503, "unavailable", {}, None)
        return io.BytesIO(body)

    assert download.fetch(URL, opener=opener, sleep=lambda _: None) == body
    assert len(attempts) == 3


@pytest.mark.parametrize("initialize", [False, True])
def test_network_failure_never_initializes_index(initialize):
    attempts = []

    def opener(url, timeout):
        attempts.append(url)
        raise urllib.error.URLError("offline")

    with pytest.raises(manifest.ManifestError):
        download.fetch(URL, initialize, opener=opener, sleep=lambda _: None)
    assert len(attempts) == 3


def test_missing_index_requires_explicit_initialization():
    def missing(url, timeout):
        raise urllib.error.HTTPError(url, 404, "missing", {}, None)

    with pytest.raises(manifest.ManifestError):
        download.fetch(URL, opener=missing)
    assert json.loads(download.fetch(URL, True, opener=missing)) == {}


@pytest.mark.parametrize("body", [b"broken", b"[]", b"{}", b'{"manifestVersion":1,"sender":{}}'])
def test_invalid_download_is_not_replaced_by_empty_index(body):
    with pytest.raises((manifest.ManifestError, ValueError)):
        download.fetch(URL, True, opener=lambda *a, **k: io.BytesIO(body))


def test_unreadable_previous_manifest_stops_release(tmp_path):
    path = tmp_path / "previous.json"
    with pytest.raises(manifest.ManifestError):
        manifest.load_previous(path)
    path.write_text("{")
    with pytest.raises(manifest.ManifestError):
        manifest.load_previous(path)


def test_secret_checker_reports_location_without_value(tmp_path, monkeypatch, capsys):
    # Deliberately fabricated token assembled at runtime; no actual credential.
    fake = "ghp_" + "z" * 36
    (tmp_path / "leak.txt").write_text("line one\n" + fake)
    monkeypatch.setattr(check_secrets, "ROOT", tmp_path)
    monkeypatch.setattr(check_secrets, "git_files", lambda: ["leak.txt"])
    class Result:
        returncode = 1
    monkeypatch.setattr(check_secrets.subprocess, "run", lambda *a, **k: Result())
    assert check_secrets.main() == 1
    output = capsys.readouterr().out
    assert "leak.txt:2:" in output
    assert "redacted" in output
    assert fake not in output
