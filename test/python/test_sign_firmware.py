"""Tests for scripts/sign_firmware.py (run: python -m pytest test/python)."""

import sys
from pathlib import Path

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import sign_firmware as sf  # noqa: E402

IMAGE = bytes([0xE9]) + bytes(range(255)) * 20


@pytest.fixture()
def key():
    return Ed25519PrivateKey.generate()


def test_sign_and_verify_roundtrip(key):
    signed = sf.sign_image(IMAGE, key)
    assert len(signed) == len(IMAGE) + sf.TRAILER_SIZE
    assert sf.verify_image(signed, [sf.raw_public_key(key)]) == sf.key_id(sf.raw_public_key(key))


def test_tampered_image_fails(key):
    signed = bytearray(sf.sign_image(IMAGE, key))
    signed[100] ^= 1
    with pytest.raises(sf.SigningError, match="invalid"):
        sf.verify_image(bytes(signed), [sf.raw_public_key(key)])


def test_wrong_key_fails(key):
    signed = sf.sign_image(IMAGE, key)
    other = sf.raw_public_key(Ed25519PrivateKey.generate())
    with pytest.raises(sf.SigningError, match="Unknown"):
        sf.verify_image(signed, [other])


def test_double_signing_and_non_esp_images_are_refused(key):
    with pytest.raises(sf.SigningError):
        sf.sign_image(sf.sign_image(IMAGE, key), key)
    with pytest.raises(sf.SigningError):
        sf.sign_image(b"\x00" + IMAGE[1:], key)


def test_header_roundtrip(tmp_path, key):
    header = tmp_path / "keys.h"
    header.write_text(sf.render_header([sf.raw_public_key(key)]))
    assert sf.parse_header(header) == [sf.raw_public_key(key)]


def test_repository_header_is_parseable():
    assert len(sf.parse_header(sf.DEFAULT_HEADER)) >= 1


def test_cli_sign_and_verify(tmp_path, key, monkeypatch):
    from cryptography.hazmat.primitives import serialization

    pem = key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                            serialization.NoEncryption()).decode()
    monkeypatch.setenv(sf.ENV_KEY, pem)
    header = tmp_path / "keys.h"
    header.write_text(sf.render_header([sf.raw_public_key(key)]))
    src, out = tmp_path / "fw.bin", tmp_path / "signed.bin"
    src.write_bytes(IMAGE)
    assert sf.main(["sign", str(src), str(out)]) == 0
    assert sf.main(["verify", str(out), "--header", str(header)]) == 0
    assert sf.main(["verify", str(src), "--header", str(header)]) == 1
