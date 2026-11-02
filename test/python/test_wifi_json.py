"""Parse the real native WLAN status serializer, with persistence replaced only."""
import json
import shutil
import subprocess
from pathlib import Path

import pytest


def test_ssid_survives_real_status_json(tmp_path):
    compiler = shutil.which("g++")
    if not compiler:
        pytest.skip("native C++ compiler unavailable")
    root = Path(__file__).resolve().parents[2]
    source = tmp_path / "wifi.cpp"
    source.write_text(r'''
#include <iostream>
#include "WifiStationManager.h"
#include "WifiCredentialStore.h"
namespace WifiNet {
WifiCredentials WifiCredentialStore::load() { return {"Test\"Net\\\n\t", "", true}; }
bool WifiCredentialStore::save(const String&, const String&) { return true; }
bool WifiCredentialStore::clear() { return true; }
}
int main() {
    WifiNet::WifiStationManager::begin();
    std::cout << WifiNet::WifiStationManager::statusJson();
}
''')
    executable = tmp_path / "wifi"
    subprocess.run([compiler, "-std=c++17", "-Iinclude", "-Ilib/utils", "-Ilib/wifi_station",
                    str(source), "lib/wifi_station/WifiStationManager.cpp", "-o", str(executable)],
                   cwd=root, check=True, capture_output=True)
    status = json.loads(subprocess.check_output([str(executable)], text=True))
    assert status["ssid"] == 'Test"Net\\\n\t'
    assert status["connected"] is False
