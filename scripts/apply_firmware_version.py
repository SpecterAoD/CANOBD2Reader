from pathlib import Path
import os

Import("env")


def _normalise_version(raw):
    text = str(raw or "").strip()
    if not text:
        return "V0.0.0"
    return text if text.upper().startswith("V") else "V" + text


project_dir = Path(env.subst("$PROJECT_DIR"))
version_file = project_dir / "VERSION.txt"

version = os.environ.get("CANOBD2_FIRMWARE_VERSION", "")
if not version and version_file.exists():
    version = version_file.read_text(encoding="utf-8").strip()

version = _normalise_version(version)

if hasattr(env, "StringifyMacro"):
    macro_value = env.StringifyMacro(version)
else:
    macro_value = '\\"{}\\"'.format(version.replace('"', ""))

env.Append(CPPDEFINES=[("CANOBD2_FIRMWARE_VERSION", macro_value)])
print("[version] CANOBD2_FIRMWARE_VERSION={}".format(version))
