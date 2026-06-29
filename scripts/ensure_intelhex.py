from importlib.util import find_spec
import subprocess
import sys


if find_spec("intelhex") is None:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "intelhex"])