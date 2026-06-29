# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUndefinedVariable=false

from SCons.Script import COMMAND_LINE_TARGETS

Import("env")


def _get_option(name, default=""):
	try:
		return env.GetProjectOption(name, default)
	except Exception:
		return default


def _parse_hex(value):
	if value is None:
		return None
	text = str(value).strip()
	if not text:
		return None
	try:
		return int(text, 16)
	except ValueError:
		return None


def _needs_auto_port():
	if "upload" not in COMMAND_LINE_TARGETS:
		return False

	protocol = str(_get_option("upload_protocol", "")).strip().lower()
	if protocol == "espota":
		return False

	current_port = str(env.subst("$UPLOAD_PORT")).strip()
	if current_port:
		# CLI --upload-port oder fest konfigurierter Port hat Vorrang.
		return False

	return True


def _select_upload_port():
	vid = _parse_hex(_get_option("custom_upload_vid", ""))
	pid = _parse_hex(_get_option("custom_upload_pid", ""))
	serial_expected = str(_get_option("custom_upload_serial", "")).strip().lower()

	if vid is None or pid is None:
		return None

	try:
		from serial.tools import list_ports
	except Exception:
		print("[auto_upload_port] pyserial nicht verfuegbar, Auto-Port wird uebersprungen")
		return None

	matches = []
	for port in list_ports.comports():
		if port.vid != vid or port.pid != pid:
			continue

		serial_number = (port.serial_number or "").strip().lower()
		if serial_expected and serial_number != serial_expected:
			continue

		matches.append(port)

	if len(matches) == 1:
		selected = matches[0].device
		print(
			"[auto_upload_port] {}: {} (VID:PID={:04X}:{:04X})".format(
				env["PIOENV"], selected, vid, pid
			)
		)
		return selected

	if not matches:
		print(
			"[auto_upload_port] Kein passendes Geraet gefunden fuer {} (VID:PID={:04X}:{:04X})".format(
				env["PIOENV"], vid, pid
			)
		)
		return None

	print(
		"[auto_upload_port] Mehrere passende Ports fuer {} gefunden, bitte upload_port explizit setzen".format(
			env["PIOENV"]
		)
	)
	for item in matches:
		print("  - {} ({})".format(item.device, item.description))
	return None


if _needs_auto_port():
	resolved = _select_upload_port()
	if resolved:
		env.Replace(UPLOAD_PORT=resolved)
