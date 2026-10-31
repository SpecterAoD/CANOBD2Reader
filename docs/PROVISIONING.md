# Provisionierung per USB (Erstinstallation, Umstieg auf v2.1)

Einmalig nötig, wenn ein Gerät neu ist, keine Zugangsdaten im NVS hat oder vom
alten Protokoll 2 kommt. Danach laufen Updates signiert per WLAN.

> **Sender und Display immer gemeinsam umstellen.** Ab V2.1 sprechen beide
> Telemetrie-Protokoll 3; ein altes Gerät versteht ein neues nicht.

## 1. Zugangsdaten erzeugen (Entwicklungsrechner)

```bash
python scripts/make_secrets.py --sender-mac AA:BB:CC:DD:EE:01 --display-mac AA:BB:CC:DD:EE:02
```

* Die MACs liefert `esptool --port /dev/ttyUSB0 read_mac` (Sender) bzw.
  `esptool --chip esp32s3 --port /dev/ttyACM0 read_mac` (Display).
* Das Skript erzeugt `include/secrets.h` mit zufälligen Passwörtern (20 Zeichen),
  API-Token (32 Zeichen) und einem zufälligen ESP-NOW-Schlüssel. Eine vorhandene
  Datei wird nie überschrieben.
* Eigene Werte sind möglich: Passwörter ≥ 8 Zeichen, API-Token ≥ 16 Zeichen.
  Der alte Beispielschlüssel `0x10, 0x32, …` ist öffentlich und wird abgelehnt.
* `include/secrets.h` ist von Git ignoriert. Eine Kopie gehört an einen sicheren
  Ort (Passwortmanager), weil sie für das nächste Provisionieren gebraucht wird.

## 2. Paket bauen (Entwicklungsrechner)

```bash
python scripts/make_provisioning_package.py --out ../CANOBD2Reader-provisioning
```

Baut `sender` und `display` **mit** den Zugangsdaten und legt Bootloader,
Partitionstabelle, otadata, Firmware und `flash.sh` in einen Ordner außerhalb
des Repositorys (Rechte 700). Der Ordner enthält die Zugangsdaten.

## 3. Flashen (beliebiger Linux-Rechner mit `esptool`)

Das offizielle esptool verwenden. Debian-/Kali-Pakete (`esptool …+dfsg`) enthalten
keine Flash-Stubs und scheitern beim Löschen. `flash.sh` nimmt automatisch
`~/.venv-esptool`, wenn vorhanden:

```bash
python3 -m venv ~/.venv-esptool && ~/.venv-esptool/bin/pip install "esptool>=4.8,<5"
```

Paket kopieren (z. B. `scp -r`), Board per USB anschließen, dann:

```bash
./flash.sh pruefen sender     # findet das Board, vergleicht die MAC – ändert nichts
./flash.sh sender             # löscht den Flash komplett und schreibt neu
./flash.sh pruefen display
./flash.sh display
./flash.sh aufraeumen         # Paket mit Zugangsdaten löschen
```

* Sender: CH340-USB (`/dev/ttyUSB*`). Das CAN-Shield darf gesteckt bleiben,
  das Fahrzeug muss nicht angeschlossen sein.
* Display: natives USB (`/dev/ttyACM*`). Wird es nicht gefunden: **BOOT**
  gedrückt halten, kurz **RESET** drücken, BOOT loslassen.
* `flash.sh` bricht ab, wenn die MAC nicht zu `secrets.h` passt (falsches Board).
* Der komplette Löschvorgang entfernt auch alte WLAN-Daten und Logs.
* Protokoll: `/tmp/canobd2-flash.log`.

## 4. Kontrolle

1. Display zeigt nach dem Start „Warte auf Sender“, sobald der Sender läuft „Verbunden“.
2. Mit dem WLAN `CANOBD2_Display` (bzw. `CANOBD2_Sender`) verbinden, Passwort aus
   `secrets.h` (`DisplayWebPassword` / `SenderWebPassword`).
3. `http://192.168.4.1` öffnen, Anmeldung `admin` / `WebPassword`.
4. Die Statuskarte zeigt „Zugangsdaten: compiled“ (direkt nach dem Flashen) bzw.
   nach einem späteren Update „stored“.
5. Optional: unter „Firmware-Update“ eine unsignierte Datei hochladen – sie muss
   abgelehnt werden.

## Spätere Updates

Unter **Firmware-Update** (`/github-update`): WLAN/Hotspot eintragen, Kanal
wählen (*stable* oder *beta*), „Nach Updates suchen“. Oder die signierte
`sender.bin` / `display.bin` aus dem GitHub-Release hochladen. Zugangsdaten
bleiben dabei erhalten. Zum Ändern eines Passworts `secrets.h` anpassen und
Schritt 2–3 wiederholen.
