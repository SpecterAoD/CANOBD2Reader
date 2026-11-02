# Sicherheit, Zugangsdaten und Updates

Das Repository ist öffentlich. Deshalb gilt: **keine Zugangsdaten im Code
oder in Release-Images**, und **nur signierte Firmware wird installiert**.

## Zugangsdaten (Provisionierung)

Jedes Gerät speichert AP-Passwort, Web-Login, API-Token, ESP-NOW-Schlüssel und
die Partner-MAC im NVS (`lib/security/DeviceSecrets`).

Schritt für Schritt: [PROVISIONING.md](PROVISIONING.md).

1. `include/secrets.h` mit `scripts/make_secrets.py` erzeugen oder aus
   `include/secrets.example.h` kopieren (von Git ignoriert) und **alle** Werte ersetzen. Passwörter ≥ 8 Zeichen, API-Token
   ≥ 16 Zeichen, ESP-NOW-Schlüssel zufällig erzeugen.
2. MAC-Adressen eintragen: Sie stehen nach dem Booten im Diagnose-Log bzw. auf
   der seriellen Konsole (`[BOOT] mac=...`).
3. Sender und Display **einmal per USB** mit dieser lokalen Firmware flashen
   (`pio run -e sender -t upload`, `pio run -e display -t upload`).
4. Ab jetzt behalten signierte Updates (Web oder GitHub) die gespeicherten
   Werte. Zum Ändern eines Passworts `secrets.h` anpassen und erneut per USB
   flashen: Eine lokal gebaute Firmware mit gültigen Werten überschreibt das NVS.

Ohne gültige Zugangsdaten startet ein Gerät **weder Access Point noch
Webserver noch ESP-NOW**. Platzhalter (`change-me…`, Null-Schlüssel, der alte
Beispielschlüssel, fehlende MAC) gelten als ungültig.

Einschränkung: Ohne ESP32-Flash-Verschlüsselung kann jemand mit physischem
Zugriff auf die Platine das NVS auslesen.

## Signierte Firmware

* Format: Das ESP32-Image bekommt einen 88-Byte-Trailer mit einer
  Ed25519-Signatur über `CANOBD2-FIRMWARE-SIGNATURE-V1 || SHA-512(Image)`
  (`lib/firmware_signing/FirmwareSignature.h`).
* Prüfung: `FirmwareUpdate::OtaSession` hasht während des Schreibens. Erst wenn
  Signatur **und** eingebettete Metadaten (Gerätetyp, Protokoll, Schema)
  stimmen, wird die neue Partition aktiviert, sonst wird abgebrochen.
* Wege: Web-Upload (`/github-update` → „Datei hochladen“ oder
  `scripts/ota_upload.py`) und GitHub-Update. Ein Update per USB-Kabel ist
  immer möglich (physischer Zugriff).
* Vor Beginn eines Updates muss ein höchstens 3 s alter Zustand `Parked` oder
  `DisplaySleep` vorliegen. `Booting`, unbekannte Zustände und ein veralteter
  Senderstatus sperren Updates ebenfalls. Die Freigabe wird vor dem Flashen geprüft;
  während eines laufenden synchronen Uploads/Downloads wird keine neue
  Fahrzeugzustandsmessung garantiert. Das Fahrzeug muss bis zum Neustart stehen bleiben.
* Für Wartung ohne Fahrzeug: Gerät vom Fahrzeug trennen, auf der Update-Seite den
  Wartungsmodus ausdrücklich bestätigen. Er gilt höchstens 10 Minuten, wird nicht
  gespeichert und bei einem neuen aktiven Fahrzeugzustand sofort aufgehoben.
  API: authentifiziertes `POST /api/update/maintenance` mit
  `confirm=vehicle-disconnected` und Header `X-Maintenance-Confirm: vehicle-disconnected`;
  `enable=0` mit demselben Header beendet ihn. Der zusätzliche Header verhindert
  die Aktivierung über fremde HTML-Formulare mit gespeicherter Browser-Anmeldung. Signatur und Anmeldung
  bleiben auch im Wartungsmodus erforderlich.
* **Automatischer Rollback:** Neue Firmware gilt erst nach 30 s stabilem Lauf
  mit erfolgreicher lokaler Initialisierung als bestätigt (`lib/update/BootValidation`).
  Geprüft werden Logspeicher, CAN-Treiber bzw. Display-Framebuffer sowie bei
  provisionierten Geräten Webserver und ESP-NOW-Initialisierung. Fehlende externe
  ECU-/Senderantworten sind kein Fehler dieser Boot-Prüfung. Bleibt die lokale
  Initialisierung 60 s lang unzureichend, wird die vorherige gültige Firmware
  angefordert. Ohne gültigen Rückweg wird der Fehler protokolliert; soweit erfolgreich
  initialisiert, bleibt die Diagnose-/Wartungsoberfläche erreichbar.
  Ein Absturz vor der Bestätigung führt ebenfalls zum Bootloader-Rollback.

* **Watchdog:** Hängt `loop()` länger als 30 s, startet das Gerät neu. Passiert
  das mit einer frisch installierten Firmware innerhalb der ersten 30 s, greift
  ebenfalls der Rollback.

### Schlüsselverwaltung

* Öffentliche Schlüssel: `include/config/FirmwareSigningKeys.h` (im Repo).
* Privater Schlüssel: nur beim Maintainer und als GitHub-Actions-Secret
  `FIRMWARE_SIGNING_KEY` (PEM-Text) im Environment `release`.
* Neuer Schlüssel:
  `python scripts/sign_firmware.py keygen --private-key <pfad außerhalb des Repos>`
* Rotation: neuen Schlüssel mit
  `export-header --private-key neu.pem --keep-existing` zusätzlich eintragen,
  eine Version mit beiden Schlüsseln ausrollen, danach den alten entfernen.
* Lokale Builds signieren:
  `python scripts/sign_firmware.py sign .pio/build/sender/firmware.bin sender.bin --private-key …`

## GitHub-Updates

Das Manifest liegt im Release `firmware-index` und wird über HTTPS mit
festgelegten Root-Zertifikaten geladen (`include/config/UpdateConfig.h`):
USERTrust ECC/Sectigo E46 für github.com und ISRG Root X1/X2 für die Asset-CDN.
Die Echtheit eines Updates hängt aber an der Signatur, nicht an TLS.
Standardkanal ist *stable*; ältere Versionen bleiben im Manifest für einen
manuellen Rollback.

## Weboberfläche

* HTTP Basic Auth oder API-Token im Header `X-API-Token` /
  `Authorization: Bearer`. Tokens in der URL werden nicht akzeptiert.
* Alle dynamischen Werte werden im Browser HTML-escaped.
* Die Oberfläche läuft über HTTP im WPA2-geschützten Geräte-AP.
