# Änderungen

## V2.1.1-beta.4

- Updates bei unbekanntem/veraltetem Fahrzeugstatus und während Booting sperren.
  Authentifizierter Wartungsmodus für das vom Fahrzeug getrennte Gerät, maximal
  zehn Minuten, bei aktivem Fahrzeugzustand automatisch aufgehoben.
- Neue Firmware erst nach 30 Sekunden erfolgreicher lokaler Initialisierung
  bestätigen; bei anhaltendem Initialisierungsfehler Rollback nach 60 Sekunden.
  Fehlende externe CAN-/Funkantworten lösen keinen Boot-Rollback aus.
- Messwerte nach 60 Sekunden ohne Messung im Snapshot ungültig machen, auch wenn
  weiterhin Funkpakete eintreffen. Protokollformat bleibt v3.
- PID-Gültigkeit bis zur Verarbeitung erhalten; ungültige Antworten zählen und
  nicht als frische Werte oder als Aktivitätsdaten übernehmen.
- Durchschnittsverbrauch aus zeitintegrierter Kraftstoffmenge und Strecke bilden;
  Nullverbrauch und Kraftstoff im Stand berücksichtigen, Messlücken auslassen.
- Frischegrenzen für Kühlmitteltemperatur und ECU-Spannung an langsame Abfrage
  anpassen. JSON-Escaping für WLAN, Management und Updates vereinheitlichen.
- Sender-/Display-Webseiten von den Routen trennen; Browser-Helfer teilen.
  Display-Rendering von Hardwareinitialisierung und Tastersteuerung trennen.
- Release-Index bei Downloadfehlern erhalten; begrenzte Retries und ausdrückliche
  Erstinitialisierung. Vorherigen Index als Release-Anhang sichern.
- Exakte ELF-/MAP-/BIN-Dateien mit Build-Metadaten und Prüfsummen archivieren;
  Debug-Archive dauerhaft an Releases anhängen, CI-Aufbewahrung 90 Tage.
- Secret-Scanner meldet Fundort und Regel ohne Geheimwerte. Veraltete espota-
  Anleitung entfernt; Regressionstests und Betriebsdokumentation ergänzt.

Hardwareprüfungen für OTA, Rollback, CAN und Display sind vor dem produktiven
Einsatz erforderlich. Einträge beschreiben den Quellstand, keine Geräteinstallation.
