# Hardware

## Sender – ESP32 DevKit V1 mit CAN-Shield (MrDIY)

| Funktion | GPIO |
|---|---|
| CAN TX / RX (TWAI) | 5 / 4 |
| LED grün / Fehler | 26 / 27 |
| Taster (LED-Test) | 25 |
| Spannungsteiler Bordnetz | 32 (Faktor 4,81) |

Partitionslayout `partitions/sender_4mb.csv`: 2 × 1,81 MB Firmware, 64 KB
Absturzbericht, 256 KB Log.

CAN: 500 kbit/s, OBD-Anfragen funktional an `0x7DF`, nach wiederholten
Timeouts physisch an `0x7E0`. Nach Bus-Off startet der Treiber automatisch neu.
CAN-Fehler (z. B. am Tisch ohne Fahrzeug: „error passive“, „Bus error“) werden
höchstens alle 5 s als Sammelzeile protokolliert. Antwortet das Display nicht,
sendet der Sender nur noch ein Paket pro Sekunde, bis es wieder quittiert.

### LED-Zustände

| Zustand | Grün | Fehler | Bedeutung |
|---|---|---|---|
| Booting | langsam blinkend | aus | Start |
| Ready / CanActive / ObdActive | an | aus | Normalbetrieb |
| VehicleOff | langsam blinkend | aus | CAN war aktiv, jetzt Ruhe (Fahrzeug aus) |
| ObdTimeout | schnell blinkend | aus | ECU antwortet nicht |
| EspNowError | aus | schnell blinkend | Funk nicht bereit / Sendefehler |
| CanError | aus | an | CAN-Treiber gestört |
| LedTest | an | an | Taster gedrückt |

## Display – LilyGO T-Display S3

ST7789 170×320 an 8-Bit-Parallelbus (Pins in `platformio.ini`), Hintergrundbeleuchtung GPIO 38,
Partitionslayout `partitions/display_16mb.csv`: 2 × 4 MB Firmware, 128 KB
Absturzbericht, 1 MB Log, Rest frei. Displayversorgung GPIO 15, Tasten GPIO 0 (BOOT) und GPIO 14.

Tasten: BOOT (GPIO 0) kurz = nächste Seite, zweite Taste (GPIO 14) kurz = vorherige
Seite, eine der beiden lang (1,2 s) = Hauptseite, BOOT sehr lang (3 s) = Diagnoseseiten ein/aus.

| Seite | Inhalt |
|---|---|
| 1 Fahren | Tempo, Drehzahl (mit Balken), Kühlmittel, Bordnetz, Öl, Fehlerstatus |
| 2 Motor | Öl, Kühlmittel, Last, Ansaugluft, Ladedruck, Saugrohr, Drossel |
| 3 Verbrauch | Momentan, Schnitt, L/h, Tempo, Tank, Laufzeit, Außentemperatur |
| 4 Verbindung | Funkqualität, Paketzähler, Heartbeat, CAN/OBD, Firmware |
| 5–8 (Diagnose) | DTC/VIN/UDS, CAN-Sniffer (Dev-Build), weitere Werte, Power-Manager |

Graue Werte sind veraltet (letzter bekannter Wert), `--` heißt noch nie
empfangen, `n.v.` heißt nicht unterstützt.

## Stromversorgung / Power-Manager

VW-MQB-Evo-Fahrzeuge halten Steuergeräte nach Zündung-aus minutenlang wach und
nutzen Start-Stopp intensiv. Der Power-Manager (`lib/power/ActivityMonitor`)
bewertet deshalb Drehzahl, Tempo, Last, Drosselklappe, Spannung und Busaktivität.
Ohne CAN-/OBD-Aktivität gilt das Fahrzeug nach 5 min als geparkt; 10 min danach schickt er dem Display
`Sleep`, beim Aufwachen `Wakeup`.

## Inbetriebnahme-Check

- [ ] Beide Geräte provisioniert (Weboberfläche zeigt „Zugangsdaten: compiled/stored“)
- [ ] Display zeigt „Verbunden“, Seite 4: kaum verlorene Pakete
- [ ] Bei laufendem Motor: Drehzahl/Tempo aktualisieren flüssig, langsame Werte innerhalb weniger Sekunden
- [ ] Signiertes Update per Web auf beiden Geräten, unsignierte Datei wird abgelehnt
- [ ] Nach Zündung-aus: Display geht nach der eingestellten Zeit schlafen
