# LoRa APRS für LILYGO T-Deck Plus

Arduino-IDE-Sketch mit drei Displayseiten, Tastatur-/Trackballbedienung, LoRa-APRS-Nachrichten, Stationsentfernung und Richtung sowie GPS-/Stationsmeldungen.

## Funktionen

- **Stationen:** empfangene Rufzeichen, Entfernung in km und Peilung in Grad (Nord = 0°, Ost = 90°); Positionen werden aus unkomprimierten APRS-Koordinaten gelesen.
- **Nachrichten:** Empfangene und gesendete Nachrichten, freies Zielrufzeichen und Nachrichtentext, Nachrichten-ID und automatische ACK-Antworten.
- Bei einer neu empfangenen Nachricht an das eigene Rufzeichen ertönt ein 500-ms-Piep mit 1750 Hz über den I2S-Lautsprecher.
- Der Nachrichtenton lässt sich in den Geräteeinstellungen und auf der Webseite ein- und ausschalten; die Auswahl wird gespeichert.
- Nachrichten-IDs wie `{1` bleiben im Funkpaket für APRS-ACKs erhalten, werden aber in der lokalen Nachrichtenliste und der lesbaren Empfangsanzeige ausgeblendet.
- **Einstellungen:** Rufzeichen, APRS-Symbol, Kommentar und Status; Einstellungen bleiben nach dem Neustart gespeichert.
- **Akku:** Ladezustand und Spannung werden anhand des Akku-ADC an GPIO4 angezeigt. Die Anzeige ist eine aus der Zellspannung abgeleitete Schätzung. Auf Seite 3 lässt sich **Akku im Status** schalten; bei eingeschalteter Option wird die Akkuangabe zusammen mit dem benutzerdefinierten APRS-Status nach jeweils zehn Positionsbaken gesendet. Der Schalter ist auch auf der Webseite verfügbar.
- **Akkuspar-Takt:** Der CPU-Takt sinkt bei weniger als 25 % Akku auf 80 MHz. Erst oberhalb von 30 % wechselt er zurück auf 160 MHz.
- **SmartBeaconing:** Positionsbeacons werden abhängig von Geschwindigkeit und Kursänderung gesendet; SmartBeaconing ist standardmäßig aktiv und kann auf Seite 3 abgeschaltet werden. Im ausgeschalteten Zustand gilt das bisherige Festintervall von drei Minuten.
- **Wireless:** Die drei Wireless-Felder liegen am Ende von Seite 3. Dort zwischen Aus, Access Point und Client wechseln sowie SSID und Passwort einstellen. Die vergebene IP wird direkt unter dem Wireless-Modus angezeigt.
- **Webseite:** Empfangene Stationen mit Entfernung/Peilung sowie empfangene und gesendete Nachrichten; neue Nachrichten und Antworten lassen sich im Browser senden.
- **Stationsliste:** Im Arbeitsspeicher und auf dem Display bleiben bis zu 16 Stationen sichtbar. Die SD-Karte hält davon nur die zehn zuletzt gehörten Stationen samt Position für den nächsten Start vor. Ohne eigenen GPS-Fix zeigt Seite 1 deren gespeicherte Koordinaten; mit GPS-Fix zeigt sie Entfernung und Richtung. Auf Seite 1 die Station auswählen und `d` drücken, um sie aus Liste und SD-Speicher zu löschen; auf der Webseite gibt es ebenfalls eine Löschtaste.
- **Nachrichtenarchiv:** An das eigene Rufzeichen adressierte, empfangene APRS-Nachrichten werden auf der integrierten SD-Karte gespeichert. Nach einem Neustart lädt der Sketch die letzten zwölf Nachrichten wieder in die Nachrichtenliste. Ohne erkannte SD-Karte läuft der Sketch weiter, aber Nachrichten gehen beim Neustart verloren.
- **Nachrichten löschen:** Auf Seite 2 eine Nachricht auswählen und `d` auf der Tastatur drücken. In der Webseite kann jede Nachricht über **Löschen** entfernt werden. Das Löschen wird auch im SD-Archiv übernommen.
- **Beacon-Inhalt:** Positionsmeldung mit Kommentar/Symbol; der APRS-Status wird nach jeweils zehn erfolgreich gesendeten Positionsbeacons einmal separat gesendet.

## Bedienung

- Trackball **links/rechts** wechselt zwischen Stationen, Nachrichten, Optionen und Details.
- Trackball **hoch/runter** bewegt die Auswahl.
- Trackball **klicken** startet das Erstellen einer Nachricht oder bearbeitet ein Einstellungsfeld.
- Auf Seite 1 sendet ein **zwei Sekunden langer Trackball-Druck** sofort eine manuelle Positionsbake. Nach erfolgreicher Übertragung erscheint kurz „Bake gesendet“.
- **Seite 3 – Optionen:** Rufzeichen, eigenes APRS-Symbol, Kommentar und Status sowie Wireless- und Tonoptionen. SmartBeaconing wird hier mit Klick/Enter ein- oder ausgeschaltet.
- **Seite 4 – Details:** Die Abschnitte **SmartBeaconing:** und **APRS Symbol:** enthalten die Beacon-Parameter und Symbolkürzel. Zahlenwerte werden ausgewählt und per Tastatur ersetzt; Enter speichert, Backspace löscht. Klick/Enter auf einem Symbolkürzel übernimmt es.
- Der Eintrag **Nachrichtenton** am Ende der Einstellungsseite schaltet den Empfangston um. Auf der Webseite gibt es denselben Schalter oberhalb der Stationenliste.
- **Akku im Status** am Ende der Einstellungsseite schaltet den Akkuwert im periodischen APRS-Status ein oder aus.
- **Wireless** mit Klick/Enter durch Aus → AP → Client schalten. SSID und Passwort vorher in den benachbarten Feldern setzen. Ein AP benötigt ein Passwort mit mindestens acht Zeichen. Im Clientmodus zeigt die Zeile unter dem Modus die IP nach erfolgreicher Verbindung.
- Sobald WLAN aktiv ist, öffne die angezeigte IP im Browser. Im AP-Modus ist die Standardadresse `http://192.168.4.1/`; im Clientmodus verwendest du die im Menü angezeigte IP. Die Webseite aktualisiert Stationen und Nachrichten automatisch und hat ein Feld für neue Nachrichten sowie Antwortknöpfe bei empfangenen Nachrichten.
- Die Symbol-Kürzel **Telefon /$**, **Auto />**, **Van /v**, **LKW /k**, **Mensch /[**, **Trecker /F** und **Schiff /s** liegen auf Seite 4. Das Feld **Symbol** auf Seite 3 erlaubt weiterhin ein eigenes Zeichen aus der primären APRS-Tabelle.
- Tastatur: Zeichen eingeben, **Enter** zum Bestätigen/Weiterschalten, **Backspace** zum Löschen.
- Die Tastaturbeleuchtung schaltet sich bei einer Taste ein und nach fünf Sekunden ohne weitere Eingabe wieder aus.
- Die Displaybeleuchtung geht nach 60 Sekunden ohne Eingabe aus. Trackball- oder Tastatureingaben sowie empfangene LoRa-Pakete schalten sie wieder ein.
- Auf der Nachrichtenseite startet die Taste `m` ebenfalls eine neue Nachricht. Zielrufzeichen eingeben, Enter drücken, Text eingeben und mit Enter senden.
- Zum Antworten auf Seite 2 die Nachricht mit dem Trackball auswählen und den Trackball drücken (oder `r` auf der Tastatur). Das Rufzeichen wird übernommen; direkt den Antworttext eingeben und mit Enter senden. `Enter` oder `m` startet weiterhin eine Nachricht an ein neues Rufzeichen.
- USB-Seriellmonitor bleibt mit Statusmeldungen und Befehlen verfügbar: `?`, `p`, `b`, `m ZIEL-SSID Nachricht`.

## Arduino IDE

1. Installiere das Espressif-ESP32-Boardpaket.
2. Benötigte Bibliotheken: **RadioLib 8.x**, **TinyGPSPlus** und **TFT_eSPI**. `Wire` und `Preferences` sind im ESP32-Boardpaket enthalten.
3. Kopiere `LoRaAPRS_TDeckPlus.ino`, `tft_setup.h` und `web_page.h` gemeinsam in denselben Sketch-Ordner und öffne die `.ino`-Datei. Der Setup-Header enthält die T-Deck-Displayparameter selbst und korrigiert die SPI-Registerportnummer für ESP32-S3/Core 3.x; eine Änderung der installierten Bibliothek ist nicht nötig.
4. Prüfe `LORA_MHZ`, `LORA_BW_KHZ`, `LORA_SF`, `LORA_CR`, `LORA_SYNC_WORD` und `LORA_TX_DBM` für deine Gegenstellen und die örtlichen Funkregeln. Der Standardwert 433.775 MHz ist keine allgemeine Frequenzfreigabe.
5. Wähle **ESP32S3 Dev Module**. Der Sketch setzt die CPU beim Start auf 160 MHz. Weitere empfohlene Optionen: USB CDC On Boot Enabled, Flash 16 MB, PSRAM OPI, Partition Scheme 16M Flash (3MB APP/9.9MB FATFS), USB Mode Hardware CDC and JTAG.
6. Lade den Sketch hoch. Bei Upload-Problemen Trackball-Mitte gedrückt halten und USB verbinden, dann hochladen und Reset drücken.

Rufzeichen, Symbol, Kommentar und Status änderst du über die Einstellungsseite am Gerät. Der Startwert des Rufzeichens ist `DK8HC-11`.

SmartBeaconing-Standardwerte: Slow rate 10 min, Fast rate 60 s, Low/High speed 5/60 km/h, Turn angle 25°, Turn slope 255 (Grad·km/h) und Turn time 30 s. Zwischen zwei automatischen SmartBeacons liegen immer mindestens 30 Sekunden, auch wenn Fast Rate niedriger eingestellt ist. Die Werte werden nichtflüchtig gespeichert. Geschwindigkeitsintervalle folgen der klassischen Kurve `Fast rate × High speed ÷ aktuelle Geschwindigkeit`, begrenzt durch Slow/Fast rate und die Mindestpause. Die Kursänderungsschwelle ist `Turn angle + Turn slope ÷ Geschwindigkeit` und wird auf 120° begrenzt. Beaconing setzt einen aktuellen GPS-Fix voraus.

Die Symbol-Kürzel entsprechen der primären APRS-Symboltabelle. Sie sendet jeweils den Tabellencode `/` zusammen mit dem Symbolzeichen; die Referenz ist [APRS Symbols New](https://www.aprs.org/symbols/symbols-new.txt).

## Protokollhinweis

Der Sketch sendet TNC2-APRS-Text mit dem RadioLib-Header für APRS-over-LoRa (`3C FF 01`). Er ist nicht mit klassischem AX.25/AFSK auf 144.800 MHz kompatibel. LoRa-Frequenz und Modemparameter müssen zu den verwendeten APRS-over-LoRa-Gegenstellen passen.

Die Hardware-Pins folgen der [LILYGO T-Deck Plus-Dokumentation](https://wiki.lilygo.cc/products/t-deck-series/t-deck-plus/): Display ST7789 über TFT_eSPI (Setup 210 in `User_Setup_Select.h`); Tastatur I²C-Adresse `0x55` an GPIO18/8 mit Interrupt GPIO46; Trackball GPIO3/2/15/1 plus Klick GPIO0; GPS UART1 mit ESP32-RX GPIO44 und TX GPIO43; I²S-Lautsprecher GPIO7 (BCK), GPIO5 (WS) und GPIO6 (DOUT).

## Allgemeiner Hinweise

Der Code wurde mit der Codex Anwendung von ChatGPT nach meinen Vorgaben erstellt, er kann Fehler enthalten, Nutzung auf eigene Gefahr
