 ich den Knopf gedrückt halte? Nur als Idee.# Änderungs-Historie - Adeept RaspClaws

Dieses Verzeichnis enthält alle dokumentierten Änderungen (Feature-Tickets, Bugfixes, Refactorings) am RaspClaws-Projekt in chronologischer Reihenfolge.

## Namenskonvention

- **FTxx** - Feature Ticket Nummer (chronologisch)
- **_de** - Deutsche Dokumentation
- **_en** - Englische Dokumentation (falls vorhanden)

## Chronologische Übersicht

### Initiale Fixes (FT1-4)
- **FT1** - Fix Connection Problem - Behebung von TCP-Verbindungsproblemen
- **FT2** - Fix Video Stream - Video-Streaming über ZMQ korrigiert
- **FT3** - Fix Second Connect - Problem beim zweiten Verbindungsaufbau gelöst
- **FT4** - Fix Second Connect Critical Bug - Kritischer Bugfix für Reconnect

### Infrastruktur & Setup (FT5-6)
- **FT5** - SSH Keys Setup - SSH-Schlüssel Konfiguration für sicheren Zugriff
- **FT6** - PCA9685 Fix - Hardware-Adressfehler korrigiert (0x5F → 0x40)

### Code-Qualität & Bugfixes (FT7-11)
- **FT7** - Critical Bugfix SetAttr Globals - Schwerwiegender Fehler bei globalen Variablen
- **FT8** - Bugfix Movement Indentation - Einrückungsfehler in Bewegungscode korrigiert
- **FT9** - Refactoring General - Allgemeine Code-Verbesserungen
- **FT10** - Refactoring Move - Bewegungscode refactored und verbessert
- **FT11** - Bugfix Continuous Movement - Kontinuierliche Bewegung funktioniert wieder

### Verbindungs-Stabilität (FT12-15)
- **FT12** - FPV Thread Fix - Video-Thread läuft stabil
- **FT13** - Reconnection Fix - Wiederverbindung ohne Server-Neustart
- **FT14** - ZMQ PubSub Fix - PUB/SUB-Pattern für Video-Stream implementiert
- **FT15** - Connection Error Handling - Robuste Fehlerbehandlung

### Servo-Testing & LED (FT16-19)
- **FT16** - Servo Tester - GUI zum Testen einzelner Servos mit SSH/X11
- **FT17** - Servo Tester Update - Verbesserungen am Servo-Tester
- **FT18** - Servo Tester Correction - Korrekturen der Kanal-Mappings
- **FT19** - LED Problem - LED-Steuerung funktioniert wieder

### Smooth Movement System (FT20-30) 🎯 Major Feature
- **FT20** - SmoothMode Analyse - Analyse des Smooth-Modus
- **FT21** - Enhancement Smooth Servos - Smooth-Servo-Bewegungen implementiert
- **FT22** - Refactoring Leg Functions - Bein-Funktionen vereinfacht
- **FT23** - Bugfix Jerk Direction - Richtungswechsel-Zuckbewegungen behoben
- **FT24** - Bugfix Jerk Smooth Mode - Zuckbewegungen im Smooth-Modus korrigiert
- **FT25** - Bugfix Incomplete Movements - Unvollständige Bewegungen behoben
- **FT26** - Refactoring Servo Position Tracking - Position-Tracking implementiert
- **FT27** - Bugfix Slow Motion - Zeitlupen-Problem durch Sleep-Removal gelöst
- **FT28** - Enhancement Servo Logging Timestamps - Millisekunden-Timestamps hinzugefügt
- **FT29** - Refactoring Sine Based Smooth Movement - **Komplett neue Bewegungs-Engine mit Sinus-Kurven!**
- **FT30** - Bugfix Continuous Movement Cycle - Kontinuierliche Bewegung ohne Sprünge

### Features & Optimierung (FT31-34)
- **FT31** - SmoothCam Implementation - Kamera-Bewegungen smooth
- **FT32** - CPU Optimization - CPU-Last-Optimierungen
- **FT33** - Battery Monitor - Akku-Überwachung in GUI
- **FT34** - Final Changes - Finale Anpassungen und Dokumentation

## Wichtigste Meilensteine

### 🎉 Smooth Movement System (FT20-30)
Die größte Feature-Entwicklung des Projekts! Von ersten Analysen (FT20) über viele Iterationen und Bugfixes bis hin zur **komplett neuen Sinus-basierten Bewegungs-Engine (FT29)** die mathematisch garantiert smooth läuft.

**Ergebnis:** Keine Zuckbewegungen mehr, kontinuierliche Bewegung, mathematisch elegante Lösung!

### 🔗 Verbindungs-Stabilität (FT1-4, FT12-15)
Robuste TCP- und ZMQ-Verbindungen, Wiederverbindung ohne Server-Neustart, fehlertolerante Kommunikation.

### 🛠️ Servo-Tester (FT16-18)
Professionelles GUI-Tool zum Testen und Kalibrieren einzelner Servos via SSH/X11.

### 🔋 Monitoring & Optimierung (FT32-33)
CPU-Optimierungen, Akku-Überwachung mit Farb-Coding, bessere System-Überwachung.

## Nutzung

Jede FT-Datei enthält:
- **Problem-Beschreibung** - Was war das Problem?
- **Lösung** - Wie wurde es gelöst?
- **Code-Änderungen** - Welche Dateien/Funktionen wurden geändert?
- **Testing** - Wie kann man es testen?
- **Lessons Learned** - Was haben wir gelernt?

## Statistik

- **Gesamt:** 34 Feature-Tickets
- **Bugfixes:** ~15
- **Refactorings:** ~8
- **Enhancements:** ~7
- **Infrastruktur:** ~4

## Autoren

Entwickelt von GitHub Copilot in Zusammenarbeit mit schmiereck  
Projekt-Zeitraum: 2026-01-18 bis 2026-01-19

---

**Hinweis:** Diese Dokumentation wird kontinuierlich erweitert. Neue Änderungen werden mit der nächsten FT-Nummer (FT35+) hinzugefügt.
enn