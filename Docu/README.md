K# Dokumentation - Adeept RaspClaws

## Verzeichnisstruktur

### `Changes/` - Änderungs-Historie
Alle Feature-Tickets (FT1-FT34+) in chronologischer Reihenfolge.  
→ Siehe [Changes/README.md](Changes/README.md) für detaillierte Übersicht

### Keyboard Shortcuts
- [KEYBOARD_SHORTCUTS.md](KEYBOARD_SHORTCUTS.md) - Deutsche Tastenkombinationen
- [KEYBOARD_SHORTCUTS_EN.md](KEYBOARD_SHORTCUTS_EN.md) - English Keyboard Shortcuts

---

## Quick Links

### 📚 Wichtigste Änderungen

**Smooth Movement System (FT20-30):**
Die größte Feature-Entwicklung! Von ersten Analysen bis zur komplett neuen Sinus-basierten Bewegungs-Engine.
- [FT29 - Refactoring Sine Based Smooth Movement](Changes/FT29%20-%20Refactoring%20Sine%20Based%20Smooth%20Movement_de.md) - **Hauptfeature!**
- [FT30 - Bugfix Continuous Movement Cycle](Changes/FT30%20-%20Bugfix%20Continuous%20Movement%20Cycle_de.md) - Finale Optimierung

**Verbindungs-Stabilität (FT1-4, FT12-15):**
- [FT1 - Fix Connection Problem](Changes/FT1%20-%20Fix%20Connection%20Problem_de.md)
- [FT13 - Reconnection Fix](Changes/FT13%20-%20Reconnection%20Fix_de.md)
- [FT14 - ZMQ PubSub Fix](Changes/FT14%20-%20ZMQ%20PubSub%20Fix_de.md)

**Servo-Tester (FT16-18):**
- [FT16 - Servo Tester](Changes/FT16%20-%20Servo%20Tester_de.md) - GUI für Servo-Testing
- [FT16 - Servo Tester SSH X11](Changes/FT16%20-%20Servo%20Tester%20SSH%20X11_de.md) - Remote-Verwendung

**Hardware-Fixes:**
- [FT6 - PCA9685 Fix](Changes/FT6%20-%20PCA9685%20Fix_de.md) - I2C-Adresse korrigiert

### 🎯 Alle Änderungen

Siehe [Changes/README.md](Changes/README.md) für vollständige Liste aller 34+ Feature-Tickets.

---

## Für neue Änderungen

Neue Dokumentationen werden als **FT35+** im `Changes/`-Verzeichnis erstellt:

```
Changes/FT35 - <Feature Name>_de.md
```

**Namenskonvention:**
- FTxx - Chronologische Nummer
- Feature Name - Kurzbeschreibung
- _de/_en - Sprache

---

## Projekt-Status

✅ **Stabile Version** - Alle Hauptfunktionen implementiert und getestet  
✅ **Smooth Movement** - Mathematisch elegante Bewegungs-Engine  
✅ **Robuste Verbindung** - Reconnection ohne Server-Neustart  
✅ **Monitoring** - CPU, RAM, Battery-Anzeige in GUI  
✅ **Testing-Tools** - Servo-Tester für Kalibrierung  

**Nächste Schritte:**
- [ ] ROS 2 Integration (geplant)
- [ ] Weitere Optimierungen
- [ ] Zusätzliche Features nach Bedarf

---

Entwickelt von GitHub Copilot in Zusammenarbeit mit schmiereck  
2026-01-18 bis 2026-01-19
