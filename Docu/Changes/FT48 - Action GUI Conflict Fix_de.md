# FT48 - Action/GUI Servo-Konflikt behoben

**Feature**: Verhindert "Zappeln" des Roboters wenn Actions und GUI parallel laufen  
**Datum**: 2026-02-22  
**Status**: ✅ Implementiert

## Problem

Wenn der GUIServer läuft und gleichzeitig ROS2 Actions Bewegungskommandos ausführen, 
kommt es zu Servo-Konflikten, die dazu führen, dass der Roboter "zappelt".

**Ursache**: 
- Der Move-Thread (`RobotM`) läuft kontinuierlich mit 100 Hz (alle 10ms)
- GUI-Befehle können jederzeit Servo-Positionen ändern
- Actions verwenden die gleichen `commandInput()` Funktionen
- Beide konkurrieren um die Kontrolle der Servos
- Keine Synchronisation zwischen GUI und Actions

## Lösung

### 1. Action Lock Mechanismus in Move.py

Neue globale Variablen und Funktionen für Action-Kontrolle:

```python
# Action control lock
action_in_progress = False
action_lock = threading.Lock()

def set_action_in_progress(in_progress: bool):
    """Set action in progress flag to prevent GUI interference"""
    global action_in_progress
    with action_lock:
        action_in_progress = in_progress
        if in_progress:
            print("[Move] ⚠️  ROS2 Action active - GUI movement commands suspended")
        else:
            print("[Move] ✓ ROS2 Action completed - GUI movement commands restored")

def is_action_in_progress() -> bool:
    """Check if an action is currently executing"""
    with action_lock:
        return action_in_progress
```

### 2. GUI-Kommandos ignorieren während Actions

In `handle_movement_command()`:

```python
def handle_movement_command(command):
    """Handle movement commands (forward, backward, stand, left, right, no)"""
    
    # Ignore GUI movement commands if a ROS2 Action is currently executing
    if is_action_in_progress():
        # Silently ignore (no spam in logs)
        return True  # Pretend we handled it
    
    # ... rest of the function
```

### 3. Actions setzen/freigeben den Lock

Alle 4 Actions wurden mit try/finally Block erweitert:

**LinearMove, Rotate, ArcMove, HeadPosition**:

```python
async def execute_XXX_action(self, goal_handle):
    # ... setup code ...
    
    # === SET ACTION LOCK ===
    self.move.set_action_in_progress(True)
    
    try:
        # ... action execution ...
        return result
    finally:
        # === ALWAYS RELEASE LOCK ===
        self.move.set_action_in_progress(False)
```

Der `finally` Block garantiert, dass der Lock **immer** freigegeben wird, 
auch bei Fehlern oder Cancel-Requests.

## Geänderte Dateien

1. **Server/Move.py**
   - Neue Funktionen: `set_action_in_progress()`, `is_action_in_progress()`
   - Globale Variablen: `action_in_progress`, `action_lock`
   - `handle_movement_command()`: Check für Action-Sperre

2. **Server/action_servers.py**
   - `execute_linear_move()`: try/finally mit Lock
   - `execute_rotate()`: try/finally mit Lock
   - `execute_arc_move()`: try/finally mit Lock
   - `execute_head_position()`: try/finally mit Lock

## Verhalten

### Ohne Action (Normal)
```
GUI sendet "forward" → Move-Thread reagiert → Robot bewegt sich ✓
```

### Mit aktiver Action
```
Action startet → Lock gesetzt
GUI sendet "forward" → IGNORIERT (silently) 
Action läuft → Roboter bewegt sich sauber
Action endet → Lock freigegeben
GUI sendet "forward" → Move-Thread reagiert ✓
```

### Bei Action-Cancel oder Fehler
```
Action startet → Lock gesetzt
[Fehler oder Cancel]
finally-Block → Lock freigegeben (garantiert!)
GUI-Kontrolle wiederhergestellt ✓
```

## Log-Ausgaben

**Action Start**:
```
[Move] ⚠️  ROS2 Action active - GUI movement commands suspended
```

**Action Ende**:
```
[Move] ✓ ROS2 Action completed - GUI movement commands restored
```

**GUI-Befehle während Action**: Werden stillschweigend ignoriert (keine Log-Spam)

## Test

1. GUIServer starten (auf raspclaws-1)
2. ROSServer starten (ebenfalls auf raspclaws-1)
3. Von ubuntu1: Action ausführen
4. Gleichzeitig GUI-Befehle senden

**Erwartetes Ergebnis**: 
- Robot führt Action sauber aus
- Kein "Zappeln" mehr
- GUI-Befehle werden während Action ignoriert
- Nach Action-Ende: GUI-Kontrolle wieder voll funktionsfähig

## Hinweise

- **Thread-sicher**: Verwendet `threading.Lock()` für Synchronisation
- **Robust**: `finally` Block garantiert Lock-Freigabe
- **Silent**: Keine Log-Spam bei ignorierten GUI-Befehlen
- **Transparent**: Klare Log-Meldungen bei Action-Start/-Ende

## Weitere Verbesserungen (Optional)

Mögliche zukünftige Erweiterungen:

1. **Queue-System**: GUI-Befehle in Queue speichern statt ignorieren
2. **Prioritäten**: Verschiedene Lock-Level für verschiedene Action-Typen
3. **Timeout**: Auto-Unlock nach X Sekunden falls Action hängt
4. **Status-API**: Abfrage-Funktion für GUI ob Action läuft

Stand jetzt ist die Lösung simpel und effektiv! ✅
