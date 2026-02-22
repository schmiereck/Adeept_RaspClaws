# FT48 - Action/GUI Servo-Konflikt behoben

**Feature**: Verhindert "Zappeln" des Roboters wenn Actions und GUI parallel laufen  
**Datum**: 2026-02-22  
**Status**: ✅ Implementiert (Rev. 2 - Critical Bugfix)

## Problem

Wenn der GUIServer läuft und gleichzeitig ROS2 Actions Bewegungskommandos ausführen, 
kommt es zu Servo-Konflikten, die dazu führen, dass der Roboter "zappelt".

**Ursache**: 
- Der Move-Thread (`RobotM`) läuft kontinuierlich mit 100 Hz (alle 10ms)
- GUI-Befehle können jederzeit Servo-Positionen ändern
- Actions verwenden die gleichen `commandInput()` Funktionen
- Beide konkurrieren um die Kontrolle der Servos
- Keine Synchronisation zwischen GUI und Actions

## Lösung (Rev. 2 - Fixed!)

**WICHTIG**: Revision 2 behebt einen kritischen Fehler in Rev. 1!

### Problem in Rev. 1
Der Check wurde in `Move.handle_movement_command()` durchgeführt.
Dies blockierte **alle** Bewegungsbefehle, auch die von Actions selbst!
→ Actions konnten sich nicht mehr bewegen! ❌

### Lösung in Rev. 2
Der Check wurde in den **GUIServer** verschoben, wo GUI-Befehle ankommen.
Actions rufen `move.commandInput()` direkt auf, umgehen also den GUI-Check.
→ Actions funktionieren, GUI wird blockiert! ✅

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

### 2. GUI-Kommandos ignorieren während Actions (im GUIServer!)

In `GUIServer.handle_movement_command()` (NICHT in Move.py!):

```python
def handle_movement_command(data):
    """Handle movement commands from GUI"""
    
    # === CHECK IF ACTION IS RUNNING ===
    # Ignore GUI movement commands if a ROS2 Action is currently executing
    if move.is_action_in_progress():
        # Silently ignore to avoid log spam
        return True  # Pretend we handled it
    
    # ... rest of function processes GUI command
```

**Wichtig**: Der Check ist im **GUIServer**, nicht in Move.py!
Actions rufen `move.commandInput()` direkt auf und umgehen diesen Check.

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
        # Actions call self.move.commandInput() directly
        # This bypasses the GUI check in GUIServer
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
   - ~~`handle_movement_command()`: Check für Action-Sperre~~ (Rev. 1 - ENTFERNT in Rev. 2)

2. **Server/GUIServer.py** (NEU in Rev. 2)
   - `handle_movement_command()`: Check für Action-Sperre **VOR** Weiterleitung an Move

3. **Server/action_servers.py**
   - `execute_linear_move()`: try/finally mit Lock
   - `execute_rotate()`: try/finally mit Lock
   - `execute_arc_move()`: try/finally mit Lock
   - `execute_head_position()`: try/finally mit Lock

## Verhalten

### Ohne Action (Normal)
```
GUI sendet "forward" → GUIServer → Move-Thread reagiert → Robot bewegt sich ✓
```

### Mit aktiver Action
```
Action startet → Lock gesetzt
GUI sendet "forward" → GUIServer prüft Lock → IGNORIERT (silently)
Action ruft move.commandInput() direkt → Move-Thread reagiert → Robot bewegt sich ✓
Action endet → Lock freigegeben
GUI sendet "forward" → GUIServer → Move-Thread reagiert ✓
```

### Bei Action-Cancel oder Fehler
```
Action startet → Lock gesetzt
[Fehler oder Cancel]
finally-Block → Lock freigegeben (garantiert!)
GUI-Kontrolle wiederhergestellt ✓
```

## Wichtige Änderung in Rev. 2

**Fehler in Rev. 1**: Check in `Move.handle_movement_command()`
- Blockierte **alle** Befehle (GUI + Actions)
- Actions konnten sich nicht bewegen ❌

**Fix in Rev. 2**: Check in `GUIServer.handle_movement_command()`
- Blockiert nur **GUI-Befehle**
- Actions funktionieren normal ✅

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
4. Gleichzeitig GUI-Befehle senden (oder GUI parallel laufen lassen)

**Erwartetes Ergebnis**: 
- Robot führt Action sauber aus ✓
- Kein "Zappeln" mehr ✓
- GUI-Befehle werden während Action ignoriert ✓
- Actions können sich bewegen! ✓
- Nach Action-Ende: GUI-Kontrolle wieder voll funktionsfähig ✓

## Hinweise

- **Thread-sicher**: Verwendet `threading.Lock()` für Synchronisation
- **Robust**: `finally` Block garantiert Lock-Freigabe
- **Silent**: Keine Log-Spam bei ignorierten GUI-Befehlen
- **Transparent**: Klare Log-Meldungen bei Action-Start/-Ende
- **Korrekt**: Check im GUIServer, nicht in Move.py!

## Revision History

- **Rev. 1** (2026-02-22 09:03): Initial implementation - Check in Move.py (FEHLERHAFT)
- **Rev. 2** (2026-02-22 09:25): Bugfix - Check verschoben nach GUIServer.py (KORREKT)

Stand Rev. 2 ist die Lösung simpel, effektiv und funktioniert! ✅
