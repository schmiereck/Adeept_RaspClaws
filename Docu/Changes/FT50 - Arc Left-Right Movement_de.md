# FT50: Implementierung der Bogenfahrt (Arc Left/Right)

**Datum:** 2026-01-21

**Autor:** Gemini

## 1. Problembeschreibung

Die bisherige Implementierung für Links- und Rechtsdrehungen (`CMD_LEFT`, `CMD_RIGHT`) ermöglichte dem Roboter nur eine Drehung auf der Stelle. Es fehlte eine Funktion, um eine Kurve zu fahren, also eine kombinierte Vorwärts- und Drehbewegung auszuführen.

Die ersten Versuche, dies über einen `turn_bias` zu implementieren, waren nicht erfolgreich, da die grundlegende Bewegungslogik nicht darauf ausgelegt war und die Bewegungen sich gegenseitig aufhoben.

## 2. Lösungsbeschreibung

Auf Anregung des Benutzers wurde die gesamte Bewegungslogik in `Server/Move.py` grundlegend refaktorisiert, um unterschiedliche Schrittlängen (Amplituden) für die linke und rechte Seite des Roboters zu ermöglichen. Dies ermöglicht eine präzise Steuerung, die einer Panzer- oder Kettensteuerung ähnelt.

### Kernänderungen:

1.  **Getrennte Geschwindigkeitssteuerung:**
    *   Das Herzstück der Bewegung ist nicht mehr ein einzelner `command`-String, sondern zwei separate Geschwindigkeitswerte: `speed_left` und `speed_right`. Diese Variablen steuern die Amplitude (Schrittlänge) für die jeweilige Seite.

2.  **Zentralisierung in `move_thread`:**
    *   Die `move_thread`-Funktion agiert nun als zentrales Gehirn. Sie berechnet basierend auf den vom Client empfangenen Befehlen (`CMD_FORWARD`, `CMD_LEFT`, `CMD_FORWARD_LEFT_ARC`, etc.) die entsprechenden Werte für `speed_left` und `speed_right`.
    *   **Bogenfahrt Links:** Die Schrittlänge der linken Beine wird reduziert (`speed_left = -movement_speed * (1 - arc_factor)`), während die rechten Beine die volle Schrittlänge beibehalten.
    *   **Drehung auf der Stelle:** Die Geschwindigkeiten sind gegenläufig (`speed_left = +speed`, `speed_right = -speed`), wodurch sich der Roboter auf der Stelle dreht.

3.  **Vereinfachung von `calculate_target_positions`:**
    *   Diese Funktion wurde radikal vereinfacht. Sie enthält keine `if/elif`-Logik für verschiedene Befehle mehr.
    *   Sie nimmt `speed_left` und `speed_right` direkt als Parameter und berechnet die horizontalen Beinpositionen basierend auf der jeweiligen seitenspezifischen Amplitude.
    *   Die Synchronisation des Gangs (welche Beine wann in der Luft sind) bleibt über die `phase` erhalten, die für alle Beine identisch ist.

4.  **Entfernung von redundantem Code:**
    *   Alle alten, spezifischen Hilfsfunktionen (`_get_turn_leg_positions`, `_get_arc_leg_positions`, etc.) wurden entfernt.

## 3. Implementierte Änderungen

-   **`Server/Move.py`:**
    *   Einführung der globalen Variable `arc_factor`.
    *   Komplettes Refactoring von `move_thread`, `execute_movement_step` und `calculate_target_positions`.
    *   Entfernung der nun überflüssigen `_get...`-Hilfsfunktionen.
-   **`Client/GUI.py`:**
    *   Hinzufügen der "Arc Left" (`Q`) und "Arc Right" (`E`) Buttons zur Benutzeroberfläche.
-   **`protocol.py`:**
    *   Definition der neuen Befehle `CMD_FORWARD_LEFT_ARC` und `CMD_FORWARD_RIGHT_ARC`.

## 4. Ergebnis

Der Roboter kann nun echte Kurvenfahrten (Bogenfahrten) nach links und rechts ausführen. Die Bewegungslogik ist insgesamt sauberer, flexibler und leichter für zukünftige Optimierungen anpassbar. Die grundlegende Funktionalität des Drehens auf der Stelle bleibt erhalten.
