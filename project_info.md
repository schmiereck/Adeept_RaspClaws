# Projektinformationen Adeept_RaspClaws

## Remote-Zugriffe

### Raspberry Pi (Pi)
- **Verbindungstyp:** SSH (Schlüsselauthentifizierung eingerichtet)
- **Host:** pi@192.168.2.126
- **Projektverzeichnis:** `/home/pi/adeept_raspclaws/`
- **Aktueller Status:** ROSServer läuft in einem Docker Container und publiziert Topics.

### Remote-PC (PC)
- **Verbindungstyp:** SSH (Schlüsselauthentifizierung eingerichtet)
- **Host:** thoma@DESKTOP-4TPK9N8
- **SSH Key:** `C:\Users\SCMJ178\.ssh\id_ed25519pc`
- **Projektverzeichnis:** `C:\Users\thoma\Adeept_RaspClaws\`
- **Aktueller Status:** ROS2 empfängt Topics vom ROSServer auf dem Pi.
- **ROS Bridge auf PC:** Ein Script `C:\Users\thoma\Apps\ros_bridge.py` (gestartet via `C:\Users\thoma\Apps\start_robot_bridge.bat`) ist aktiv und dient als UDP-Relais zwischen der physischen WLAN-Karte (Pi-Seite) und dem virtuellen WSL-Adapter.
  *Hinweis: Der Python-Prozess für die Bridge muss laufen.*
- **ROS2 Umgebung im WSL:** Es existiert ein `ros_pi` Alias (`source ~/.bashrc && ros_pi`) der `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` und `FASTRTPS_DEFAULT_PROFILES_FILE=/home/smk/pc_bridge.xml` exportiert. Dieser Alias funktioniert nur in interaktiven Shells.
  - **ROS-Desktop:** `ros-humble-desktop` wurde erfolgreich im WSL installiert. Grafische Tools wie `rviz2` sind jetzt verfügbar. Für die Anzeige dieser grafischen Anwendungen auf Windows (unter Windows 10) muss ein X-Server (wie VcXsrv) laufen und die `DISPLAY` Umgebungsvariable im WSL gesetzt werden (z.B. `export DISPLAY=172.21.208.1:0.0`).
- **ROS2 Topic Discovery Lösung (PC-Seite, WSL für nicht-interaktive Shells):** Für nicht-interaktive WSL-Shells müssen die FastDDS-Umgebungsvariablen `RMW_IMPLEMENTATION` und `FASTRTPS_DEFAULT_PROFILES_FILE=/home/smk/pc_bridge.xml` direkt exportiert werden, da der `ros_pi` Alias nur in interaktiven Shells geladen wird. Beispiel: `wsl bash -c 'source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/smk/pc_bridge.xml && ros2 topic list'`

## Lokales Entwicklungsumfeld
- **Aktuelles Arbeitsverzeichnis:** `C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws`
- **Typ:** Git-Projekt (wird auf Pi und PC gepushed)

## Erfolgreiche Lösung des ROS2-Topic-Empfangsproblems in WSL

**Problemzusammenfassung:**
Das Problem lag an einer Inkompatibilität in den FastDDS-Konfigurationen des Raspberry Pi und des WSL-PCs, sowie an der falschen internen WSL-IP-Adresse in der `pc_bridge.xml`.

**Lösung:**
1.  **Pi `docker-compose.ros2.yml`:** Die Konfiguration wurde vereinfacht, um `ROS_STATIC_PEERS=192.168.2.121` direkt zu setzen, um den Windows-Host als Peer zu adressieren.
2.  **WSL `pc_bridge.xml`:** Die Datei wurde so korrigiert, dass `fastdds.external_address` auf die Windows Host IP (`192.168.2.121`) und die `initialPeersList` auf die korrekte interne WSL-IP (`172.21.208.1`) zeigt, wie in der `ROS2-WSL-setup.md` dokumentiert.
3.  **Windows Firewall:** Die Windows-Firewall für private Profile wurde deaktiviert und eine spezifische eingehende UDP-Regel für die Ports `7400-7415` wurde manuell hinzugefügt.
4.  **Python Bridge:** Die `ros_bridge.py` auf dem Windows-Host war aktiv und wurde als essenziell für die Weiterleitung der UDP-Pakete zwischen Pi und WSL bestätigt.

Durch diese Anpassungen funktioniert die ROS2-Discovery und der Empfang der Topics auf dem WSL-PC wieder.