# Projektinformationen Adeept_RaspClaws

## Remote-Zugriffe

### Raspberry Pi (Pi)
- **Verbindungstyp:** SSH (Schlüsselauthentifizierung eingerichtet)
- **Host:** pi@192.168.2.126
- **Projektverzeichnis:** `/home/pi/adeept_raspclaws/`
- **Aktueller Status:** ROSServer läuft in einem Docker Container.

### Remote-PC (PC)
- **Verbindungstyp:** SSH (Schlüsselauthentifizierung eingerichtet)
- **Host:** thoma@DESKTOP-4TPK9N8
- **SSH Key:** `C:\Users\SCMJ178\.ssh\id_ed25519pc`
- **Projektverzeichnis:** `C:\Users\thoma\Adeept_RaspClaws\`
- **Aktueller Status:** ROS2 empfängt Topics vom ROSServer auf dem Pi.
- **ROS Bridge auf PC:** Ein Script `C:\Users\thoma\Apps\ros_bridge.py` (gestartet via `C:\Users\thoma\Apps\start_robot_bridge.bat`) ist aktiv und dient als Bridge in das WSL.
  *Hinweis: Der Python-Prozess für die Bridge konnte nicht auf dem Remote-PC gefunden werden. Bitte stelle sicher, dass sie läuft, ggf. manuell neu starten.*
- **ROS2 Umgebung im WSL:** Es existiert ein `ros_pi` Alias (`source ~/.bashrc && ros_pi`) der `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` und `FASTRTPS_DEFAULT_PROFILES_FILE=/home/smk/pc_bridge.xml` exportiert. Dieser Alias funktioniert nur in interaktiven Shells.
  - **ROS-Desktop:** `ros-humble-desktop` wurde erfolgreich im WSL installiert. Grafische Tools wie `rviz2` sind jetzt verfügbar. Für die Anzeige dieser grafischen Anwendungen auf Windows (unter Windows 10) muss ein X-Server (wie VcXsrv) laufen und die `DISPLAY` Umgebungsvariable im WSL gesetzt werden (z.B. `export DISPLAY=172.21.208.1:0.0`).
- **ROS2 Topic Discovery Lösung (PC-Seite, WSL für nicht-interaktive Shells):** Für nicht-interaktive WSL-Shells müssen die FastDDS-Umgebungsvariablen `RMW_IMPLEMENTATION` und `FASTRTPS_DEFAULT_PROFILES_FILE=/home/smk/pc_bridge.xml` direkt exportiert werden, da der `ros_pi` Alias nur in interaktiven Shells geladen wird. Beispiel: `wsl bash -c 'source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/smk/pc_bridge.xml && ros2 topic list'`

## Lokales Entwicklungsumfeld
- **Aktuelles Arbeitsverzeichnis:** `C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws`
- **Typ:** Git-Projekt (wird auf Pi und PC gepushed)

## Aktuelles Problem: ROS2-Topic-Empfang in WSL

**Zusammenfassung:**

Alle bekannten softwareseitigen Konfigurationen (Pi Docker Compose, `pi_static_peers.xml`, `pc_bridge.xml` auf WSL) wurden gemäß der bereitgestellten Dokumentation und den Erwartungen an die Python-Bridge angepasst. Der Pi publiziert die Topics, die Python-Bridge läuft, und die Windows-Firewall wurde vom Benutzer deaktiviert. Trotzdem empfängt der WSL-PC keine ROS2-Topics vom Pi.

Dies deutet auf ein tieferliegendes Netzwerkproblem hin, das höchstwahrscheinlich mit der Interaktion zwischen Windows, WSL2 und der Python-Bridge zusammenhängt, insbesondere mit der Art und Weise, wie Windows virtuelle Netzwerke handhabt und wie die Firewall *auch bei deaktiviertem Profil* noch eingreifen kann oder wie das Netzwerkprofil von WSL erkannt wird.

**Verbleibende mögliche Ursachen (manuelle Überprüfung erforderlich):**

1.  **Windows-Netzwerkprofil für WSL:** Selbst wenn die Firewall für das "Private" Profil deaktiviert ist, muss sichergestellt sein, dass das virtuelle Netzwerk, das WSL verwendet, auch wirklich als "Privat" und nicht als "Öffentlich" erkannt wird. Überprüfe dies manuell in den Windows-Netzwerkeinstellungen.
2.  **Firewall-Regel für Ports:** Obwohl die Firewall deaktiviert wurde, ist es ratsam, die in der `ROS2-WSL-setup.md` erwähnte spezifische eingehende Firewall-Regel für UDP-Ports `7400-7415` manuell hinzuzufügen. Manchmal kann eine explizite Regel auch bei deaktivierter Firewall helfen oder sicherstellen, dass bestimmte Ausnahmen korrekt behandelt werden, sobald die Firewall wieder aktiviert wird.
3.  **`C:\Users\thoma\.wslconfig`:** Überprüfe, ob die `vmSwitch=WSL-Bridge` Einstellung in der `.wslconfig` noch vorhanden und korrekt ist.
4.  **IP-Adressen der Bridge:** Vergewissere dich, dass die `PI_IP` (`192.168.2.126`) und `WSL_IP` (`172.21.213.17`) in `ros_bridge.py` immer noch korrekt sind und mit den aktuellen IPs übereinstimmen. Die WSL-IP (`172.21.213.17`) ist dynamisch und kann sich ändern, wenn WSL neu gestartet wird.
