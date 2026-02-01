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

Das Problem liegt höchstwahrscheinlich an der Netzwerk-Isolation zwischen dem Windows-Host und der WSL2-Umgebung. Die Windows-Firewall blockiert wahrscheinlich die UDP-Pakete, die für die ROS2-Discovery erforderlich sind, und wir können die erforderlichen Administratorrechte nicht erhalten, um sie zu deaktivieren oder Regeln hinzuzufügen. Die WSL-IP-Adresse `172.21.213.17` ist eine interne Docker-Netzwerk-IP, die nicht direkt vom Pi aus erreichbar ist.

**Empfohlene nächste Schritte:**

1.  **Windows-Firewall-Regel manuell hinzufügen:** Der Benutzer sollte manuell eine eingehende Firewall-Regel für das "Private" Netzwerkprofil erstellen, um UDP-Verkehr auf den Ports `7400-7420` zu erlauben. Dies erfordert Administratorrechte.
2.  **WSL-Netzwerkmodus auf "Bridged" umstellen:** Der Benutzer kann versuchen, den Netzwerkmodus von WSL2 von "NAT" (Standard) auf "Bridged" umzustellen. Dadurch erhält die WSL-Instanz eine eigene IP-Adresse im selben Netzwerk wie der Host-Computer, was die Kommunikation erheblich vereinfacht. Dies ist eine fortgeschrittene Konfiguration und erfordert Änderungen in der `.wslconfig`-Datei.
3.  **Verwendung eines ROS2-Discovery-Servers auf dem Host:** Eine weitere Möglichkeit ist, einen Discovery-Server auf dem Windows-Host (nicht in WSL) zu starten und sowohl den Pi als auch die WSL-Instanz damit zu verbinden.