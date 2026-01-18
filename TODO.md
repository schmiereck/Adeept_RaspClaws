# ToDo

## ROS 2

* Cosmos Reason 2
* ROS Planner
* ROS 2 (Rasperry Pi 3B+)
  * Docker-Container
    * ros-humble-ros-base (Bare Bones ohne Desktop)

### Der "Teleop"-Test (Ohne KI)
Auf dem PC: Du startest ein "Teleop"-Node (z. B. teleop_twist_keyboard). Wenn du eine Taste drückst, sendet der PC eine Nachricht auf dem Kanal (Topic) /cmd_vel (Command Velocity).

Auf dem Pi: Du schreibst ein kleines Python-Skript (einen ROS-Node), das dieses Topic aboniert.

Hardware-Aktion: Sobald die Nachricht auf dem Pi ankommt, übersetzt dein Skript die Geschwindigkeit in ein PWM-Signal für die GPIO-Pins des Pi 3+.

### Hindernisse beim Pi 3+ (und Lösungen)
RAM-Limit: Wenn du Code direkt auf dem Pi kompilieren willst (colcon build), kann der Speicher knapp werden.

Lösung: Nutze eine Swap-Datei (virtueller Arbeitsspeicher auf der SD-Karte) oder kompiliere nur ein Paket nach dem anderen mit --packages-select.

Stromversorgung: Der Pi 3+ zieht bei WLAN-Last und Motorsteuerung Spitzenströme. Achte auf ein stabiles 5V/3A Netzteil oder einen guten Step-Down-Wandler am Akku.

### Zusammenfassung der Schritte für dich:
Image flashen: Ubuntu Server 22.04 (64-bit) auf die SD-Karte.

ROS 2 Humble Base installieren (Anleitungen finden sich offiziell bei ROS.org).

Netzwerk: Sicherstellen, dass PC und Pi im selben WLAN sind und die Variable export ROS_DOMAIN_ID=0 auf beiden gesetzt ist.

Test: ros2 topic list auf dem PC eingeben – wenn du die Topics vom Pi siehst, steht die Brücke!
