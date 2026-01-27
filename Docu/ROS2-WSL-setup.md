# ROS2 WSL Setup
* https://gemini.google.com/app/63d43c0b1129a897?hl=de
* https://gemini.google.com/share/445c4d1ebfa1

Das Setup ist durch die WSL2-Netzwerkisolierung recht komplex geworden, daher ist eine Übersicht der "Puzzleteile" goldwert.

Hier ist die vollständige Liste der Komponenten, unterteilt in **Dateien** und **System-Einstellungen**, damit deine Doku lückenlos ist:

---

### 1. Dateien auf dem Windows-Host

* **`C:\Users\thoma\.wslconfig`**
* *Zweck:* Grundkonfiguration von WSL2. Hier haben wir sichergestellt, dass WSL2 stabil läuft (z. B. Speicherzuweisung).


* **`C:\Users\thoma\Apps\ros_bridge.py`**
* *Zweck:* Das Herzstück der Verbindung. Es fungiert als UDP-Relais zwischen der physischen WLAN-Karte (Pi-Seite) und dem virtuellen WSL-Adapter.


* **`C:\Users\thoma\Apps\start_robot_bridge.bat`**
* *Zweck:* Komfort-Starter für das Python-Skript.



### 2. Dateien im WSL2 (Linux)

* **`/home/smk/pc_bridge.xml`**
* *Zweck:* Die FastDDS-Konfiguration. Sie zwingt ROS 2 zur Unicast-Kommunikation und "lügt" dem Pi gegenüber über die eigene IP (External Address), damit dieser die Pakete an die Windows-Haustür schickt.


* **`~/.bashrc`**
* *Zweck:* Enthält den Alias `ros_pi` und die Export-Befehle für `RMW_IMPLEMENTATION` und `FASTRTPS_DEFAULT_PROFILES_FILE`.



---

### 3. Was wir sonst noch angepasst haben (Wichtig für die Doku!)

Zusätzlich zu den Dateien gab es drei kritische **System-Eingriffe auf Windows**, ohne die der Tunnel blockiert würde:

#### A. Windows-Firewall (Port-Freigaben)

Wir haben explizite Regeln erstellt, um den ROS 2 Discovery- und Datenverkehr durchzulassen.

* *Befehl:* `New-NetFirewallRule` für UDP-Ports 7400 bis 7415.
* *Grund:* Windows blockiert standardmäßig unaufgeforderten UDP-Verkehr von "fremden" Geräten wie dem Pi.

#### B. Netzwerk-Profil (Privat statt Öffentlich)

Wir haben alle Netzwerkadapter auf **"Privat"** umgestellt.

* *Zweck:* Im Profil "Öffentlich" (Public) greifen extrem restriktive Filter, die selbst bei offenen Firewall-Ports die Kommunikation zwischen WSL und WLAN unterbinden können.
* *Befehl:* `Set-NetConnectionProfile -NetworkCategory Private`.

#### C. Registry / Lokale Sicherheitsrichtlinie (Unidentifizierte Netzwerke)

Wir haben Windows angewiesen, auch Netzwerke ohne Internetzugriff (wie die virtuelle WSL-Brücke) als "Privat" zu behandeln.

* *Zweck:* Verhindert, dass der WSL-Adapter bei jedem Neustart wieder in den blockierenden "Öffentlich"-Modus zurückfällt.

---

### 4. Konfiguration auf dem Raspberry Pi

* Hier war **keine** dauerhafte Dateiänderung nötig, da wir das Standard-Docker-Image von Adeept nutzen.
* *Wichtig für die Doku:* Der Container muss **neu gestartet** werden, sobald die PC-Konfiguration geändert wird, damit der Pi die neue "Visitenkarte" des PCs (aus der XML) einliest.

---

**Zusammenfassender Ablauf für deine Doku:**

1. Python-Bridge auf Windows starten (öffnet den Tunnel).
2. In WSL2 `ros_pi` aufrufen (setzt die ROS-Pfade).
3. Pi-Container ggf. neustarten (erzwingt neues Discovery).
4. Kommunikation läuft!

---

# Files

## WSL: C:\Users\thoma\.wslconfig
```
[wsl2]
vmSwitch=WSL-Bridge
```
---
## WSL: /home/smk/pc_bridge.xml:
```
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="pc_profile" is_default_profile="true">
        <rtps>
            <propertiesPolicy>
                <properties>
                    <property>
                        <name>fastdds.external_address</name>
                        <value>192.168.2.121</value>
                    </property>
                </properties>
            </propertiesPolicy>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>172.21.208.1</address> <port>7400</port>
                        </udpv4>
                    </locator>
                </initialPeersList>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>192.168.2.121</address>
                            <port>7410</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
            </builtin>
            <defaultUnicastLocatorList>
                <locator>
                    <udpv4>
                        <address>192.168.2.121</address>
                        <port>7411</port>
                    </udpv4>
                </locator>
            </defaultUnicastLocatorList>
        </rtps>
    </participant>
</profiles>
```
---
## WSL: C:\Users\thoma\Apps\ros_bridge.py:
```
import socket
import threading
import time

# KONFIGURATION
PI_IP = "192.168.2.126"
WSL_IP = "172.21.213.17"
PORTS = list(range(7400, 7416)) # Deckt Discovery und Daten ab

# Statistiken für den Herzschlag
stats = {"wsl_to_pi": 0, "pi_to_wsl": 0}

def heartbeat():
"""Gibt jede Minute einen kurzen Statusbericht aus."""
while True:
time.sleep(60)
print(f"[{time.strftime('%H:%M:%S')}] Tunnel-Status: "
f"WSL->Pi: {stats['wsl_to_pi']} Pkt, "
f"Pi->WSL: {stats['pi_to_wsl']} Pkt")
# Zähler zurücksetzen
stats["wsl_to_pi"] = 0
stats["pi_to_wsl"] = 0

def bridge(port):
"""Leitet UDP-Pakete zwischen Pi und WSL weiter."""
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
s.bind(('0.0.0.0', port))
except Exception as e:
print(f"Fehler an Port {port}: {e}")
return

    while True:
        data, addr = s.recvfrom(65535)
        if addr[0] == WSL_IP:
            s.sendto(data, (PI_IP, port))
            stats["wsl_to_pi"] += 1
        elif addr[0] == PI_IP:
            s.sendto(data, (WSL_IP, port))
            stats["pi_to_wsl"] += 1

if __name__ == "__main__":
print(f"--- ROS 2 BRÜCKE GESTARTET ---")
print(f"Pi: {PI_IP} <-> WSL: {WSL_IP}")
print(f"Überwache {len(PORTS)} Ports (7400-7415)...")
print("Der Status-Herzschlag erscheint alle 60 Sekunden.")

    # Herzschlag-Thread starten
    threading.Thread(target=heartbeat, daemon=True).start()
    
    # Port-Threads starten
    for p in PORTS:
        threading.Thread(target=bridge, args=(p,), daemon=True).start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nBrücke wird beendet...")
```
---
## WSL: ~/.bashrc
Permananet einrichten:

nano ~/.bashrc

vor "humble":
```
# ROS 2 Pi-Bridge Automatisierung
alias ros_pi='export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/smk/pc_bridge.xml && echo "ROS 2 Pi-Bridge aktiv!"'
```
=>
Ab jetzt musst du in einem neuen Fenster nur noch ros_pi tippen, und alles ist bereit.
ros_pi

---
## WSL: C:\Users\thoma\Apps\start_robot_bridge.bat:
```
python C:\Users\thoma\Apps\ros_bridge.py
```
