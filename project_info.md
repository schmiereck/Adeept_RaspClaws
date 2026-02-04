# Adeept_RaspClaws Project Information (Automated Update)

## ⚠️ WICHTIG: Dokumentations-Richtlinien für dieses Projekt

**Diese Datei ist der zentrale Wissensstand für das gesamte Projekt.**

Bei **jeder Arbeitssitzung** (auch mit KI-Assistenten):
1. **VOR dem Start:** Diese Datei lesen, um den aktuellen Stand zu verstehen
2. **WÄHREND der Arbeit:** Alle Erkenntnisse, durchgeführten Aktionen und Probleme hier dokumentieren
3. **NACH jedem wichtigen Schritt:** Datei aktualisieren mit:
   - Was wurde getan?
   - Was wurde herausgefunden?
   - Welche Probleme gibt es noch?
   - Was sind die nächsten Schritte?

**Ziel:** Jede neue Session kann nahtlos dort weitermachen, wo die letzte aufgehört hat.

---

## Raspberry Pi Setup Status (Dienstag, 4. Februar 2026)

**Operating System:** Raspberry Pi OS Lite arm64 (Debian Trixie)
**Python Environment:** RoboStack with Micromamba 2.0+ (using `ros_env` virtual environment)
**Project Directory on Pi:** `/home/pi/Adeept_RaspClaws`

---
## Development Workflow

To update code on the Raspberry Pi, the following workflow **MUST** be followed:

1.  **Modify Locally:** Edit and test all code changes on the local Windows machine.
2.  **Commit Changes:** Use `git add` and `git commit` to save the changes to the local repository.
3.  **Push to Remote:** Use `git push` to upload the committed changes to the remote Git repository (e.g., GitHub).
4.  **Pull on Raspberry Pi:**
    *   SSH into the Raspberry Pi (`ssh pi@192.168.2.126`).
    *   Navigate to the project directory (`cd /home/pi/Adeept_RaspClaws`).
    *   Use `git pull` to download and apply the changes from the remote repository.

**DO NOT** modify files directly on the Raspberry Pi or use `scp` to transfer individual files, as this will cause the local and remote Git repositories to become out of sync.

---

### Key Learnings and Current Status:

1.  **Codebase Migration:**
    *   Successfully migrated `Adafruit_PCA9685` to `adafruit-circuitpython-pca9685` in `Server/RPIservo.py` and `Server/Move.py`.
    *   Successfully migrated legacy `mpu6050` to `adafruit-circuitpython-mpu6050` in `Server/Move.py`.
    *   Removed redundant `Adafruit_PCA9685` initialization in `Server/GUIServer.py`.
    *   Corrected `SyntaxError` (`ndef` to `def`) in `Server/GUIServer.py`.
    *   Added verbose logging to `Server/GUIServer.py` and `Server/RPIservo.py` for improved diagnostics.
    *   All local codebase changes have been committed and pushed to `origin/master`.

2.  **Dependency Management on Raspberry Pi (`ros_env`):**
    *   `pip` has been upgraded within `ros_env`.
    *   `requirements_native.txt` (including `numpy`, `opencv-python`) dependencies are now successfully installed in `ros_env`.
    *   `smbus`, `RPi.GPIO`, `imutils`, and `picamera2` have been successfully installed into `ros_env`.

3.  **Raspberry Pi System Configuration:**
    *   `build-essential`, `python3-dev`, `swig`, `libgl1-mesa-dri`, `mesa-utils`, `python3-rpi.gpio`, `libcap-dev`, and `libcamera-dev` system packages were installed via `apt-get`.
    *   I2C and SPI interfaces were enabled via `sudo raspi-config nonint do_i2c 0` and `sudo raspi-config nonint do_spi 0`.
    *   User `pi` was added to the `gpio` group (`sudo usermod -a -G gpio pi`).
    *   The Raspberry Pi has been rebooted multiple times after system changes.

---

### Outstanding Issues / Next Steps:

**The `GUIServer` is currently starting, but with camera and LED issues.**

1.  **ADS7830 (Battery Monitor) - HARDWARE NICHT VORHANDEN:**
    *   `No I2C device at address: 0x48` - Die ADS7830 Hardware ist **nicht angeschlossen/nicht vorhanden**.
    *   **Status:** Dies ist KEIN Software-Problem. Die Hardware fehlt physisch.
    *   **Auswirkung:** Batteriemonitor funktioniert nicht, aber dies verhindert nicht den Start des restlichen Servers.
    *   **Aktion:** Keine - akzeptierter Zustand. Code kann so bleiben, Fehler wird ignoriert.

2.  **FPV/Camera Issues (Still Not Working):**
    *   `ModuleNotFoundError: No module named 'libcamera'` persists.
    *   **Recommended Action on Raspberry Pi:**
        1.  **Install `libcamera` Python Bindings and `picamera2` via `apt` (outside micromamba):**
            ```bash
            sudo apt update
            sudo apt install -y python3-libcamera python3-picamera2
            ```
        2.  **Verify system-wide installation:** Run `python3 -c "import libcamera; import picamera2"` to confirm system-wide availability.
        3.  **If using `micromamba`, ensure your environment has access to system site packages:**
            If your `ros_env` was not created with `--with-system-site-packages`, consider recreating it or creating a new environment that clones the base and includes system site packages:
            ```bash
            micromamba create -n ros_camera_env python=<your_python_version> --clone-base --with-system-site-packages
            micromamba activate ros_camera_env
            ```
            Replace `<your_python_version>` with the Python version used in `ros_env` (likely 3.9 or 3.10).
        4.  **Verify within micromamba:** Activate your micromamba environment and run `python -c "import libcamera; import picamera2"`.
    *   **CRITICAL NOTE: Debian Trixie libcamera bug:** Raspberry Pi engineers have confirmed that Debian Trixie currently has a broken `libcamera` stack due to a `libpisp` update. This means that even with correct installation, `libcamera` and `picamera2` might not function correctly until this issue is officially resolved or `libcamera` is manually rebuilt.

3.  **RobotLight (WS2812 LEDs) Issues (Resolved in software, awaiting hardware verification):**
    *   `ModuleNotFoundError: No module named 'spidev'` was resolved by installing `spidev` in `ros_env`.
    *   Actual LED functionality needs to be verified on the hardware.

4.  **Raspberry Pi Stability / SSH:**
    *   The Raspberry Pi occasionally experiences extended boot times and SSH connection timeouts. This complicates remote debugging.
    *   **Recommendation:** Ensure the Raspberry Pi has a stable power supply, a reliable network connection, and that the SSH service is robust.

---

---

## ✅ GUIServer Status (Dienstag, 4. Februar 2026 - 21:47 Uhr)

**Der GUIServer läuft jetzt erfolgreich!**

### Lösung: System-Python 3.13 statt Micromamba

Das Hauptproblem war, dass `libcamera` und `picamera2` für Python 3.13 kompiliert sind (die System-Python-Version), aber die micromamba `ros_env` Python 3.11 verwendet. Native Bibliotheken können nicht zwischen Python-Versionen geteilt werden.

**Implementierte Lösung:**
- GUIServer läuft jetzt mit **system-Python 3.13** (nicht micromamba)
- Alle Dependencies wurden system-weit installiert:
  - `python3-libcamera`, `python3-picamera2` (via apt)
  - `python3-opencv`, `python3-numpy`, `python3-smbus`, `python3-rpi.gpio` (via apt)
  - `imutils`, `psutil`, `pyzmq`, `adafruit-circuitpython-pca9685`, `adafruit-circuitpython-mpu6050`, `spidev` (via pip)

### Aktueller Funktionsstatus:

✅ **Vollständig funktionsfähig:**
- **Kamera (FPV)**: libcamera und picamera2 funktionieren perfekt
- **Video-Stream**: Port 5555 aktiv, Clients können sich verbinden
- **PCA9685 (Servo-Controller)**: Erfolgreich auf Adresse 0x40 initialisiert
- **WS2812 LEDs**: SPI-basierte LEDs initialisiert und aktiv
- **Netzwerk**: IP 192.168.2.126, Command Port 10223 aktiv
- **Client-Verbindung**: Erfolgreich getestet

⚠️ **Bekannte Probleme (nicht kritisch):**
- **ADS7830 (Batteriemonitor)**: Hardware physisch nicht vorhanden (akzeptiert)
- **ROS2**: Nicht verfügbar in system-Python (nur für GUI-Server nicht kritisch)
- **Kleiner Code-Bug**: `'Adeept_SPI_LedPixel' object has no attribute 'breath_status_set'` - verursacht Warnung, aber LEDs funktionieren

### Durchgeführte Code-Änderungen:
1. **Server/GUIServer.py**: FPV und RobotLight Imports wieder aktiviert (waren temporär auskommentiert)
2. **Server/FPV.py**: Tippfehler behoben (`setsopt` → `setsockopt`)

### Start-Befehl für GUIServer:
```bash
ssh pi@192.168.2.126
cd /home/pi/Adeept_RaspClaws/Server
python3 GUIServer.py  # System-Python 3.13, NICHT micromamba!
```

---

### Next Planned Actions (Optional):

1.  ~~**Perform `libcamera` and `picamera2` installation**~~ ✅ ERLEDIGT
2.  ~~**Verify LED functionality**~~ ✅ ERLEDIGT (funktionieren)
3.  ~~**Re-run `GUIServer`**~~ ✅ ERLEDIGT (läuft erfolgreich)
4.  **Optional: Fix LED breath_status_set bug** - Kleiner Code-Fehler in RobotLight.py, nicht kritisch
5.  **Optional: Test mit Client-Anwendung** - GUI-Client verbinden und volle Funktionalität testen