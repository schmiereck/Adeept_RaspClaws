# Adeept_RaspClaws Project Information (Automated Update)

## Raspberry Pi Setup Status (Montag, 2. Februar 2026)

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

1.  **ADS7830 (Battery Monitor) - Hardware / I2C Address Issue:**
    *   `No I2C device at address: 0x48` - The ADS7830 chip is not being detected on the I2C bus. This requires physical verification of the sensor's connection and its actual I2C address (e.g., using a multimeter, checking hardware documentation).
    *   This is not a code bug, but a hardware/configuration issue that prevents the battery monitoring from working. The rest of the server should ideally proceed.

2.  **FPV/Camera Issues (Still Not Working):**
    *   `picamera2` is installed, but `ModuleNotFoundError: No module named 'libcamera'` persists.
    *   This indicates an issue with the `libcamera` system library or its Python bindings not being correctly found/linked.
    *   **Action Required:** Verify `libcamera` Python bindings, check system-level camera configurations (e.g., in `/boot/firmware/config.txt`), and ensure the "legacy camera driver" is disabled on `raspi-config`.

3.  **RobotLight (WS2812 LEDs) Issues (FIXED):**
    *   `ModuleNotFoundError: No module named 'spidev'` was resolved by installing `spidev` in `ros_env`.
    *   Now need to verify actual LED functionality.

4.  **Raspberry Pi Stability / SSH:**
    *   The Raspberry Pi occasionally experiences extended boot times and SSH connection timeouts. This complicates remote debugging.
    *   **Recommendation:** Ensure the Raspberry Pi has a stable power supply, a reliable network connection, and that the SSH service is robust.

---

### Next Planned Actions:

1.  **Re-run `GUIServer` with current state:** Observe the output for both camera and LED related messages.
2.  **Investigate `libcamera` Python bindings:** Search for how to correctly install/link `libcamera` Python bindings for `picamera2` in a micromamba environment on Raspberry Pi OS Lite arm64 (Debian Trixie).
3.  **Verify LED functionality:** Check if `RobotLight` starts correctly and the LEDs are functional after `spidev` installation.
4.  **Clean up temporary prints:** Once camera/LED issues are better understood, remove temporary debug print statements from `GUIServer.py`.