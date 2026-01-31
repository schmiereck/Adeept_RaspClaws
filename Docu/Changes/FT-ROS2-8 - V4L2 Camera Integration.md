# FT-ROS2-8: V4L2 Camera Integration in ROS2 Docker Container

**Datum:** 31.01.2026

## Problembeschreibung

Die direkte Verwendung der Raspberry Pi-Kamera (CSI) in einem ROS2 Humble Docker-Container auf Basis von Ubuntu 22.04 ist problematisch. Die `libcamera`-Bibliothek und ihre Python-Bindings (`picamera2`), die für den Zugriff auf die Kamera unter dem aktuellen Raspberry Pi OS erforderlich sind, sind nicht standardmäßig in den Ubuntu 22.04-Repositories verfügbar. Frühere Versuche, diese Abhängigkeiten manuell oder über `pip` zu installieren, führten zu Instabilitäten und Fehlern.

Ziel ist es, einen stabilen Kamera-Stream vom Raspberry Pi zum ROS2-PC zu erhalten, wobei die Verarbeitung innerhalb des Docker-Containers auf dem Pi stattfindet.

## Implementierte Lösung: V4L2-Kompatibilitätsschicht

Anstatt die `libcamera`-spezifischen Bibliotheken zu verwenden, wurde der "V4L2-Kompatibilitäts-Trick" implementiert. Dieser Ansatz behandelt die Pi-Kamera wie eine Standard-USB-Webcam, die über das `Video4Linux2` (V4L2) Interface unter `/dev/video0` angesprochen wird.

Diese Methode bietet maximale Stabilität und Kompatibilität innerhalb des ROS2-Ökosystems, da sie auf bewährten Standard-Treibern aufsetzt.

### Änderungen im Detail

1.  **Dockerfile.ros2:**
    *   **Hinzugefügt:** Das Paket `v4l2-utils` wurde hinzugefügt, um sicherzustellen, dass die V4L2-Tools im Container verfügbar sind.
    *   **Hinzugefügt:** Das ROS2-Paket `ros-humble-v4l2-camera` wurde installiert. Dieses Paket stellt den `v4l2_camera_node` bereit, der die Kamera ausliest und die Bilder als ROS2-Topics veröffentlicht.
    *   **Entfernt:** Alle `libcamera`-spezifischen Abhängigkeiten (`libcamera-dev`, `libcamera-tools`, `libcamera0`) sowie die `pip`-Installation von `picamera2` wurden entfernt, um Konflikte zu vermeiden und das Image sauber zu halten.

2.  **Neues Launch-File (`hexapod_description/launch/camera.launch.py`):**
    *   Es wurde ein neues ROS2-Launch-File erstellt, um den `v4l2_camera_node` zu starten und zu konfigurieren.
    *   Der Node wird so konfiguriert, dass er das Videogerät `/dev/video0` verwendet.

3.  **docker-compose.ros2.yml:**
    *   Der `command` des `raspclaws_camera`-Containers wurde geändert. Anstatt eines benutzerdefinierten Skripts wird nun das neue Launch-File über `ros2 launch` gestartet:
        ```yaml
        command: >
          bash -c "source /opt/ros/humble/setup.bash && 
                ros2 launch hexapod_description camera.launch.py"
        ```
    *   Der `healthcheck`, der auf die alten Skripte prüfte, wurde entfernt, da er nicht mehr relevant ist.
    *   Die Device-Mappings für `/dev/video*` und `/dev/media*` wurden beibehalten, um dem Container den nötigen Hardware-Zugriff zu ermöglichen.

## Ergebnis

Nach dem Build des neuen Docker-Images und dem Start der Container auf dem Raspberry Pi veröffentlicht der `raspclaws_camera`-Container erfolgreich einen komprimierten Bild-Stream.

*   **Topic:** `/raspclaws/camera/image_compressed`
*   **Typ:** `sensor_msgs/msg/CompressedImage`

Dieser Stream kann nun von einem ROS2-Client, z.B. auf dem Entwicklungs-PC, abonniert und angezeigt werden, um den FPV-Stream zu realisieren. Die Lösung ist stabil und in die bestehende ROS2-Architektur integriert.
