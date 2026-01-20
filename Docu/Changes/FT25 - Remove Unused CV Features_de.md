# FT25 - Entfernung nicht benötigter CV Features

**Datum:** 2026-01-19  
**Typ:** Cleanup / Refactoring  
**Status:** ✅ Abgeschlossen

## Zusammenfassung

FindColor, WatchDog, Police LED und Line Following Features wurden komplett entfernt, da sie nicht benötigt werden. Dies vereinfacht die Codebasis und entfernt ungenutzten Code.

## Änderungen

### Client/GUI.py

**Entfernte Callback-Funktionen:**
- `call_FindColor()` - Toggle find color mode
- `call_WatchDog()` - Toggle watchdog/motion detection mode
- `call_Police()` - Toggle police LED mode
- `call_Smooth()` - Bereits entfernt (Smooth Mode ist jetzt Standard)
- `all_btn_red()` - Helper Funktion
- `all_btn_normal()` - Helper Funktion

**Entfernte GUI-Elemente:**
- RGB Color Sliders (R, G, B)
- Color Set Button
- CV FL Scales (lip1, lip2, err)
- CV FL Button
- LineColorSwitch Button
- FindColor Button [X]
- WatchDog Button [C]
- Police Button [B]

**Entfernte Funktionen:**
- `EC_send()` - EC value setter
- `EC_default()` - EC default resetter
- `scale_FL()` - Line following UI builder
- `R_send()`, `G_send()`, `B_send()` - RGB slider callbacks
- `call_SET()` - Color set sender
- `call_CVFL()` - CV line following toggle
- `call_WB()` - White/Black line switch

**Entfernte globale Variablen:**
- `var_lip1`, `var_lip2`, `var_err` - Line following parameters
- `var_R`, `var_G`, `var_B` - RGB color values
- `var_ec` - EC value
- `canvas_show` - Color preview canvas
- `Btn_FindColor`, `Btn_WatchDog`, `Btn_Police` - Button references

**Entfernte Connection Thread Handler:**
- `elif 'findColor' in car_info` - FindColor aktivierung
- `elif 'motionGet' in car_info` - WatchDog aktivierung
- `elif 'police' == car_info` - Police LED aktivierung
- `elif 'policeOff' == car_info` - Police LED deaktivierung
- `elif 'CVFL' in car_info` - Line following aktivierung
- `elif 'stopCV' in car_info` - CV features deaktivierung

**Bereinigte globale Deklaration in loop():**
- Von: `canvas_ultra, var_lip1, var_lip2, var_err, var_R, var_B, var_G, var_ec, Btn_Police, Btn_FindColor, Btn_WatchDog, Btn_Fun5, Btn_Fun6`
- Zu: Nur noch benötigte Variablen

### Server/FPV.py

**Entfernte globale Variablen:**
- `FindColorMode = 0`
- `WatchDogMode = 0`
- `UltraData = 3`
- `CVrun = 1`
- `FindLineMode = 0`
- `linePos_1 = 440`
- `linePos_2 = 380`
- `lineColorSet = 255`
- `findLineError = 20`
- `Threshold = 80`
- `findLineMove = 1`
- `tracking_servo_status = 0`
- `FLCV_Status = 0`
- `colorUpper = np.array([44, 255, 255])`
- `colorLower = np.array([24, 100, 100])`
- `tracking_servo_left = None`
- `tracking_servo_left_mark = 0`
- `tracking_servo_right_mark = 0`
- `servo_left_stop = 0`
- `servo_right_stop = 0`
- `avg = None` - WatchDog background model
- `motionCounter = 0`
- `lastMovtionCaptured`

**Entfernte Funktionen:**
- `map()` - Input mapping Funktion
- `findLineCtrl()` - Line following Steuerung
- `cvFindLine()` - Line detection mit OpenCV

**Entfernte FPV-Klassen-Methoden:**
- `FindColor()` - FindColor Mode Setter
- `WatchDog()` - WatchDog Mode Setter
- `FindLineMode()` - Line Following Mode Setter
- `UltraData()` - Ultrasonic data Setter
- `colorFindSet()` - Color detection Parameter Setter
- `servoMove()` - Servo Kalman-Filter Bewegung
- `changeMode()` - Mode Text Setter

**Entfernte Logik aus capture_thread():**
- **FindColor Mode:**
  - HSV Color Space Conversion
  - Color Masking mit `colorLower`/`colorUpper`
  - Contour Detection
  - Target tracking mit Servo Movement
  - "Target Detected" / "Target Detecting" Overlay

- **WatchDog Mode:**
  - Grayscale Conversion
  - Gaussian Blur
  - Background Model (`avg`)
  - Frame Delta Berechnung
  - Motion Detection mit Contours
  - Bounding Box Drawing
  - GPIO Switch Aktivierung bei Motion
  - 0.5s Timeout für Switch Deaktivierung

- **FindLine Mode:**
  - `cvFindLine()` Frame Processing
  - Alternative JPEG Encoding für Line Frame

**Vereinfachte capture_thread():**
```python
while True:
    frame_image = picam2.capture_array()
    timestamp = datetime.datetime.now()
    cv2.line(frame_image, (300, 240), (340, 240), (128, 255, 128), 1)  # Crosshair
    cv2.line(frame_image, (320, 220), (320, 260), (128, 255, 128), 1)  # Crosshair

    # Frame encoding and sending
    encoded, buffer = cv2.imencode('.jpg', frame_image)
    jpg_as_text = base64.b64encode(buffer)
    footage_socket.send(jpg_as_text)

    time.sleep(0.033)  # ~30 FPS
```

## Verbleibende Features

### GUI:
- ✅ Movement Controls (Forward, Backward, Left, Right)
- ✅ Camera Controls (Up, Down, Left, Right, Home)
- ✅ Steady Camera Mode [Z]
- ✅ Smooth Camera Mode [N]
- ✅ GPIO Switches (Port 1, 2, 3)
- ✅ Power Management (Servo Standby [M], Camera Pause [,])
- ✅ MPU6050 Gyro/Accel Visualization
- ✅ System Info (CPU, RAM, Battery)
- ✅ Connection Management

### Server (FPV):
- ✅ Video Streaming (ZMQ PUB/SUB)
- ✅ Camera Initialization (Picamera2)
- ✅ Crosshair Overlay
- ✅ Power Management (Pause/Resume)
- ✅ FPS Limiting (~30 FPS)

## Vorteile

1. **Reduzierte Komplexität:**
   - GUI hat ~250 Zeilen weniger Code
   - FPV.py hat ~180 Zeilen weniger Code
   - Keine ungenutzten Variablen mehr

2. **Verbesserte Wartbarkeit:**
   - Klare Fokussierung auf genutzte Features
   - Weniger Code = weniger Bugs
   - Einfachere Navigation im Code

3. **Bessere Performance:**
   - Keine ungenutzten if-Branches in der Video-Loop
   - Weniger Memory Footprint
   - Keine unnötigen OpenCV Berechnungen

4. **Aufgeräumte GUI:**
   - Mehr Platz für wichtige Controls
   - Weniger Verwirrung durch ungenutzte Buttons
   - Fokussierung auf Bewegung + Monitoring

## Migration

Keine User-Action erforderlich - Features wurden nicht genutzt.

Falls jemand doch eines der Features benötigt:
- Git History konsultieren (FT25 commit)
- Code aus vorherigem Commit wiederherstellen
- Alternative: WebServer.py hat noch einige CV Features

## Getestet

- ✅ GUI startet ohne Fehler
- ✅ Keine Syntax-Errors
- ✅ Connection Thread läuft
- ✅ Video Stream funktioniert
- ✅ Alle verbleibenden Buttons funktionieren

## Nächste Schritte

- Server/GUIServer.py: Entfernung der Command Handler für FindColor/WatchDog/Police
- Documentation Update in README
- Optional: Unused Imports aufräumen (imutils, numpy in FPV.py)
