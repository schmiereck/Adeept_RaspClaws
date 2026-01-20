# FT25 - Removal of Unused CV Features

**Date:** 2026-01-19  
**Type:** Cleanup / Refactoring  
**Status:** ✅ Completed

## Summary

FindColor, WatchDog, Police LED and Line Following features have been completely removed as they are not needed. This simplifies the codebase and removes unused code.

## Changes

### Client/GUI.py

**Removed Callback Functions:**
- `call_FindColor()` - Toggle find color mode
- `call_WatchDog()` - Toggle watchdog/motion detection mode
- `call_Police()` - Toggle police LED mode
- `call_Smooth()` - Already removed (Smooth Mode is now standard)
- `all_btn_red()` - Helper function
- `all_btn_normal()` - Helper function

**Removed GUI Elements:**
- RGB Color Sliders (R, G, B)
- Color Set Button
- CV FL Scales (lip1, lip2, err)
- CV FL Button
- LineColorSwitch Button
- FindColor Button [X]
- WatchDog Button [C]
- Police Button [B]

**Removed Functions:**
- `EC_send()` - EC value setter
- `EC_default()` - EC default resetter
- `scale_FL()` - Line following UI builder
- `R_send()`, `G_send()`, `B_send()` - RGB slider callbacks
- `call_SET()` - Color set sender
- `call_CVFL()` - CV line following toggle
- `call_WB()` - White/Black line switch

**Removed Global Variables:**
- `var_lip1`, `var_lip2`, `var_err` - Line following parameters
- `var_R`, `var_G`, `var_B` - RGB color values
- `var_ec` - EC value
- `canvas_show` - Color preview canvas
- `Btn_FindColor`, `Btn_WatchDog`, `Btn_Police` - Button references

**Removed Connection Thread Handlers:**
- `elif 'findColor' in car_info` - FindColor activation
- `elif 'motionGet' in car_info` - WatchDog activation
- `elif 'police' == car_info` - Police LED activation
- `elif 'policeOff' == car_info` - Police LED deactivation
- `elif 'CVFL' in car_info` - Line following activation
- `elif 'stopCV' in car_info` - CV features deactivation

**Cleaned Global Declaration in loop():**
- From: `canvas_ultra, var_lip1, var_lip2, var_err, var_R, var_B, var_G, var_ec, Btn_Police, Btn_FindColor, Btn_WatchDog, Btn_Fun5, Btn_Fun6`
- To: Only required variables

### Server/FPV.py

**Removed Global Variables:**
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

**Removed Functions:**
- `map()` - Input mapping function
- `findLineCtrl()` - Line following control
- `cvFindLine()` - Line detection with OpenCV

**Removed FPV Class Methods:**
- `FindColor()` - FindColor Mode Setter
- `WatchDog()` - WatchDog Mode Setter
- `FindLineMode()` - Line Following Mode Setter
- `UltraData()` - Ultrasonic data Setter
- `colorFindSet()` - Color detection parameter setter
- `servoMove()` - Servo Kalman-Filter movement
- `changeMode()` - Mode text setter

**Removed Logic from capture_thread():**
- **FindColor Mode:**
  - HSV Color Space Conversion
  - Color Masking with `colorLower`/`colorUpper`
  - Contour Detection
  - Target tracking with Servo Movement
  - "Target Detected" / "Target Detecting" Overlay

- **WatchDog Mode:**
  - Grayscale Conversion
  - Gaussian Blur
  - Background Model (`avg`)
  - Frame Delta Calculation
  - Motion Detection with Contours
  - Bounding Box Drawing
  - GPIO Switch Activation on Motion
  - 0.5s Timeout for Switch Deactivation

- **FindLine Mode:**
  - `cvFindLine()` Frame Processing
  - Alternative JPEG Encoding for Line Frame

**Simplified capture_thread():**
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

## Remaining Features

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

## Benefits

1. **Reduced Complexity:**
   - GUI has ~250 lines less code
   - FPV.py has ~180 lines less code
   - No unused variables anymore

2. **Improved Maintainability:**
   - Clear focus on used features
   - Less code = fewer bugs
   - Easier code navigation

3. **Better Performance:**
   - No unused if-branches in video loop
   - Lower memory footprint
   - No unnecessary OpenCV calculations

4. **Cleaner GUI:**
   - More space for important controls
   - Less confusion from unused buttons
   - Focus on movement + monitoring

## Migration

No user action required - features were not in use.

If someone needs one of these features:
- Consult Git history (FT25 commit)
- Restore code from previous commit
- Alternative: WebServer.py still has some CV features

## Tested

- ✅ GUI starts without errors
- ✅ No syntax errors
- ✅ Connection thread runs
- ✅ Video stream works
- ✅ All remaining buttons work

## Next Steps

- Server/GUIServer.py: Remove command handlers for FindColor/WatchDog/Police
- Documentation update in README
- Optional: Clean up unused imports (imutils, numpy in FPV.py)
