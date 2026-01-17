# Fix: Video Stream Not Working

## Problem

The client successfully connects to the video stream server (ZMQ on port 5555), but no video frames are received. The client continuously shows "Timeout waiting for video frame".

### Symptoms
```
[Footage] Connecting to video stream @ 127.0.0.1:5555
[Footage] Connected to video stream, waiting for frames...
[Footage] Starting video receive loop...
[Footage] ⚠ Timeout waiting for video frame
[Footage] ⚠ Timeout waiting for video frame
...
```

## Root Cause

In `Server/FPV.py`, the frame encoding and sending logic was indented **outside** the `while True:` loop (lines 385-395). This meant these lines were never executed, as they were at the same indentation level as the loop itself.

### Incorrect Structure
```python
while True:
    frame_image = picam2.capture_array()
    # ... image processing ...
    
if FindLineMode:  # ❌ WRONG: Outside the loop!
    encoded, buffer = cv2.imencode('.jpg', frame_findline)
else:
    encoded, buffer = cv2.imencode('.jpg', frame_image)
footage_socket.send(jpg_as_text)
```

## Solution

The frame encoding and sending logic was correctly indented inside the `while True:` loop.

### File: `Server/FPV.py` (lines 385-395)

**Before:**
```python
        if (timestamp - lastMovtionCaptured).seconds >= 0.5:
            switch.switch(1,0)
            switch.switch(2,0)
            switch.switch(3,0)

if FindLineMode:  # ❌ Outside the loop
    frame_findline = cvFindLine(frame_image)
    encoded, buffer = cv2.imencode('.jpg', frame_findline)
else:
    encoded, buffer = cv2.imencode('.jpg', frame_image)
jpg_as_text = base64.b64encode(buffer)
footage_socket.send(jpg_as_text)
time.sleep(0.033)
```

**After:**
```python
        if (timestamp - lastMovtionCaptured).seconds >= 0.5:
            switch.switch(1,0)
            switch.switch(2,0)
            switch.switch(3,0)

    # Frame encoding and sending (MUST be inside the while True loop!)
    if FindLineMode:  # ✅ Inside the loop
        frame_findline = cvFindLine(frame_image)
        encoded, buffer = cv2.imencode('.jpg', frame_findline)
    else:
        encoded, buffer = cv2.imencode('.jpg', frame_image)
    jpg_as_text = base64.b64encode(buffer)
    footage_socket.send(jpg_as_text)
    
    # Limit frame rate to reduce CPU load (~30 FPS = 33ms per frame)
    time.sleep(0.033)  # 33ms = ~30 FPS, reduces CPU load significantly
```

## Additional Changes

### Client/GUI.py
- `run_open()` now uses `subprocess.Popen()` instead of `subprocess.run()` for non-blocking start of the Footage-GUI process
- A monitor thread displays the output of the Footage-GUI process in real-time
- Proper cleanup of the Footage process on exit

### Client/Footage-GUI.py
- Better debug output added (frame counter every 30 frames)
- Timeout handling with `poll()` for better error diagnosis
- Error handling with detailed traceback output

## Result

After this change:
- The FPV thread continuously sends frames via ZMQ
- The client receives and displays frames (~30 FPS)
- CPU load on the Raspberry Pi remains low due to frame rate limiting

## Testing

1. Start server on Raspberry Pi: `sudo systemctl restart robot_server.service`
2. Start client on Windows and connect
3. After the "VIDEO_READY" signal, the video window should appear and show frames

## Related Files
- `Server/FPV.py` - Frame capture and video streaming
- `Client/GUI.py` - Main application with video thread management
- `Client/Footage-GUI.py` - Video display window
