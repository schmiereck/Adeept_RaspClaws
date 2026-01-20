# FT45 - GUI Clean Shutdown

**Date:** 2026-01-20  
**Type:** Bugfix  
**Component:** Client/GUI.py

## Problem

Closing the GUI window did not properly terminate the application:
- TCP socket connection remained open
- Footage-GUI process kept running
- ZMQ Footage socket was not closed
- **The application continued running in the background even though the window was closed**
- Subsequent commands caused error messages like:
  ```
  Exception in Tkinter callback
  Traceback (most recent call last):
    File "...\tkinter\__init__.py", line 1705, in __call__
      return self.func(*args)
    File "...\GUI.py", line 175, in call_forward
      send_movement_command('forward', 'c_f_stu')
    File "...\GUI.py", line 167, in send_movement_command
      send_command(command)
    File "...\GUI.py", line 149, in send_command
      tcpClicSock.send(command.encode())
  ```

## Solution

### 1. Added on_closing() Function
A new function was implemented that is called when the window is closed and properly releases all resources:

```python
def on_closing():
    """Clean shutdown when window is closed"""
    global tcpClicSock, footage_socket, footage_process, shutdown_requested
    
    shutdown_requested = True  # Signal that shutdown was requested
    print("Shutting down GUI...")
    
    # Close TCP socket
    if tcpClicSock and tcpClicSock != '':
        tcpClicSock.close()
    
    # Close footage socket
    if footage_socket is not None:
        footage_socket.close()
    
    # Terminate Footage-GUI process
    if footage_process is not None and footage_process.poll() is None:
        footage_process.terminate()
        footage_process.wait(timeout=2)
    
    # Close OpenCV windows
    cv2.destroyAllWindows()
    
    # Close GUI window
    root.destroy()
    
    # Exit application completely
    import sys
    sys.exit(0)
```

### 2. Added shutdown_requested Flag
A global flag was added to signal that the application should be terminated:

```python
shutdown_requested = False  # Flag to signal application shutdown
```

### 3. Modified While Loop
The while loop in the `loop()` function was extended to check after `root.mainloop()` whether a shutdown was requested:

```python
if shutdown_requested:
    print("Exiting application loop...")
    break
```

### 2. Registered Window Close Protocol
The on_closing() function is automatically called when the window is closed:

```python
root.protocol("WM_DELETE_WINDOW", on_closing)
```

### 3. Improved Error Handling in send_command()
The send_command() function now checks if the socket is still valid before attempting to send:

```python
def send_command(command):
    """Send a command to the server"""
    try:
        if tcpClicSock:
            tcpClicSock.send(command.encode())
    except Exception as e:
        print(f"Error sending command '{command}': {e}")
```

### 4. connection_thread() with Error Handling
The connection thread now terminates properly when the socket is closed:

```python
def connection_thread():
    try:
        while 1:
            car_info = (tcpClicSock.recv(BUFSIZ)).decode()
            # ... processing ...
    except Exception as e:
        print(f"Connection thread stopped: {e}")
```

### 5. Consistent Use of send_command()
The functions `call_servo_standby()` and `call_camera_pause()` now use the central send_command() function instead of direct socket calls.

## Changes

**Client/GUI.py:**
- Added `shutdown_requested` flag (line ~68)
- Added `on_closing()` function with `sys.exit(0)` (line ~763-812)
- Registered `root.protocol("WM_DELETE_WINDOW", on_closing)` (line ~814)
- Added `shutdown_requested` to global variable list of `loop()` function (line ~748)
- Extended while loop with shutdown check and `break` (line ~986-989)
- Extended `send_command()` with error handling (line ~149-155)
- Wrapped `connection_thread()` with try-except (line ~507-642)
- Fixed indentation in VIDEO_READY block (line ~515-522)
- `call_servo_standby()` now uses `send_command()` (line ~327-340)
- `call_camera_pause()` now uses `send_command()` (line ~343-356)

## Test Results

### Before Changes:
- ❌ Closing window causes Tkinter error messages
- ❌ Footage-GUI process continues running after exit
- ❌ Socket connections remain open

### After Changes:
- ✅ Window closes cleanly without error messages
- ✅ All processes are properly terminated
- ✅ All socket connections are closed
- ✅ OpenCV windows are closed

## Impact

- **User Experience:** ✅ Improved - no more error messages on close
- **Stability:** ✅ Improved - clean shutdown behavior
- **Performance:** ✅ Improved - no more zombie processes
- **Backward Compatibility:** ✅ Fully maintained

## Open Issues

None.

## Notes

The solution follows the best-practice pattern for Tkinter applications, using the WM_DELETE_WINDOW protocol to execute a clean cleanup routine when closing the window.
