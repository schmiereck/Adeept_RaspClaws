# Fix: Second Connect Not Working - Server Won't Accept New Connections

## Problem

On the **first connect**, everything works perfectly (video stream works). But on the **second connect** (after GUI restart):
- GUI shows: "Connected" and "Waiting for video server to initialize..."
- Server logs show: **No** new connection
- No video window appears
- GUI hangs and waits forever

## Root Cause

There were **two critical bugs** in the server code:

### Bug #1: `continue` instead of `break` on empty socket (MAIN PROBLEM!)

**File:** `Server/GUIServer.py`, Line 168-169 (function `run()`)

**Buggy Code:**
```python
data = str(tcpCliSock.recv(BUFSIZ).decode())
if not data:
    continue  # ❌ WRONG!
```

**Problem:**
When a client disconnects, `recv()` returns an **empty string**. The code then does `continue`, which means the `while True:` loop **keeps running** and waits forever for data from the dead socket!

**Consequence:**
- First connect: Works, then close GUI
- `recv()` returns empty string
- Code does `continue` → **Loop continues**
- `run()` **NEVER** returns
- Second connect: Server **CANNOT** accept new connection because it's still stuck in the old `run()` loop!

**Solution:**
```python
data = str(tcpCliSock.recv(BUFSIZ).decode())
if not data:
    # Empty string means socket was closed by client
    print("Client disconnected (empty recv)")
    sys.stdout.flush()
    break  # ✅ Exit loop to wait for new connection
```

With `break`, the `run()` function exits the loop, returns to the main loop, closes the sockets, and accepts a new connection.

### Bug #2: `VIDEO_READY` Signal Lost on Second Connect

**File:** `Server/GUIServer.py`, function `info_send_client()`

**Problems in Original Version:**
1. Signal was sent only **5 times** (too few for slow clients)
2. On error, immediately called `break` → no more retries
3. No debug output → impossible to debug
4. `print()` was buffered → logs didn't appear immediately

**Solution:**
```python
def info_send_client():
    print("INFO_SEND_CLIENT: Starting to send VIDEO_READY signals...")
    sys.stdout.flush()  # Force output immediately
    
    success_count = 0
    for i in range(10):  # Try 10 times (increased from 5)
        try:
            tcpCliSock.send('VIDEO_READY\n'.encode())
            success_count += 1
            print(f"✅ Sent VIDEO_READY signal (attempt {i+1}/10, success #{success_count})")
            sys.stdout.flush()
            time.sleep(1)
        except Exception as e:
            print(f"⚠ Failed to send VIDEO_READY (attempt {i+1}/10): {e}")
            sys.stdout.flush()
            # Don't break - continue trying!  ← IMPORTANT!
            time.sleep(1)
    
    print(f"INFO_SEND_CLIENT: Finished VIDEO_READY phase ({success_count}/10 successful)")
    sys.stdout.flush()
    
    # Then continue with regular info sending
    while 1:
        try:
            info_data = 'INFO:' + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(1)
        except Exception as e:
            print(f"⚠ Failed to send INFO: {e}")
            sys.stdout.flush()
            break  # Exit thread on persistent error
```

**Improvements:**
- 10 attempts instead of 5
- On error, **doesn't** abort but continues trying
- Comprehensive debug output with `sys.stdout.flush()`
- Counts successful sends

## Changes in Detail

### File: `Server/GUIServer.py`

**Line 11:** Added `sys` import
```python
import sys
```

**Lines 106-137:** Improved `info_send_client()` function
- More attempts (10 instead of 5)
- Better error handling
- Debug output with `sys.stdout.flush()`

**Lines 168-172:** Critical bugfix in `run()` function
```python
if not data:
    # Empty string means socket was closed by client
    print("Client disconnected (empty recv)")
    sys.stdout.flush()
    break  # ← CHANGED from 'continue' to 'break'
```

## How the Bugs Were Found

### Phase 1: Log Analysis
Server logs on second connect showed:
- **No** "waiting for connection..." message
- **No** "...connected from..." message
- **No** VIDEO_READY signals

→ **Conclusion:** Server doesn't accept new connection!

### Phase 2: Code Analysis
Main loop in `__main__` block:
```python
while 1:
    # Accept connection
    tcpCliSock, addr = tcpSerSock.accept()
    print('...connected from :', addr)
    
    # Run command processing
    run()  # ← Stuck here on second connect!
    
    # Cleanup and loop back
    print("Client disconnected. Cleaning up...")
```

The `run()` function should return when client disconnects. **BUT:** Logs never showed "Client disconnected. Cleaning up..."

→ **Conclusion:** `run()` doesn't return!

### Phase 3: `run()` Analysis
```python
def run():
    while True:
        data = str(tcpCliSock.recv(BUFSIZ).decode())
        if not data:
            continue  # ← HERE IS THE BUG!
```

When `recv()` returns empty string (socket closed), does `continue` → loop runs forever!

## Server Architecture (for clarity)

```
Server Start
    │
    ├─ Start FPV Thread (runs continuously, sends video frames)
    │
    └─ Main Loop (while 1):
        │
        ├─ Accept new connection: tcpSerSock.accept()
        │
        ├─ Call run()
        │   │
        │   ├─ Start info_send_client Thread
        │   │   └─ Send 10x VIDEO_READY
        │   │   └─ Send INFO messages
        │   │
        │   └─ Command Loop (while True):
        │       └─ Receive and process commands
        │       └─ On disconnect: break → return to main loop
        │
        ├─ Cleanup (close sockets)
        │
        └─ Back to step 1 (accept new connection)
```

**With the bug:** `run()` never returns → no new connection possible
**With the fix:** `run()` returns → new connection can be accepted ✅

## Result

After this fix:
- ✅ **First connect**: Video works
- ✅ **Second connect**: Video works
- ✅ **Third, fourth, ... connect**: Always works
- ✅ Server accepts unlimited consecutive connections

## Test Scenarios

### Scenario 1: Multiple GUI Restarts
1. Start GUI and connect → Video works ✅
2. Close GUI
3. Server logs show: "Client disconnected (empty recv)"
4. Server logs show: "Client disconnected. Cleaning up..."
5. Server logs show: "Ready for new connection..."
6. Restart GUI and connect → Video works ✅
7. Repeat steps 2-6 as many times as desired → Always works ✅

### Scenario 2: Server Restart
1. Restart server
2. Connect GUI → Video works ✅
3. Close and restart GUI → Video works ✅

### Scenario 3: Network Interruption
1. GUI connected, video running
2. Network interruption (e.g., WiFi briefly disconnected)
3. `run()` throws exception → `break` → cleanup
4. Network restored
5. Reconnect GUI → Video works ✅

## Related Files
- `Server/GUIServer.py` - Main server with critical bugfix
- `Client/GUI.py` - Client application (no changes needed)
