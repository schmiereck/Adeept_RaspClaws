# Changes to Fix Connection Problem

## Problem
The server (`GUIServer.py`) tried to establish a **new TCP connection back to the client** on port 2256 to send CPU/RAM information. This connection was blocked by the Windows firewall.

## Solution
The CPU/RAM information is now sent over the **existing connection** that the client established to the server.

---

## Modified Files

### 1. **Server/GUIServer.py**

**Function `info_send_client()` (Line ~87-100):**
- ❌ **Before:** Attempted to create new connection to client on port 2256
- ✅ **Now:** Uses the existing `tcpCliSock` connection
- Info data is marked with the `INFO:` prefix

```python
def info_send_client():
    # Use the existing tcpCliSock connection instead of creating a new one
    while 1:
        try:
            info_data = 'INFO:' + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(1)
        except:
            pass
```

---

### 2. **Client/GUI.py**

**A) Function `connection_thread()` (Line ~301):**
- ✅ **New:** Added processing of `INFO:` messages
- CPU/RAM data is now processed directly in the existing thread

```python
elif car_info.startswith('INFO:'):
    # Process CPU/RAM info from server
    try:
        info_data = car_info[5:].strip()  # Remove 'INFO:' prefix
        info_get = info_data.split()
        if len(info_get) >= 3:
            CPU_TEP, CPU_USE, RAM_USE = info_get[0], info_get[1], info_get[2]
            CPU_TEP_lab.config(text='CPU Temp: %s℃'%CPU_TEP)
            CPU_USE_lab.config(text='CPU Usage: %s'%CPU_USE)
            RAM_lab.config(text='RAM Usage: %s'%RAM_USE)
    except:
        pass
```

**B) Function `Info_receive()` (Line ~382-404):**
- ❌ **Removed:** The separate function is no longer needed

**C) Thread start for `Info_receive()` (Line ~450-453):**
- ❌ **Removed:** The separate thread is no longer started

---

## Benefits
✅ No additional connection needed  
✅ Works behind firewalls/NAT  
✅ Simpler architecture  
✅ Port 2256 no longer needs to be opened  

---

## Next Steps
1. **Commit and push changes:**
   ```bash
   git add Server/GUIServer.py Client/GUI.py Docu/Changes/FT1 - Fix Connection Problem_de.md Docu/Changes/FT1 - Fix Connection Problem_en.md
   git commit -m "Fix: Use existing connection for CPU/RAM info instead of opening new connection"
   git push
   ```

2. **On the Raspberry Pi:**
   ```bash
   cd Adeept_RaspClaws
   git pull
   ```

3. **Testing:**
   - Start client on Windows
   - Start server on Raspberry Pi
   - CPU/RAM information should now be displayed
