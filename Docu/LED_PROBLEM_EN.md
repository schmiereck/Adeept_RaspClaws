# WS2812 LED Problem Diagnosis and Fix

## Problem
The WS2812 RGB LEDs (front and top of robot) are not lighting up / staying dark.

---

## ✅ Implemented Code Improvements

### **1. Better Error Handling in GUIServer.py**

**Before:**
```python
try:
    ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
    ws2812.start()
    ws2812.breath(70,70,255)
except:
    pass  # ❌ Errors are suppressed
```

**Now:**
```python
ws2812 = None
try:
    print("Initializing WS2812 LEDs...")
    ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
    if ws2812.check_spi_state() != 0:
        print("WS2812 initialized successfully")
        ws2812.start()
        ws2812.breath(70, 70, 255)
    else:
        print("Warning: SPI not available for WS2812 LEDs")
        ws2812 = None
except Exception as e:
    print(f"Warning: Could not initialize WS2812 LEDs: {e}")
    ws2812 = None
```

**Benefits:**
- ✅ Error messages are displayed
- ✅ Server starts even without working LEDs
- ✅ Clear diagnosis possible

### **2. Safety Checks in run() Function**

**Before:**
```python
elif 'police' == data:
    ws2812.police()  # ❌ Crashes if ws2812 = None
```

**Now:**
```python
elif 'police' == data:
    if ws2812:
        ws2812.police()  # ✅ Only execute if available
    tcpCliSock.send(('police').encode())
```

---

## 🔍 Error Diagnosis on Raspberry Pi

### **Step 1: Check Server Log**

After pulling and restarting the server:

```bash
cd /home/pi/adeept_raspclaws
git pull
sudo python3 Server/server.py
```

**Look for these messages:**

#### **Case A: LEDs working ✅**
```
Initializing WS2812 LEDs...
WS2812 initialized successfully
WS2812 LEDs set to blue (connected state)
```
→ Software is OK, if LEDs still dark → **Hardware problem**

#### **Case B: SPI not available ⚠️**
```
Initializing WS2812 LEDs...
Warning: SPI not available for WS2812 LEDs
```
→ **SPI is not enabled** (see solution below)

#### **Case C: Error during initialization ❌**
```
Initializing WS2812 LEDs...
Warning: Could not initialize WS2812 LEDs: [Errno XY] ...
```
→ **Hardware or configuration problem** (see below)

---

## 🔧 Solutions

### **Solution 1: Enable SPI**

**On Raspberry Pi:**

```bash
sudo raspi-config
```

→ **Interface Options** → **SPI** → **Enable**

```bash
sudo reboot
```

**Check after reboot:**

```bash
ls -l /dev/spidev*
```

**Expected output:**
```
crw-rw---- 1 root spi 153, 0 Jan 16 10:00 /dev/spidev0.0
crw-rw---- 1 root spi 153, 1 Jan 16 10:00 /dev/spidev0.1
```

If `/dev/spidev0.0` exists → SPI is enabled ✅

---

### **Solution 2: Check config.txt**

**Open file:**

```bash
sudo nano /boot/firmware/config.txt
```

**Make sure this line is NOT commented out:**

```
dtparam=spi=on
```

If the line is missing or commented with `#`, enable it:

```
dtparam=spi=on
```

**Save:** `Ctrl+O`, `Enter`, `Ctrl+X`

**Reboot:**
```bash
sudo reboot
```

---

### **Solution 3: Check WS2812 Wiring**

**WS2812 LEDs require:**

| Pin | Function | GPIO |
|-----|----------|------|
| VCC | 5V | 5V |
| GND | Ground | GND |
| DIN | Data | GPIO10 (SPI0-MOSI) |

**Check:**
1. Are all cables firmly connected?
2. Is 5V power connected?
3. Is GPIO10 (Pin 19) connected to DIN pin?

**Wiring diagram:**
```
Raspberry Pi          WS2812 LED Strip
Pin 19 (GPIO10) ----> DIN
Pin 2  (5V)     ----> VCC/5V
Pin 6  (GND)    ----> GND
```

---

### **Solution 4: Manual LED Tests**

**Create test script:**

```bash
cd /home/pi/adeept_raspclaws
nano test_leds.py
```

**Content:**
```python
#!/usr/bin/env python3
import sys
sys.path.append('/home/pi/adeept_raspclaws/Server')
import RobotLight as robotLight
import time

print("Testing WS2812 LEDs...")

try:
    ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
    
    if ws2812.check_spi_state() != 0:
        print("SPI initialized successfully")
        ws2812.start()
        
        print("Setting LEDs to RED...")
        ws2812.set_all_led_color_data(255, 0, 0)
        ws2812.show()
        time.sleep(2)
        
        print("Setting LEDs to GREEN...")
        ws2812.set_all_led_color_data(0, 255, 0)
        ws2812.show()
        time.sleep(2)
        
        print("Setting LEDs to BLUE...")
        ws2812.set_all_led_color_data(0, 0, 255)
        ws2812.show()
        time.sleep(2)
        
        print("Turning LEDs OFF...")
        ws2812.set_all_led_color_data(0, 0, 0)
        ws2812.show()
        
        ws2812.led_close()
        print("Test completed successfully!")
    else:
        print("ERROR: SPI not available")
except Exception as e:
    print(f"ERROR: {e}")
```

**Execute:**
```bash
sudo python3 test_leds.py
```

**Expected result:**
- LEDs light up **RED** (2 seconds)
- LEDs light up **GREEN** (2 seconds)
- LEDs light up **BLUE** (2 seconds)
- LEDs turn **OFF**

If this works → **Hardware is OK, software problem in GUIServer.py**  
If this doesn't work → **Hardware or wiring problem**

---

## 📋 Deployment

### **Push changes to GitHub:**

```powershell
git add Server/GUIServer.py Docu/LED_PROBLEM.md Docu/LED_PROBLEM_EN.md
git commit -m "Fix: Improve WS2812 LED error handling and diagnostics"
git push
```

### **On Raspberry Pi:**

```bash
cd /home/pi/adeept_raspclaws
git pull
sudo python3 Server/server.py
```

**Watch for output:**
```
Initializing WS2812 LEDs...
WS2812 initialized successfully
```

---

## 🎯 Expected Behavior After Fix

### **At Server Start:**
1. Blue breathing effect
2. On connection: Blue solid

### **On Commands:**
- **Police button:** Red/Blue flashing
- **Police Off:** Back to blue

---

## ❓ Troubleshooting

### **Problem: "Permission denied" accessing SPI**

**Solution:** Add user to `spi` group:

```bash
sudo usermod -a -G spi pi
```

Log out and log back in or reboot.

---

### **Problem: LEDs show wrong colors**

**Possible cause:** Wrong LED type configuration

**In GUIServer.py line 286 change:**

```python
# Default is GRB:
ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)

# If colors are wrong, try RGB:
ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255, sequence='RGB')

# Or other variants: 'RBG', 'GBR', 'BRG', 'BGR'
```

---

### **Problem: Only some LEDs light up**

**Possible cause:** Defective LED in strip or wrong count

**In GUIServer.py line 286:**

```python
ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)  # 16 LEDs
```

If your robot has a different number, change accordingly.

---

## ✅ Summary

After code changes:
1. ✅ Server starts even without working LEDs
2. ✅ Error messages show the problem
3. ✅ No more crashes from missing LEDs
4. ✅ Easy diagnosis possible

**Next step:** Pull on Pi and check log output!
