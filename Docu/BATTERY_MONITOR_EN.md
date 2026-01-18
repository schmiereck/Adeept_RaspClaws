# Battery Monitoring Feature

## Overview

The battery monitoring feature displays the current battery voltage and charge percentage in the GUI with color-coded visual feedback.

## Hardware

**ADC Chip**: ADS7830 (I²C address 0x48)
- 8-channel 8-bit ADC
- Connected via I²C bus

**Voltage Divider**:
- R15 = 3000Ω
- R17 = 1000Ω
- Division Ratio = 0.25

**Battery Specifications**:
- Reference Voltage (Full): 8.4V
- Warning Threshold: 6.0V
- Voltage Range: 6.0V - 8.4V

## Implementation

### Server Side (GUIServer.py)

1. **ADS7830 Initialization**:
   - Attempts to initialize ADS7830 via I²C
   - Falls back to mock mode (returns 0.0V) if hardware not available
   - No crash if battery monitoring unavailable

2. **Voltage Reading**:
   - Function: `get_battery_voltage()`
   - Reads ADC channel 0
   - Converts ADC value to actual voltage using voltage divider ratio
   - Returns voltage as string (2 decimal places)

3. **INFO Protocol Extension**:
   - Old format: `INFO:<CPU_TEMP> <CPU_USE> <RAM_USE>`
   - New format: `INFO:<CPU_TEMP> <CPU_USE> <RAM_USE> <BATTERY_VOLTAGE>`
   - Sent every 1 second to client

### Client Side (GUI.py)

1. **New GUI Element**:
   - Label: `BATTERY_lab`
   - Position: Below RAM Usage (x=400, y=105)
   - Width: 18 characters

2. **Display Format**:
   - "Battery: N/A" - if voltage = 0.0V (monitoring unavailable)
   - "Battery: 7.5V (62%)" - with color-coded background

3. **Color Coding**:
   - **Green (#4CAF50)**: ≥ 60% charge (≥ 7.44V)
   - **Orange (#FF9800)**: 30-60% charge (6.72V - 7.44V)
   - **Red (#F44336)**: < 30% charge (< 6.72V)
   - **Gray (#757575)**: Battery monitoring not available

4. **Percentage Calculation**:
   ```python
   battery_percent = ((voltage - 6.0) / (8.4 - 6.0)) * 100
   ```
   - 6.0V = 0%
   - 8.4V = 100%
   - Clamped to 0-100% range

## Mock Mode

If ADS7830 is not available (development environment without hardware):
- Server prints warning: "⚠ Battery monitoring not available"
- Function returns "0.0" voltage
- GUI displays "Battery: N/A" with gray background
- No errors or crashes

## Testing

### Without Hardware (Mock Mode)
```bash
# Server output
⚠ Battery monitoring not available: [Errno 2] No such file or directory

# GUI displays
Battery: N/A
```

### With Hardware
```bash
# Server output
✓ ADS7830 battery monitor initialized successfully

# GUI displays (examples)
Battery: 8.2V (96%)   # Green background
Battery: 7.0V (42%)   # Orange background
Battery: 6.2V (8%)    # Red background
```

## Backwards Compatibility

The implementation is backwards compatible:
- Old servers (without battery) send 3 values → GUI shows "Battery: N/A"
- New servers send 4 values → GUI shows battery voltage and percentage
- No errors if protocol mismatch

## Dependencies

**Server**:
- `smbus` - for I²C communication
- Part of standard Raspberry Pi OS

**Client**:
- No additional dependencies
- Standard tkinter widgets

## Files Modified

1. `Server/GUIServer.py`:
   - Added ADS7830 class and initialization
   - Added `get_battery_voltage()` function
   - Extended INFO protocol

2. `Client/GUI.py`:
   - Added `BATTERY_VOLTAGE` global variable
   - Added `BATTERY_lab` label widget
   - Extended INFO parsing logic
   - Implemented color-coded display

## Configuration

Battery constants in `GUIServer.py`:
```python
ADCVref = 4.93              # ADC reference voltage
battery_channel = 0          # ADC channel (0-7)
R15 = 3000                   # Voltage divider resistor
R17 = 1000                   # Voltage divider resistor
DivisionRatio = R17 / (R15 + R17)  # = 0.25
```

GUI constants in `GUI.py`:
```python
Vref = 8.4                   # Full battery voltage
WarningThreshold = 6.0       # Empty battery voltage
```

## Troubleshooting

### "Battery: N/A" displayed

**Possible causes**:
1. ADS7830 not connected
2. Wrong I²C address (check with `i2cdetect -y 1`)
3. I²C not enabled (`sudo raspi-config`)
4. Development environment without hardware (normal)

### Wrong voltage readings

**Check**:
1. Voltage divider correct? (R15=3kΩ, R17=1kΩ)
2. ADCVref correct? (measure with multimeter)
3. Battery actually connected to ADC input?

### Color not changing

**Check**:
1. Voltage thresholds: 60% = 7.44V, 30% = 6.72V
2. Tkinter label updates correctly?
3. Check actual voltage with multimeter

## Future Enhancements

- [ ] Low battery warning popup
- [ ] Battery history graph
- [ ] Estimated runtime calculation
- [ ] Battery voltage logging to file
- [ ] Customizable voltage thresholds in GUI

## See Also

- `Examples/07_Voltage/BatteryLevelMonitoring.py` - Standalone example
- `Server/Voltage.py` - Full voltage monitoring with alarms
- `agents.MD` - General system documentation
