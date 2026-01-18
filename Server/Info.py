#!/usr/bin/env/python3
# File name   : Info.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date        : 2025/04/16
import psutil

# Battery monitoring using ADS7830
battery_available = False
adc = None

try:
    import smbus

    class ADS7830(object):
        def __init__(self):
            self.cmd = 0x84
            self.bus = smbus.SMBus(1)
            self.address = 0x48  # ADS7830 default i2c address

        def analogRead(self, chn):  # ADS7830 has 8 ADC input pins, chn:0-7
            value = self.bus.read_byte_data(self.address, self.cmd|(((chn<<2 | chn>>1)&0x07)<<4))
            return value

    # Try to initialize ADS7830
    adc = ADS7830()
    # Test read to verify it works
    test_value = adc.analogRead(0)
    battery_available = True
    print("✓ ADS7830 battery monitor initialized successfully")
except Exception as e:
    print(f"⚠ Battery monitoring not available: {e}")
    battery_available = False

# Battery constants (from Voltage.py)
ADCVref = 4.93
battery_channel = 0
R15 = 3000
R17 = 1000
DivisionRatio = R17 / (R15 + R17)


def get_cpu_tempfunc():
    """ Return CPU temperature """
    result = 0
    mypath = "/sys/class/thermal/thermal_zone0/temp"
    with open(mypath, 'r') as mytmpfile:
        for line in mytmpfile:
            result = line

    result = float(result)/1000
    result = round(result, 1)
    return str(result)


def get_gpu_tempfunc():
    """ Return GPU temperature as a character string"""
    res = os.popen('/opt/vc/bin/vcgencmd measure_temp').readline()
    return res.replace("temp=", "")


def get_cpu_use():
    """ Return CPU usage using psutil"""
    cpu_cent = psutil.cpu_percent()
    return str(cpu_cent)


def get_ram_info():
    """ Return RAM usage using psutil """
    ram_cent = psutil.virtual_memory()[2]
    return str(ram_cent)


def get_swap_info():
    """ Return swap memory  usage using psutil """
    swap_cent = psutil.swap_memory()[3]
    return str(swap_cent)


def get_battery_voltage():
    """ Return battery voltage in Volts """
    global battery_available, adc

    if not battery_available or adc is None:
        return "0.0"  # Return 0.0 if battery monitoring not available

    try:
        ADCValue = adc.analogRead(battery_channel)
        A0Voltage = ADCValue / 255.0 * ADCVref
        actual_battery_voltage = A0Voltage / DivisionRatio
        return str(round(actual_battery_voltage, 2))
    except Exception as e:
        print(f"⚠ Error reading battery voltage: {e}")
        return "0.0"

