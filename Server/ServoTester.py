#!/usr/bin/env python3
# File name   : ServoTester.py
# Description : Standalone servo testing tool with GUI
# Author      : GitHub Copilot
# Date        : 2026/01/18

import sys
import tkinter as tk
from tkinter import ttk
import Adafruit_PCA9685

# ==================== Servo Configuration ====================

# Servo channel mapping (from Move.py and RPIservo.py)
SERVO_CHANNELS = {
    'Head Up/Down': 0,      # Vertical head movement
    'Head Left/Right': 1,   # Horizontal head rotation
    'Leg 1': 4,             # Front-right leg
    'Leg 2': 5,             # Back-right leg
    'Leg 3': 6,             # Back-left leg
    'Leg 4': 7,             # Front-left leg
    'Arm 1': 8,             # Right arm
    'Arm 2': 9,             # Left arm
}

# Default ranges (can be adjusted per servo if needed)
PWM_MIN = 100
PWM_MAX = 600
PWM_DEFAULT = 300

# PCA9685 settings
PCA9685_ADDRESS = 0x40
PCA9685_BUSNUM = 1
PCA9685_FREQUENCY = 50

# ==================== PCA9685 Initialization ====================

class ServoPWM:
    """Wrapper for PCA9685 PWM controller with mock mode support"""

    def __init__(self):
        self.mock_mode = False
        self.pwm = None
        self.current_values = {}

        try:
            self.pwm = Adafruit_PCA9685.PCA9685(address=PCA9685_ADDRESS, busnum=PCA9685_BUSNUM)
            self.pwm.set_pwm_freq(PCA9685_FREQUENCY)
            print(f"✓ PCA9685 initialized successfully on address 0x{PCA9685_ADDRESS:02X}")
        except (OSError, IOError) as e:
            print(f"\033[38;5;3mWarning:\033[0m Could not initialize PCA9685: {e}")
            print("Running in MOCK MODE - servo commands will be simulated")
            self.mock_mode = True
        except (ModuleNotFoundError, ImportError) as e:
            print(f"\033[38;5;3mWarning:\033[0m Platform not supported (missing module): {e}")
            print("Running in MOCK MODE - servo commands will be simulated")
            print("(This is normal on Windows - hardware access only works on Raspberry Pi)")
            self.mock_mode = True

    def set_pwm(self, channel, on, off):
        """Set PWM value for a servo channel"""
        if self.mock_mode:
            self.current_values[channel] = off
            print(f"[MOCK] Channel {channel}: PWM={off}")
        else:
            try:
                self.pwm.set_pwm(channel, on, off)
                self.current_values[channel] = off
            except Exception as e:
                print(f"Error setting PWM on channel {channel}: {e}")

    def get_current_value(self, channel):
        """Get current PWM value for a channel"""
        return self.current_values.get(channel, PWM_DEFAULT)


# ==================== GUI Application ====================

class ServoTesterGUI:
    """Main GUI application for servo testing"""

    def __init__(self, root):
        self.root = root
        self.root.title('Adeept RaspClaws - Servo Tester')
        self.root.geometry('600x700')
        self.root.configure(bg='#1E1E1E')

        # Initialize PWM controller
        self.pwm_controller = ServoPWM()

        # Store slider widgets
        self.sliders = {}
        self.value_labels = {}

        # Create GUI
        self.create_widgets()

        # Set all servos to default position on startup
        self.reset_all_servos()

    def create_widgets(self):
        """Create all GUI widgets"""

        # Title
        title_label = tk.Label(
            self.root,
            text='Servo Tester',
            font=('Arial', 18, 'bold'),
            fg='#E1F5FE',
            bg='#1E1E1E'
        )
        title_label.pack(pady=10)

        # Mode indicator
        mode_text = "MOCK MODE" if self.pwm_controller.mock_mode else "HARDWARE MODE"
        mode_color = "#FF9800" if self.pwm_controller.mock_mode else "#4CAF50"
        mode_label = tk.Label(
            self.root,
            text=mode_text,
            font=('Arial', 12),
            fg='#FFFFFF',
            bg=mode_color
        )
        mode_label.pack(pady=5)

        # Info label
        info_label = tk.Label(
            self.root,
            text='Move sliders to test each servo',
            font=('Arial', 10),
            fg='#B0BEC5',
            bg='#1E1E1E'
        )
        info_label.pack(pady=5)

        # Scrollable frame for servos
        canvas = tk.Canvas(self.root, bg='#1E1E1E', highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.root, orient='vertical', command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg='#1E1E1E')

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        # Create servo controls
        for servo_name, channel in SERVO_CHANNELS.items():
            self.create_servo_control(scrollable_frame, servo_name, channel)

        canvas.pack(side='left', fill='both', expand=True, padx=10, pady=10)
        scrollbar.pack(side='right', fill='y')

        # Control buttons at bottom
        self.create_control_buttons()

    def create_servo_control(self, parent, servo_name, channel):
        """Create control widgets for a single servo"""

        # Frame for this servo
        frame = tk.Frame(parent, bg='#2C2C2C', relief='raised', borderwidth=1)
        frame.pack(fill='x', padx=5, pady=5)

        # Servo name and channel
        header_frame = tk.Frame(frame, bg='#2C2C2C')
        header_frame.pack(fill='x', padx=10, pady=5)

        name_label = tk.Label(
            header_frame,
            text=f'{servo_name} (Ch {channel})',
            font=('Arial', 11, 'bold'),
            fg='#E1F5FE',
            bg='#2C2C2C',
            anchor='w'
        )
        name_label.pack(side='left')

        # Current value display
        value_label = tk.Label(
            header_frame,
            text=f'{PWM_DEFAULT}',
            font=('Arial', 11),
            fg='#4CAF50',
            bg='#2C2C2C',
            anchor='e'
        )
        value_label.pack(side='right')
        self.value_labels[channel] = value_label

        # Slider
        slider_frame = tk.Frame(frame, bg='#2C2C2C')
        slider_frame.pack(fill='x', padx=10, pady=5)

        slider = tk.Scale(
            slider_frame,
            from_=PWM_MIN,
            to=PWM_MAX,
            orient='horizontal',
            length=500,
            resolution=1,
            command=lambda value, ch=channel: self.on_slider_change(ch, value),
            bg='#2C2C2C',
            fg='#E1F5FE',
            troughcolor='#424242',
            highlightthickness=0,
            relief='flat'
        )
        slider.set(PWM_DEFAULT)
        slider.pack(fill='x')
        self.sliders[channel] = slider

        # Min/Max labels
        range_frame = tk.Frame(frame, bg='#2C2C2C')
        range_frame.pack(fill='x', padx=10, pady=2)

        min_label = tk.Label(
            range_frame,
            text=f'Min: {PWM_MIN}',
            font=('Arial', 8),
            fg='#B0BEC5',
            bg='#2C2C2C',
            anchor='w'
        )
        min_label.pack(side='left')

        max_label = tk.Label(
            range_frame,
            text=f'Max: {PWM_MAX}',
            font=('Arial', 8),
            fg='#B0BEC5',
            bg='#2C2C2C',
            anchor='e'
        )
        max_label.pack(side='right')

    def create_control_buttons(self):
        """Create control buttons at the bottom"""

        button_frame = tk.Frame(self.root, bg='#1E1E1E')
        button_frame.pack(fill='x', padx=10, pady=10)

        # Reset All button
        reset_btn = tk.Button(
            button_frame,
            text='Reset All to Default',
            command=self.reset_all_servos,
            bg='#2196F3',
            fg='#FFFFFF',
            font=('Arial', 11, 'bold'),
            relief='raised',
            borderwidth=2,
            padx=20,
            pady=10
        )
        reset_btn.pack(side='left', padx=5)

        # Set All to Min button
        min_btn = tk.Button(
            button_frame,
            text='All to Min',
            command=self.set_all_min,
            bg='#FF9800',
            fg='#FFFFFF',
            font=('Arial', 11),
            relief='raised',
            borderwidth=2,
            padx=20,
            pady=10
        )
        min_btn.pack(side='left', padx=5)

        # Set All to Max button
        max_btn = tk.Button(
            button_frame,
            text='All to Max',
            command=self.set_all_max,
            bg='#FF9800',
            fg='#FFFFFF',
            font=('Arial', 11),
            relief='raised',
            borderwidth=2,
            padx=20,
            pady=10
        )
        max_btn.pack(side='left', padx=5)

        # Quit button
        quit_btn = tk.Button(
            button_frame,
            text='Quit',
            command=self.on_quit,
            bg='#F44336',
            fg='#FFFFFF',
            font=('Arial', 11, 'bold'),
            relief='raised',
            borderwidth=2,
            padx=20,
            pady=10
        )
        quit_btn.pack(side='right', padx=5)

    def on_slider_change(self, channel, value):
        """Handle slider value change"""
        pwm_value = int(float(value))
        self.pwm_controller.set_pwm(channel, 0, pwm_value)

        # Update value label
        if channel in self.value_labels:
            self.value_labels[channel].config(text=f'{pwm_value}')

    def reset_all_servos(self):
        """Reset all servos to default position"""
        print("Resetting all servos to default position...")
        for channel, slider in self.sliders.items():
            slider.set(PWM_DEFAULT)
            self.pwm_controller.set_pwm(channel, 0, PWM_DEFAULT)
            if channel in self.value_labels:
                self.value_labels[channel].config(text=f'{PWM_DEFAULT}')
        print("✓ All servos reset to default")

    def set_all_min(self):
        """Set all servos to minimum position"""
        print("Setting all servos to minimum position...")
        for channel, slider in self.sliders.items():
            slider.set(PWM_MIN)
            self.pwm_controller.set_pwm(channel, 0, PWM_MIN)
            if channel in self.value_labels:
                self.value_labels[channel].config(text=f'{PWM_MIN}')
        print("✓ All servos set to minimum")

    def set_all_max(self):
        """Set all servos to maximum position"""
        print("Setting all servos to maximum position...")
        for channel, slider in self.sliders.items():
            slider.set(PWM_MAX)
            self.pwm_controller.set_pwm(channel, 0, PWM_MAX)
            if channel in self.value_labels:
                self.value_labels[channel].config(text=f'{PWM_MAX}')
        print("✓ All servos set to maximum")

    def on_quit(self):
        """Handle quit button"""
        print("\nShutting down servo tester...")
        self.reset_all_servos()
        print("Goodbye!")
        self.root.quit()


# ==================== Main Entry Point ====================

def main():
    """Main entry point"""
    print("="*60)
    print("Adeept RaspClaws - Servo Tester")
    print("="*60)
    print()
    print("Starting GUI...")
    print()

    try:
        root = tk.Tk()
        app = ServoTesterGUI(root)
        root.mainloop()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\033[38;5;1mError:\033[0m {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
