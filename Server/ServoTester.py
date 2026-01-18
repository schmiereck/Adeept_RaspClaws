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
# Each leg has 2 servos: one for rotation (horizontal) and one for height (vertical)

# Layout structure to match hardware
# Left Side (Channels 0-5)          Right Side (Channels 6-11)
# Front-Left (Left I): Ch 0-1       Front-Right (Right I): Ch 6-7
# Back-Left (Left II): Ch 2-3       Back-Right (Right II): Ch 8-9
# (Left III): Ch 4-5                (Right III): Ch 10-11

SERVO_LAYOUT = {
    'left': {
        'Front-Left (Left I)': {
            'Rotation': 0,
            'Height': 1
        },
        'Back-Left (Left II)': {
            'Rotation': 2,
            'Height': 3
        },
        'Left III': {
            'Rotation': 4,
            'Height': 5
        }
    },
    'right': {
        'Front-Right (Right I)': {
            'Rotation': 6,
            'Height': 7
        },
        'Back-Right (Right II)': {
            'Rotation': 8,
            'Height': 9
        },
        'Right III': {
            'Rotation': 10,
            'Height': 11
        }
    },
    'head': {
        'Head': {
            'Up/Down': 12,
            'Left/Right': 13
        }
    }
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
        self.root.geometry('850x600')  # Kompakter: 850x600 statt 900x750
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

        # Title (kleiner)
        title_label = tk.Label(
            self.root,
            text='Servo Tester - Hardware Layout',
            font=('Arial', 14, 'bold'),  # 14 statt 18
            fg='#E1F5FE',
            bg='#1E1E1E'
        )
        title_label.pack(pady=3)  # 3 statt 10

        # Mode indicator (kleiner)
        mode_text = "MOCK MODE" if self.pwm_controller.mock_mode else "HARDWARE MODE"
        mode_color = "#FF9800" if self.pwm_controller.mock_mode else "#4CAF50"
        mode_label = tk.Label(
            self.root,
            text=mode_text,
            font=('Arial', 9),  # 9 statt 12
            fg='#FFFFFF',
            bg=mode_color
        )
        mode_label.pack(pady=2)  # 2 statt 5

        # Info label (kleiner)
        info_label = tk.Label(
            self.root,
            text='Layout reflects robot hardware: Left side | Right side',
            font=('Arial', 8),  # 8 statt 10
            fg='#B0BEC5',
            bg='#1E1E1E'
        )
        info_label.pack(pady=2)  # 2 statt 5

        # Main container for 2-column layout
        main_frame = tk.Frame(self.root, bg='#1E1E1E')
        main_frame.pack(fill='both', expand=True, padx=5, pady=3)  # 5,3 statt 10,10

        # Create left and right columns
        self.create_side_column(main_frame, 'left', 'LEFT SIDE', 0)
        self.create_side_column(main_frame, 'right', 'RIGHT SIDE', 1)

        # Create head section (centered below)
        self.create_head_section(self.root)

        # Control buttons at bottom
        self.create_control_buttons()

    def create_side_column(self, parent, side, title, column):
        """Create a column for left or right side servos"""

        # Column frame
        column_frame = tk.Frame(parent, bg='#1E1E1E')
        column_frame.grid(row=0, column=column, sticky='nsew', padx=3)  # 3 statt 5
        parent.grid_columnconfigure(column, weight=1)

        # Title (kleiner)
        title_label = tk.Label(
            column_frame,
            text=title,
            font=('Arial', 11, 'bold'),  # 11 statt 14
            fg='#4CAF50' if side == 'left' else '#FF6D00',
            bg='#1E1E1E'
        )
        title_label.pack(pady=2)  # 2 statt 5

        # Scrollable canvas for servos
        canvas = tk.Canvas(column_frame, bg='#1E1E1E', highlightthickness=0, height=400)  # 400 statt 500
        scrollbar = tk.Scrollbar(column_frame, orient='vertical', command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg='#1E1E1E')

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        # Create servo controls for this side
        for leg_name, servos in SERVO_LAYOUT[side].items():
            self.create_leg_group(scrollable_frame, leg_name, servos)

        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

    def create_leg_group(self, parent, leg_name, servos):
        """Create a group of servos for one leg"""

        # Group frame (kompakter)
        group_frame = tk.Frame(parent, bg='#2C2C2C', relief='raised', borderwidth=1)  # borderwidth 1 statt 2
        group_frame.pack(fill='x', padx=3, pady=3)  # 3,3 statt 5,10

        # Leg name header (kleiner)
        header_label = tk.Label(
            group_frame,
            text=leg_name,
            font=('Arial', 9, 'bold'),  # 9 statt 12
            fg='#FFD700',
            bg='#2C2C2C',
            anchor='w'
        )
        header_label.pack(fill='x', padx=5, pady=2)  # 5,2 statt 10,5

        # Create servo control for each servo in this leg
        for servo_function, channel in servos.items():
            self.create_servo_control(group_frame, f"{servo_function}", channel, compact=True)

    def create_head_section(self, parent):
        """Create head servo section (centered)"""

        head_frame = tk.Frame(parent, bg='#1E1E1E')
        head_frame.pack(fill='x', padx=10, pady=2)  # pady 2 statt 5

        # Title (kleiner)
        title_label = tk.Label(
            head_frame,
            text='HEAD SERVOS',
            font=('Arial', 11, 'bold'),  # 11 statt 14
            fg='#2196F3',
            bg='#1E1E1E'
        )
        title_label.pack(pady=2)  # 2 statt 5

        # Container for head servos
        head_controls = tk.Frame(head_frame, bg='#2C2C2C', relief='raised', borderwidth=1)  # borderwidth 1 statt 2
        head_controls.pack(fill='x', padx=100, pady=2)  # pady 2 statt 5

        # Create head servo controls
        for leg_name, servos in SERVO_LAYOUT['head'].items():
            for servo_function, channel in servos.items():
                self.create_servo_control(head_controls, f"{servo_function}", channel, compact=True)

    def create_servo_control(self, parent, servo_name, channel, compact=False):
        """Create control widgets for a single servo"""

        # Frame for this servo (kompakter)
        frame = tk.Frame(parent, bg='#3C3C3C' if compact else '#2C2C2C', relief='flat' if compact else 'raised', borderwidth=1)
        frame.pack(fill='x', padx=3, pady=1)  # 3,1 statt 5,3/5

        # Servo name and channel
        header_frame = tk.Frame(frame, bg='#3C3C3C' if compact else '#2C2C2C')
        header_frame.pack(fill='x', padx=3, pady=1)  # 3,1 statt 5,3

        name_label = tk.Label(
            header_frame,
            text=f'{servo_name} (Ch {channel})',
            font=('Arial', 8 if compact else 9),  # 8/9 statt 10/11
            fg='#E1F5FE',
            bg='#3C3C3C' if compact else '#2C2C2C',
            anchor='w'
        )
        name_label.pack(side='left')

        # Current value display
        value_label = tk.Label(
            header_frame,
            text=f'{PWM_DEFAULT}',
            font=('Arial', 8 if compact else 9),  # 8/9 statt 10/11
            fg='#4CAF50',
            bg='#3C3C3C' if compact else '#2C2C2C',
            anchor='e',
            width=5
        )
        value_label.pack(side='right')
        self.value_labels[channel] = value_label

        # Slider (kompakter)
        slider = tk.Scale(
            frame,
            from_=PWM_MIN,
            to=PWM_MAX,
            orient='horizontal',
            length=250 if compact else 400,  # 250/400 statt 300/500
            resolution=1,
            command=lambda value, ch=channel: self.on_slider_change(ch, value),
            bg='#3C3C3C' if compact else '#2C2C2C',
            fg='#E1F5FE',
            troughcolor='#424242',
            highlightthickness=0,
            relief='flat',
            showvalue=0,
            width=8  # Schmalerer Slider
        )
        slider.set(PWM_DEFAULT)
        slider.pack(fill='x', padx=3, pady=1)  # 3,1 statt 5,2
        self.sliders[channel] = slider

        # Min/Max labels (nur wenn nicht compact) - noch kleiner
        if not compact:
            range_frame = tk.Frame(frame, bg='#3C3C3C' if compact else '#2C2C2C')
            range_frame.pack(fill='x', padx=5, pady=1)  # pady 1 statt 2

            min_label = tk.Label(
                range_frame,
                text=f'Min: {PWM_MIN}',
                font=('Arial', 7),  # 7 statt 8
                fg='#B0BEC5',
                bg='#3C3C3C' if compact else '#2C2C2C',
                anchor='w'
            )
            min_label.pack(side='left')

            max_label = tk.Label(
                range_frame,
                text=f'Max: {PWM_MAX}',
                font=('Arial', 7),  # 7 statt 8
                fg='#B0BEC5',
                bg='#3C3C3C' if compact else '#2C2C2C',
                anchor='e'
            )
            max_label.pack(side='right')

    def create_control_buttons(self):
        """Create control buttons at the bottom"""

        button_frame = tk.Frame(self.root, bg='#1E1E1E')
        button_frame.pack(fill='x', padx=5, pady=3)  # 5,3 statt 10,10

        # Reset All button (kompakter)
        reset_btn = tk.Button(
            button_frame,
            text='Reset All',  # Kürzerer Text
            command=self.reset_all_servos,
            bg='#2196F3',
            fg='#FFFFFF',
            font=('Arial', 9, 'bold'),  # 9 statt 11
            relief='raised',
            borderwidth=1,  # 1 statt 2
            padx=15,  # 15 statt 20
            pady=5   # 5 statt 10
        )
        reset_btn.pack(side='left', padx=3)  # 3 statt 5

        # Set All to Min button (kompakter)
        min_btn = tk.Button(
            button_frame,
            text='All Min',  # Kürzerer Text
            command=self.set_all_min,
            bg='#FF9800',
            fg='#FFFFFF',
            font=('Arial', 9),  # 9 statt 11
            relief='raised',
            borderwidth=1,
            padx=15,
            pady=5
        )
        min_btn.pack(side='left', padx=3)

        # Set All to Max button (kompakter)
        max_btn = tk.Button(
            button_frame,
            text='All Max',  # Kürzerer Text
            command=self.set_all_max,
            bg='#FF9800',
            fg='#FFFFFF',
            font=('Arial', 9),
            relief='raised',
            borderwidth=1,
            padx=15,
            pady=5
        )
        max_btn.pack(side='left', padx=3)

        # Quit button (kompakter)
        quit_btn = tk.Button(
            button_frame,
            text='Quit',
            command=self.on_quit,
            bg='#F44336',
            fg='#FFFFFF',
            font=('Arial', 9, 'bold'),
            relief='raised',
            borderwidth=1,
            padx=15,
            pady=5
        )
        quit_btn.pack(side='right', padx=3)

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
