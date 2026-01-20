#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Test script to verify movement button functionality"""

import tkinter as tk
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Test callbacks
test_log = []

def test_forward(event):
    msg = "✓ Forward button clicked"
    print(msg)
    test_log.append(msg)
    label.config(text=msg, bg='#4CAF50')

def test_backward(event):
    msg = "✓ Backward button clicked"
    print(msg)
    test_log.append(msg)
    label.config(text=msg, bg='#4CAF50')

def test_left(event):
    msg = "✓ Left button clicked"
    print(msg)
    test_log.append(msg)
    label.config(text=msg, bg='#4CAF50')

def test_right(event):
    msg = "✓ Right button clicked"
    print(msg)
    test_log.append(msg)
    label.config(text=msg, bg='#4CAF50')

def test_release(event):
    msg = "✓ Button released"
    print(msg)
    test_log.append(msg)
    label.config(text=msg, bg='#FF9800')

# Create test window
root = tk.Tk()
root.title('Movement Button Test')
root.geometry('400x400')
root.config(bg='#000000')

# Status label
label = tk.Label(root, text='Click a button to test', width=40, height=2,
                 fg='#FFFFFF', bg='#757575', font=('Arial', 12))
label.place(x=50, y=20)

# Test buttons
color_text = '#E1F5FE'
color_btn = '#0277BD'

Btn0 = tk.Button(root, width=8, text='Forward', fg=color_text, bg=color_btn, relief='ridge')
Btn1 = tk.Button(root, width=8, text='Backward', fg=color_text, bg=color_btn, relief='ridge')
Btn2 = tk.Button(root, width=8, text='Left', fg=color_text, bg=color_btn, relief='ridge')
Btn3 = tk.Button(root, width=8, text='Right', fg=color_text, bg=color_btn, relief='ridge')

Btn0.place(x=150, y=100)
Btn1.place(x=150, y=150)
Btn2.place(x=80, y=150)
Btn3.place(x=220, y=150)

# Bind test callbacks
Btn0.bind('<ButtonPress-1>', test_forward)
Btn0.bind('<ButtonRelease-1>', test_release)
Btn1.bind('<ButtonPress-1>', test_backward)
Btn1.bind('<ButtonRelease-1>', test_release)
Btn2.bind('<ButtonPress-1>', test_left)
Btn2.bind('<ButtonRelease-1>', test_release)
Btn3.bind('<ButtonPress-1>', test_right)
Btn3.bind('<ButtonRelease-1>', test_release)

# Keyboard bindings
root.bind('<KeyPress-w>', test_forward)
root.bind('<KeyRelease-w>', test_release)
root.bind('<KeyPress-s>', test_backward)
root.bind('<KeyRelease-s>', test_release)
root.bind('<KeyPress-a>', test_left)
root.bind('<KeyRelease-a>', test_release)
root.bind('<KeyPress-d>', test_right)
root.bind('<KeyRelease-d>', test_release)

# Info label
info = tk.Label(root, text='Use mouse or keyboard (W/A/S/D)',
                fg=color_text, bg='#000000', font=('Arial', 10))
info.place(x=80, y=250)

# Log display
log_text = tk.Text(root, width=40, height=5, bg='#212121', fg=color_text)
log_text.place(x=50, y=290)

def update_log():
    log_text.delete('1.0', tk.END)
    log_text.insert('1.0', '\n'.join(test_log[-5:]))
    root.after(100, update_log)

update_log()

print("Test GUI started - Click buttons or use W/A/S/D keys")
root.mainloop()
print(f"\nTotal events captured: {len(test_log)}")
