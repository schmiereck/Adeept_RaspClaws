#!/usr/bin/env python3
# File name   : CommandHandler.py
# Description : Central command handler for both GUIServer and ROSServer
# Author      : GitHub Copilot
# Date        : 2026/02/15

"""
Central Command Handler for Adeept RaspClaws Robot

This module provides a unified command handling interface used by both
GUIServer.py (TCP socket) and ROSServer.py (ROS2 topics/services).

Purpose:
- Eliminates code duplication between servers
- Provides thread-safe access to Move.py hardware API
- Ensures consistent command handling across different interfaces

Thread Safety:
- Uses threading.Lock() for all Move.py calls
- Allows GUIServer and ROSServer to run in parallel without conflicts

Architecture:
    GUIServer.py  →  CommandHandler  →  Move.py (Hardware)
    ROSServer.py  →  CommandHandler  →  Move.py (Hardware)
"""

import threading
import sys
import os

# Add parent directory to path for protocol.py import
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from protocol import *

# Import Move module (hardware API)
try:
    import Move as move
    MOVE_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Move module not available: {e}")
    print("Running in MOCK MODE - hardware control disabled")
    MOVE_AVAILABLE = False
    move = None


class CommandHandler:
    """
    Central command handler for robot control.
    
    Thread-safe wrapper around Move.py that can be used by multiple servers.
    """
    
    def __init__(self):
        """Initialize command handler with thread lock."""
        self._lock = threading.Lock()
        self._initialized = False
        
        # State tracking (for logging/debugging)
        self._last_movement_command = None
        self._last_camera_command = None
        self._current_speed = 35  # Default from Move.py
        self._current_arc_factor = 0.5  # Default from Move.py
        
        print("[CommandHandler] Initialized (thread-safe mode)")
    
    def is_move_available(self):
        """Check if Move module is available (not in mock mode)."""
        return MOVE_AVAILABLE
    
    # ==================== Movement Commands ====================
    
    def handle_movement_command(self, command, speed=None):
        """
        Handle movement commands (forward, backward, left, right, arc, stand).
        
        Args:
            command (str): Movement command from protocol.py (e.g., CMD_FORWARD)
            speed (int, optional): Movement speed override (10-100)
        
        Returns:
            bool: True if command was handled successfully, False otherwise
        """
        if not MOVE_AVAILABLE:
            print(f"[CommandHandler] MOCK: Movement command '{command}'")
            return True
        
        with self._lock:
            try:
                # Set speed if provided
                if speed is not None:
                    move.set_movement_speed(speed)
                    self._current_speed = speed
                
                # Call Move.py's handle_movement_command (handles resume logic)
                result = move.handle_movement_command(command)
                
                if result:
                    self._last_movement_command = command
                    print(f"[CommandHandler] Movement: '{command}' (speed={self._current_speed})")
                
                return result
                
            except Exception as e:
                print(f"[CommandHandler] Error in movement command '{command}': {e}")
                import traceback
                traceback.print_exc()
                return False
    
    # ==================== Camera Commands ====================
    
    def handle_camera_command(self, command):
        """
        Handle camera movement commands (up, down, left, right, home).
        
        Args:
            command (str): Camera command from protocol.py (e.g., CMD_LOOK_UP)
        
        Returns:
            bool: True if command was handled successfully, False otherwise
        """
        if not MOVE_AVAILABLE:
            print(f"[CommandHandler] MOCK: Camera command '{command}'")
            return True
        
        with self._lock:
            try:
                pre_lr = getattr(move, 'Left_Right_input', None)
                pre_ud = getattr(move, 'Up_Down_input', None)
                print(f"[CommandHandler] Camera cmd='{command}' pre: L/R={pre_lr}, U/D={pre_ud}")
                if command == CMD_LOOK_UP:
                    move.look_up()
                elif command == CMD_LOOK_DOWN:
                    move.look_down()
                elif command == CMD_LOOK_LEFT:
                    move.look_left()
                elif command == CMD_LOOK_RIGHT:
                    move.look_right()
                elif command == CMD_LOOK_HOME:
                    move.look_home()
                elif command == CMD_STEADY_CAMERA:
                    move.commandInput(command)
                elif command == CMD_STEADY_CAMERA_OFF:
                    move.commandInput(command)
                elif command == CMD_SMOOTH_CAM:
                    move.commandInput(command)
                elif command == CMD_SMOOTH_CAM_OFF:
                    move.commandInput(command)
                else:
                    return False  # Unknown camera command
                
                self._last_camera_command = command
                post_lr = getattr(move, 'Left_Right_input', None)
                post_ud = getattr(move, 'Up_Down_input', None)
                print(f"[CommandHandler] Camera: '{command}' post: L/R={post_lr}, U/D={post_ud}")
                return True
                
            except Exception as e:
                print(f"[CommandHandler] Error in camera command '{command}': {e}")
                import traceback
                traceback.print_exc()
                return False
    
    # ==================== Speed & Arc Control ====================
    
    def set_movement_speed(self, speed):
        """
        Set movement speed.
        
        Args:
            speed (int): Speed value (10-100, clamped in Move.py)
        
        Returns:
            bool: True if successful
        """
        if not MOVE_AVAILABLE:
            print(f"[CommandHandler] MOCK: Set speed {speed}")
            return True
        
        with self._lock:
            try:
                move.set_movement_speed(speed)
                self._current_speed = speed
                print(f"[CommandHandler] Speed: {speed}")
                return True
            except Exception as e:
                print(f"[CommandHandler] Error setting speed: {e}")
                return False
    
    def set_arc_factor(self, arc_factor):
        """
        Set arc movement factor (curvature).
        
        Args:
            arc_factor (float): Arc factor (0.0-1.0, clamped in Move.py)
        
        Returns:
            bool: True if successful
        """
        if not MOVE_AVAILABLE:
            print(f"[CommandHandler] MOCK: Set arc factor {arc_factor}")
            return True
        
        with self._lock:
            try:
                move.set_arc_factor(arc_factor)
                self._current_arc_factor = arc_factor
                print(f"[CommandHandler] Arc factor: {arc_factor:.2f}")
                return True
            except Exception as e:
                print(f"[CommandHandler] Error setting arc factor: {e}")
                return False
    
    def get_current_speed(self):
        """Get current movement speed (cached value)."""
        return self._current_speed
    
    def get_current_arc_factor(self):
        """Get current arc factor (cached value)."""
        return self._current_arc_factor
    
    # ==================== Power Management ====================
    
    def set_servo_standby(self, enable):
        """
        Enable or disable servo standby mode.
        
        Args:
            enable (bool): True to enter standby (stop PWM), False to wake up
        
        Returns:
            bool: True if successful
        """
        if not MOVE_AVAILABLE:
            print(f"[CommandHandler] MOCK: Servo standby={enable}")
            return True
        
        with self._lock:
            try:
                if enable:
                    move.standby()
                    print("[CommandHandler] 🔋 Servo STANDBY activated")
                else:
                    move.wakeup()
                    print("[CommandHandler] ⚡ Servo WAKEUP activated")
                return True
            except Exception as e:
                print(f"[CommandHandler] Error in servo standby: {e}")
                return False
    
    def set_camera_pause(self, enable):
        """
        Pause or resume camera stream.
        
        Args:
            enable (bool): True to pause, False to resume
        
        Returns:
            bool: True if successful
        """
        if not MOVE_AVAILABLE:
            print(f"[CommandHandler] MOCK: Camera pause={enable}")
            return True
        
        with self._lock:
            try:
                if enable:
                    move.commandInput(CMD_CAMERA_PAUSE)
                    print("[CommandHandler] 📷 Camera PAUSED")
                else:
                    move.commandInput(CMD_CAMERA_RESUME)
                    print("[CommandHandler] 📷 Camera RESUMED")
                return True
            except Exception as e:
                print(f"[CommandHandler] Error in camera pause: {e}")
                return False
    
    # ==================== Utility Methods ====================
    
    def get_status(self):
        """
        Get current command handler status.
        
        Returns:
            dict: Status information
        """
        return {
            'move_available': MOVE_AVAILABLE,
            'last_movement': self._last_movement_command,
            'last_camera': self._last_camera_command,
            'current_speed': self._current_speed,
            'current_arc_factor': self._current_arc_factor
        }


# Singleton instance (optional, can also be instantiated per server)
_global_handler = None

def get_global_handler():
    """Get or create global CommandHandler instance."""
    global _global_handler
    if _global_handler is None:
        _global_handler = CommandHandler()
    return _global_handler
