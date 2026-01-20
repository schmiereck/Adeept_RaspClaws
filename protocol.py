"""
Protocol Constants for Adeept RaspClaws Communication

This module defines all command strings used for communication between
the GUI Client and the GUIServer. Using constants ensures consistency
and prevents typos.

Author: schmiereck
Date: 2026-01-20
"""

# ==================== Movement Commands ====================
# Commands sent from GUI to control robot movement

# Direction commands (forward/backward)
CMD_FORWARD = 'forward'
CMD_BACKWARD = 'backward'
CMD_DIRECTION_STOP = 'DS'  # Stop forward/backward movement

# Turn commands (left/right rotation)
CMD_LEFT = 'left'
CMD_RIGHT = 'right'
CMD_TURN_STOP = 'TS'  # Stop turning

# Strafe commands (side movement)
CMD_LEFT_SIDE = 'leftside'
CMD_RIGHT_SIDE = 'rightside'

# Internal movement commands (used in Move.py)
MOVE_STAND = 'stand'  # Stand still position
MOVE_NO = 'no'  # No movement command

# Speed commands
CMD_FAST = 'fast'
CMD_SLOW = 'slow'

# Smooth movement mode
CMD_SMOOTH = 'smooth'
CMD_SMOOTH_OFF = 'smoothOff'


# ==================== Camera Commands ====================
# Commands to control camera pan/tilt servos

CMD_LOOK_UP = 'lookUp'
CMD_LOOK_DOWN = 'lookDown'
CMD_LOOK_LEFT = 'lookLeft'
CMD_LOOK_RIGHT = 'lookRight'
CMD_LOOK_HOME = 'lookHome'  # Center camera position

# Camera stop commands
CMD_LR_STOP = 'LRstop'  # Stop left/right pan
CMD_UD_STOP = 'UDstop'  # Stop up/down tilt

# Camera modes
CMD_STEADY_CAMERA = 'steadyCamera'  # Enable camera stabilization
CMD_STEADY_CAMERA_OFF = 'steadyCameraOff'  # Disable camera stabilization
CMD_SMOOTH_CAM = 'smoothCam'  # Enable smooth camera movement
CMD_SMOOTH_CAM_OFF = 'smoothCamOff'  # Disable smooth camera movement


# ==================== Power Management Commands ====================
# Commands to control servo and camera power states

CMD_SERVO_STANDBY = 'servo_standby'  # Put servos in low-power standby mode
CMD_SERVO_WAKEUP = 'servo_wakeup'  # Wake servos from standby
CMD_CAMERA_PAUSE = 'camera_pause'  # Pause video streaming
CMD_CAMERA_RESUME = 'camera_resume'  # Resume video streaming


# ==================== LED Commands ====================
# Commands to control WS2812 RGB LEDs

CMD_POLICE = 'police'  # Police light effect
CMD_POLICE_OFF = 'policeOff'  # Turn off police light effect


# ==================== Switch Commands ====================
# Commands to control GPIO relay switches

CMD_SWITCH_1_ON = 'Switch_1_on'
CMD_SWITCH_1_OFF = 'Switch_1_off'
CMD_SWITCH_2_ON = 'Switch_2_on'
CMD_SWITCH_2_OFF = 'Switch_2_off'
CMD_SWITCH_3_ON = 'Switch_3_on'
CMD_SWITCH_3_OFF = 'Switch_3_off'


# ==================== Computer Vision Commands ====================
# Commands for computer vision features

CMD_FIND_COLOR = 'findColor'  # Enable color tracking
CMD_MOTION_GET = 'motionGet'  # Enable motion detection
CMD_CVFL = 'CVFL'  # Find line mode (line following)
CMD_STOP_CV = 'stopCV'  # Stop all CV features


# ==================== Status Response Strings ====================
# Status messages sent FROM server TO client

STATUS_SERVO_STANDBY = 'servoStandby'  # Servos are in standby mode
STATUS_SERVO_WAKEUP = 'servoWakeup'  # Servos are awake
STATUS_CAMERA_PAUSED = 'cameraPaused'  # Camera is paused
STATUS_CAMERA_RESUMED = 'cameraResumed'  # Camera is resumed
STATUS_VIDEO_READY = 'VIDEO_READY'  # Video stream is ready
STATUS_INFO_PREFIX = 'INFO:'  # Prefix for system info messages


# ==================== Command Mapping ====================
# Maps GUI commands to internal Move.py commands

GUI_TO_MOVE_COMMAND_MAP = {
    CMD_FORWARD: CMD_FORWARD,
    CMD_BACKWARD: CMD_BACKWARD,
    CMD_DIRECTION_STOP: MOVE_STAND,
    CMD_LEFT: CMD_LEFT,
    CMD_RIGHT: CMD_RIGHT,
    CMD_TURN_STOP: MOVE_NO,
    CMD_LEFT_SIDE: CMD_LEFT,  # Strafe left -> turn left
    CMD_RIGHT_SIDE: CMD_RIGHT,  # Strafe right -> turn right
}
