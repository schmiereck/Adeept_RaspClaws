# FT50: Implementation of Arc Left/Right Movement

**Date:** 2026-01-21

**Author:** Gemini

## 1. Problem Description

The previous implementation for left and right turns (`CMD_LEFT`, `CMD_RIGHT`) only allowed the robot to turn on the spot. A feature to drive in a curve, i.e., a combined forward and turning motion, was missing.

Initial attempts to implement this using a `turn_bias` were unsuccessful because the fundamental movement logic was not designed for it, causing the movements to cancel each other out.

## 2. Solution Description

At the user's suggestion, the entire movement logic in `Server/Move.py` was fundamentally refactored to allow for different step lengths (amplitudes) for the left and right sides of the robot. This enables precise control similar to that of a tank or tracked vehicle.

### Core Changes:

1.  **Separate Speed Control:**
    *   The core of the movement is no longer a single `command` string but two separate speed values: `speed_left` and `speed_right`. These variables control the amplitude (step length) for each respective side.

2.  **Centralization in `move_thread`:**
    *   The `move_thread` function now acts as the central control unit. It calculates the appropriate values for `speed_left` and `speed_right` based on the commands received from the client (`CMD_FORWARD`, `CMD_LEFT`, `CMD_FORWARD_LEFT_ARC`, etc.).
    *   **Arc Left:** The step length of the left legs is reduced (`speed_left = -movement_speed * (1 - arc_factor)`), while the right legs maintain their full step length.
    *   **Turn on the Spot:** The speeds are opposing (`speed_left = +speed`, `speed_right = -speed`), causing the robot to turn in place.

3.  **Simplification of `calculate_target_positions`:**
    *   This function has been radically simplified. It no longer contains any `if/elif` logic for different commands.
    *   It takes `speed_left` and `speed_right` directly as parameters and calculates the horizontal leg positions based on the respective side-specific amplitude.
    *   The gait synchronization (which legs are in the air and when) is maintained via the `phase`, which remains identical for all legs.

4.  **Removal of Redundant Code:**
    *   All old, specific helper functions (`_get_turn_leg_positions`, `_get_arc_leg_positions`, etc.) were removed.

## 3. Implemented Changes

-   **`Server/Move.py`:**
    *   Introduction of the global variable `arc_factor`.
    *   Complete refactoring of `move_thread`, `execute_movement_step`, and `calculate_target_positions`.
    *   Removal of the now-obsolete `_get...` helper functions.
-   **`Client/GUI.py`:**
    *   Addition of "Arc Left" (`Q`) and "Arc Right" (`E`) buttons to the user interface.
-   **`protocol.py`:**
    *   Definition of the new commands `CMD_FORWARD_LEFT_ARC` and `CMD_FORWARD_RIGHT_ARC`.

## 4. Result

The robot can now execute proper curved movements (arc movements) to the left and right. The overall movement logic is cleaner, more flexible, and more easily adaptable for future optimizations. The fundamental functionality of turning on the spot is preserved.

## 5. Follow-up Changes (2026-01-21)

To provide more control over the new movement capabilities, the following enhancements were made:

### 5.1. Dynamic Arc Factor Control

-   **Problem:** The `arc_factor` was a hard-coded constant in `Move.py`, requiring a code change to adjust the turning radius.
-   **Solution:** A slider was added to the GUI to allow for dynamic adjustment of the `arc_factor` during runtime.
-   **Changes:**
    -   **`protocol.py`:** Added a new command `CMD_SET_ARC_FACTOR` to communicate the value from the client to the server.
    -   **`Client/GUI.py`:**
        -   Introduced a new slider for the `arc_factor` with a range from 0.0 to 1.0.
        -   The GUI layout was adjusted to accommodate the new slider.
        -   An `on_arc_factor_change` callback was implemented to send the new value to the server.
    -   **`Server/GUIServer.py`:** Added `handle_arc_factor_command` to receive the new command and pass the value to the `Move` module.
    -   **`Server/Move.py`:** Implemented a `set_arc_factor` function to safely update the global `arc_factor` variable with clamping.

### 5.2. Increased Maximum Speed

-   **Problem:** The maximum movement speed was limited to 60, which was not fast enough.
-   **Solution:** The maximum speed was increased to 100.
-   **Changes:**
    -   **`Client/GUI.py`:** The range of the speed slider was updated from `10-60` to `10-100`.
    -   **`Server/Move.py`:** The clamping in the `set_movement_speed` function was adjusted to allow values up to 100.
    -   **`Server/GUIServer.py`:** The clamping in `handle_speed_command` was removed to avoid duplicate clamping and to respect the new limit set in `Move.py`.