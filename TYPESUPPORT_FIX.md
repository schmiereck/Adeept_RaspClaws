# Typesupport Extension Fix for raspclaws_interfaces

## Problem

When building `raspclaws_interfaces` with robostack/micromamba on Raspberry Pi, the Python typesupport modules are compiled without the `.so` extension, causing this error:

```
ModuleNotFoundError: No module named 'raspclaws_interfaces.raspclaws_interfaces_s__rosidl_typesupport_c'
```

## Root Cause

The build system creates ELF shared object files but doesn't add the `.so` extension that Python expects for module imports:
- Created: `raspclaws_interfaces_s__rosidl_typesupport_c` (no extension)
- Expected: `raspclaws_interfaces_s__rosidl_typesupport_c.so`

## Solution

Run the fix script after building the package:

```bash
./fix_typesupport_extensions.sh
```

This creates symbolic links with the correct `.so` extensions.

## When to Apply

- After initial build: `colcon build --packages-select raspclaws_interfaces`
- After rebuilding the package
- After updating the ros2_ws workspace

## Verification

Test that action servers can be created:

```bash
python3 << 'EOF'
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from raspclaws_interfaces.action import HeadPosition

rclpy.init()
node = Node('test')
server = ActionServer(node, HeadPosition, '/test', lambda x: x)
print("SUCCESS!")
rclpy.shutdown()
EOF
```

## Permanent Fix

This issue should be fixed in the CMakeLists.txt or reported to the robostack project. For now, the fix script is a reliable workaround.
