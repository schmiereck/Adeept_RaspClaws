#!/usr/bin/env python3
"""
Simple CLI tool to send camera/servo commands to GUIServer via TCP.

Examples:
  python3 Server/CameraCommandCli.py --host 127.0.0.1 wake home left left up left
  python3 Server/CameraCommandCli.py --host 192.168.2.126 --interactive
"""

import argparse
import socket
import time
import os
import sys

# Allow running from either repo root or Server/ directory.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol import (
    CMD_LOOK_DOWN,
    CMD_LOOK_HOME,
    CMD_LOOK_LEFT,
    CMD_LOOK_RIGHT,
    CMD_LOOK_UP,
    CMD_SERVO_STANDBY,
    CMD_SERVO_WAKEUP,
)


ALIAS_TO_COMMAND = {
    "up": CMD_LOOK_UP,
    "down": CMD_LOOK_DOWN,
    "left": CMD_LOOK_LEFT,
    "right": CMD_LOOK_RIGHT,
    "home": CMD_LOOK_HOME,
    "wake": CMD_SERVO_WAKEUP,
    "standby": CMD_SERVO_STANDBY,
}


def send_sequence(host: str, port: int, commands, delay: float) -> None:
    with socket.create_connection((host, port), timeout=5) as sock:
        for alias in commands:
            cmd = ALIAS_TO_COMMAND.get(alias.lower())
            if cmd is None:
                print(f"Skipping unknown command alias: {alias}")
                continue
            sock.send(cmd.encode())
            print(f"sent {alias:<8} -> {cmd}")
            time.sleep(delay)


def interactive_mode(host: str, port: int, delay: float) -> None:
    print("Interactive mode. Enter commands: up/down/left/right/home/wake/standby/quit")
    with socket.create_connection((host, port), timeout=5) as sock:
        while True:
            user_input = input("> ").strip().lower()
            if user_input in ("quit", "exit", "q"):
                break
            cmd = ALIAS_TO_COMMAND.get(user_input)
            if cmd is None:
                print(f"Unknown command: {user_input}")
                continue
            sock.send(cmd.encode())
            print(f"sent {user_input:<8} -> {cmd}")
            time.sleep(delay)


def main() -> None:
    parser = argparse.ArgumentParser(description="Send camera/servo commands to GUIServer")
    parser.add_argument("commands", nargs="*", help="Sequence of command aliases")
    parser.add_argument("--host", default="127.0.0.1", help="GUIServer host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=10223, help="GUIServer TCP port (default: 10223)")
    parser.add_argument("--delay", type=float, default=0.35, help="Delay between commands in seconds")
    parser.add_argument("--interactive", action="store_true", help="Start interactive command mode")
    args = parser.parse_args()

    if args.interactive:
        interactive_mode(args.host, args.port, args.delay)
    elif args.commands:
        send_sequence(args.host, args.port, args.commands, args.delay)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
