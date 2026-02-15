#!/bin/bash
# Symlink wrapper for systemd service
# Points to the actual startup script
exec "$(dirname "$0")/start_rosserver_wrapper.sh" "$@"
