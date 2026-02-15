#!/bin/bash
# Wrapper script for GUI Server (optional)
# Currently the service starts GUIServer.py directly,
# but this script can be used if environment setup is needed

cd "$(dirname "$0")"
exec /usr/bin/python3 ./Server/GUIServer.py "$@"
