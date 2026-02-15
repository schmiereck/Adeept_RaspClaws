# Systemd Service Configurations for RaspClaws

This directory contains the systemd service definitions for the RaspClaws robot servers.

## Services

### 1. GUI Server (`gui_server.service`)

**Location on Pi**: `/etc/systemd/system/gui_server.service`

**Purpose**: TCP-based GUI control server with video streaming

**Service Definition**: See `gui_server.service`

**Commands**:
```bash
# Start/Stop/Restart
sudo systemctl start gui_server.service
sudo systemctl stop gui_server.service
sudo systemctl restart gui_server.service

# Status and Logs
sudo systemctl status gui_server.service
sudo journalctl -u gui_server.service -f
```

---

### 2. ROS Server (`ros_server.service`)

**Location on Pi**: `/etc/systemd/system/ros_server.service`

**Purpose**: ROS2-based control server with topics/services/actions

**Service Definition**: See `ros_server.service`

**Startup Script**: `run_rosserver_systemd.sh`
- Wrapper that calls `start_rosserver_wrapper.sh`
- Activates ROS2 environment (micromamba ros_env)
- Sets ROS_DOMAIN_ID=1
- Starts ROSServer.py

**Commands**:
```bash
# Start/Stop/Restart
sudo systemctl start ros_server.service
sudo systemctl stop ros_server.service
sudo systemctl restart ros_server.service

# Status and Logs
sudo systemctl status ros_server.service
sudo journalctl -u ros_server.service -f
```

---

## Installation Instructions

### Install GUI Server Service

```bash
# Copy service file to systemd
sudo cp systemd/gui_server.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable gui_server.service

# Start service
sudo systemctl start gui_server.service
```

### Install ROS Server Service

```bash
# Copy service file to systemd
sudo cp systemd/ros_server.service /etc/systemd/system/

# Ensure startup script is executable
chmod +x /home/pi/Adeept_RaspClaws/run_rosserver_systemd.sh
chmod +x /home/pi/Adeept_RaspClaws/start_rosserver_wrapper.sh

# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable ros_server.service

# Start service
sudo systemctl start ros_server.service
```

---

## Troubleshooting

### Service won't start

Check logs:
```bash
sudo journalctl -u gui_server.service -n 50 --no-pager
sudo journalctl -u ros_server.service -n 50 --no-pager
```

### Permission errors

Ensure scripts are executable:
```bash
chmod +x /home/pi/Adeept_RaspClaws/Server/*.py
chmod +x /home/pi/Adeept_RaspClaws/*.sh
```

### Port already in use

Check for running processes:
```bash
sudo lsof -i :10223  # GUI Server port
sudo lsof -i :5555   # Video stream port
```

### ROS environment not activated

Check if micromamba is properly configured:
```bash
/usr/local/bin/micromamba env list
micromamba activate ros_env
```

---

## Service Status

Check both services:
```bash
sudo systemctl status gui_server.service ros_server.service
```

Check which services are enabled:
```bash
systemctl list-unit-files | grep -E 'gui_server|ros_server'
```

---

## Notes

- Both services use `Restart=always` for automatic recovery
- Services run as user `pi` (not root)
- Logs are sent to systemd journal (use `journalctl` to view)
- Services start automatically on boot if enabled
- Working directory is `/home/pi/Adeept_RaspClaws`

---

**Last Updated**: 2026-02-16  
**Author**: FT47 CommandHandler Refactoring
