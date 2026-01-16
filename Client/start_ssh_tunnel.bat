@echo off
echo Starting SSH Tunnel...
echo Please enter your Raspberry Pi password when prompted.
echo Keep this window open while using the robot!
echo.
ssh -L 10223:localhost:10223 -L 5555:localhost:5555 pi@192.168.2.126
pause
