@echo off
echo Starting SSH Tunnel...
start /B ssh -L 10223:localhost:10223 -L 5555:localhost:5555 pi@192.168.2.126
timeout /t 2
