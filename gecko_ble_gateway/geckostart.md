#!/bin/bash
#/etc/init.d/geckostart

### BEGIN INIT INFO
# Provides:         geckostart
# Required-Start:   
# Required-Stop:    
# Default-Start:    2 3 4 5
# Default-Stop:     0 1 6
# Short-Description:  Starts Gecko_RxGateway on system startup
# Description you need to copy this into /etc/init.d/  then run: sudo chmod 755 /etc/init.d/geckostart then register it to run at startup: sudo update-rc.d geckostart defaults
### END INIT INFO

screen -S gecko -d -m
screen -r gecko -X stuff "echo abc101 | sudo -S python3 /home/geckoadmin_pi/gecko-gw/python/gecko_RxGateway.py
"
