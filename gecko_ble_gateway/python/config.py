# 	Colyright 2018-2022 Nokia
#
# 	All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# 	Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).

# These are the global variables used for the Gecko Gateway
import dbus
try:
  from gi.repository import GObject
except ImportError:
  import gobject as GObject
  
import pika
import time
import json
import requests
import sys
import datetime
import os
import random

##################################################################################
### NOTE THAT YOU HAVE TO CHANGE "ADAPTER_MAC" TO YOUR OWN HARDWARE ADDRESS!!! ###
##################################################################################

def variables_init():  
    ####### User Changeable Variables ########
    global save_local
    save_local = 0
    
    global use_WWS
    use_WWS = 1
    
    global gatewayID
    gatewayID = "default"
    
    global wwsIP
    wwsIP = '10.4.86.150' # '10.4.82.52'  # 
    
    global ADAPTER_MAC
        
    global ADAPTER_NAME
    ADAPTER_NAME = '/org/bluez/hci0'

    global connectAll
    connectAll = 1
    
    global hardCodedGeckos
    hardCodedGeckos = []
    
    ######### Fixed Variables ############
    global bus
    bus = dbus.SystemBus()

    global mainloop
    mainloop = GObject.MainLoop()
    
    global geckos
    geckos = {}
    
    global status
    status = {}
    
    global datafile
    
    global adapter
    
    global GECKO_ADDRESS
    GECKO_ADDRESS =  ADAPTER_NAME + '/dev_'
    
    global channel
    
    global wws
    
    global keyword
    keyword = 'AAAID'
    
class gecko_info(object):
    def __init__(self,MAC=None,dev_address=None,discovered=0,wwsConnected=0,bleConnected=0,saveLocal=0,myGecko=0,LED=1):
        self.MAC = MAC
        self.dev_address = dev_address
        self.discovered = discovered
        self.wwsConnected = wwsConnected
        self.bleConnected = bleConnected
        self.saveLocal = saveLocal
        self.myGecko = myGecko
        self.callback = None
        self.Tx = None
        self.LED = LED
       




    