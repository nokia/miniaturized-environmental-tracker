#!/usr/bin/env python3
# 	Colyright 2018-2022 Nokia
#
# 	All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# 	Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).


import traceback
import dbus
try:
  from gi.repository import GObject
except ImportError:
  import gobject as GObject
import sys
import time
import threading
from geckoDataProcessing import *
import config as gv
from ble_discovery import *
import bluezutils
from dbus.mainloop.glib import DBusGMainLoop
import pika
import json
import datetime
from geckoTx_func import *
import wws_controller

#bus = None
#mainloop = None

BLUEZ_SERVICE_NAME = 'org.bluez'
DBUS_OM_IFACE =      'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE =    'org.freedesktop.DBus.Properties'

GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE =    'org.bluez.GattCharacteristic1'

NORDIC_RX_UUID =        '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
#GECKO_ADDRESS =      '/org/bluez/hci0/dev_'  #This makes an address of type '/org/bluez/hci0/dev_FC_1B_E9_E5_90_80'


def new_data_Rx(iface, changed_props, invalidated_props, path=None):
    # iface is the interface ('org.bluez.GattCharacteristic1')
    # changed_props is the changed output data
    logger_id = 0
    dev_name = path[path.find('dev_')+4:path.find('dev_')+21]  # This is the MAC Address
    
    # Try to process new data and save/send to WWS
    try:
        data = bytearray(changed_props['Value'][:])

        # First look at the Packet Header
#        PacketHeaderSize = 4 + 4 + 1 + 1 + 2  # This is for old firmware with 32bit timestamp
        PacketHeaderSize = 4 + 4 + 4 + 1 + 1 + 2
        
        SensorData_PacketHeader = bytearray(data[:PacketHeaderSize])
        
        # Extract the SOP, timestamp, logger_id, data length from the Header
        sop, timestamp, timestamp1, logger_id, length, custom = unpack('IIIBBH', SensorData_PacketHeader)
        
        if logger_id > 6:
            #Something is wrong
            return
        # Check that there is something there, if there is process it
        if sop == 0 and timestamp == 0 and logger_id == 0 and length == 0 and custom == 0:
            print("End of Data...")
        else:
            # I think these two strings should match...
            checkstring = 'StartOfPacket: ' + repr(hex(sop)) + ' MagicNumber: ' + repr(hex(MagicNumber[0]))

            # Compute the length of data in the packet
            align32_length = align32(length)

    ######### Read the raw data and then process it ##########
            SensorData = bytes(data[PacketHeaderSize:PacketHeaderSize+align32_length])
            # Process the data into a readable format
#            print("Logger ID = ", logger_id)
#            print("Data = ", SensorData)
            data = process(logger_id, SensorData)
        
    ######### Add Metadata #############
            # Add Timestamp
            ts = ((timestamp*2**32+timestamp1) / Ticks_per_second)*3600
            data[data['type']] = ts
            sensor_type = data['type'][:-3]
            del data['type']
            
            # Add Device Address
            data.update({'Device' : dev_name})
            
            # Add Header
            data.update({'from': 'mh:com.nokia.sensor.gecko:'+dev_name+':'+sensor_type,'mirror':1})
            
            
    ######### Save the Data or Send it Out ######       
            if gv.save_local:
            # Try to Save to File
                try:
                    save_data(data)
                    if gv.geckos[dev_name].saveLocal != 1:
                        gv.geckos[dev_name].saveLocal = 1
                except Exception as e:
                    print(e)
                    print("Data saving failed")
                
            # Try to send to WWS
            if gv.use_WWS:
                try:
                    routing_key='sensors.gecko.'+data['Device']+'.'+sensor_type
                    
                    # Put the Data on the Queue
                    gv.wws.q.put((data,routing_key))

                    if gv.geckos[dev_name].wwsConnected != 1:
                        gv.geckos[dev_name].wwsConnected = 1
                except Exception as e:
                    print(e)
                    print("WWS Connection Failed")
    #############################################
    # Print an error if anything goes wrong above                
    except:
        if logger_id == 0:
            pass
        else:
            traceback.print_exc(file=sys.stdout)
            print("Error in Processing Data")

def save_data(data):
    # Append new data to Datafile
    try:
        gv.datafile.writelines([str(data),'\n'])
    except Exception as e:
        print(e)
        print("Error Writing Data to File")    


def onboard_gecko(dev_address):     
    # Connect to Gecko
    print("Attempting to Connect to ", dev_address)
    gecko_device = gv.bus.get_object(BLUEZ_SERVICE_NAME, dev_address)
    manager = dbus.Interface(gecko_device, 'org.bluez.Device1')
    props = dbus.Interface(gv.bus.get_object(BLUEZ_SERVICE_NAME, dev_address), 'org.freedesktop.DBus.Properties')
    dev_name = dev_address[dev_address.find('dev_')+4:]
    
    if props.Get('org.bluez.Device1','Connected') == False:
        # Try to connect to Gecko
        try:
            manager.Connect()
            print("Gecko Connected")
            time.sleep(1)
            print("Bluetooth Connected, setting up UART...")
        
        except Exception as e:
            print(e)
            if props.Get('org.bluez.Device1','Connected') == False:
                print("BLE connection could not be established, Gecko Removed")
                
                # Technically this shouldn't ever work, but hey, why not try?
                try:
                    manager.Disconnect()
                except:
                    print("No really, I'm not connected to ",dev_name)
                # Remove Gecko Device from BLE and Geckos List
                gv.adapter.RemoveDevice(gecko_device)
                print("Gecko Removed from BLE List")
                gv.geckos[dev_name].discovered = 0
                gv.geckos[dev_name].bleConnected = 0
                print("Success")
                
                return
            
        # Gecko is Onboarded, now setup up UART       
        connect_RX_UART(dev_address)
        connect_TX_UART(dev_address)
        
        # Synchronize Timebase
        try:
            setTimebase(dev_address)
        except:
            print("Could not set timebase")
        # Turn off LED
        try:
            if gv.geckos[dev_name].LED == 0:
                LEDoff(dev_address)
        except:
            pass

def connect_RX_UART(dev_address):
    # For services /service000b is the UART service
    ##### /service000b/char000c is the Write Channel
    ##### /service000b/char000e is the Read Channel
    dev_name = dev_address[dev_address.find('dev_')+4:]
    loop_counter = 0
    
    # Try several times to connect to Rx UART
    while True:
        time.sleep(1)
        if loop_counter > 3:  # Attempt to connect 3 times (3seconds)
            print("Device Not Found")
            return 0
        try:
            # Setup UART Rx object and check for Properties
            uart_Rxobj = gv.bus.get_object(BLUEZ_SERVICE_NAME, dev_address+'/service000b/char000e')
            uart_Rxprop = dbus.Interface(uart_Rxobj, 'org.freedesktop.DBus.Properties')
            props = uart_Rxprop.GetAll('org.bluez.GattCharacteristic1') #this throws an error if BLE not connected, keep it!

            if gv.geckos[dev_name].callback == None:
                geckoCallback = uart_Rxprop.connect_to_signal("PropertiesChanged",new_data_Rx, path_keyword='path')
                gv.geckos[dev_name].callback = geckoCallback
                print("Created Data Callback")
            else:
                print("Gecko Re-Onboarded")
                
            # Check if notifications are active for this Gecko, if not, start them
            if props['Notifying'] == 0:
                gecko_Rx = dbus.Interface(uart_Rxobj, 'org.bluez.GattCharacteristic1')
                gecko_Rx.StartNotify()
                print("Gecko Now Notifying")
                
            return 1                


        except Exception as e:
            print(e)
            pass
        
        print("UART not connected, trying again")
        loop_counter += 1
        
            
def connect_TX_UART(dev_address):
    # For services /service000b is the UART service
    ##### /service000b/char000c is the Write Channel
    ##### /service000b/char000e is the Read Channel
    dev_name = dev_address[dev_address.find('dev_')+4:]
    loop_counter = 0
    
    # Try several times to connect to Rx UART
    while True:
        time.sleep(1)
        if loop_counter > 3:  # Attempt to connect 3 times (3seconds)
            print("Device Not Found")
            return 0
        try:
            # Setup UART Rx object and check for Properties
            uart_obj = gv.bus.get_object(BLUEZ_SERVICE_NAME, dev_address+'/service000b/char000c')
            uart_prop = dbus.Interface(uart_obj, 'org.freedesktop.DBus.Properties')
            props = uart_prop.GetAll('org.bluez.GattCharacteristic1') #this throws an error if BLE not connected, keep it!
            gv.geckos[dev_name].Tx = dbus.Interface(uart_obj, 'org.bluez.GattCharacteristic1')
            print("UART TX Configured")
            
            return 1
        
        except Exception as e:
            print(e)
            pass
        
        print("UART Tx not connected, trying again")
        loop_counter += 1

def main_sequence():
    while gv.mainloop.is_running():
        time.sleep(5)
        # Check for New Devices
        try:
            report_status()
            time.sleep(1)
        except:
            print("Could not report Status")
            
        # Restart Discovery
        try:
            gv.adapter.StartDiscovery()
            time.sleep(3)
        except:
            print("Could not enter Discovery Mode")
        try:
            # Pause Discovery and Onboard
            gv.adapter.StopDiscovery()
        except:
            print("Could not stop discovery")

def gateway_heartbeat():
    # Prep Gecko Data
    myGeckos = {}
    seenGeckos = []
    for dev_name in gv.geckos:
        seenGeckos.append(gv.geckos[dev_name].MAC)
        # Add it and status if this gateway owns it
        if gv.geckos[dev_name].myGecko == 1:
            myGeckos.update({
                            gv.geckos[dev_name].MAC: {
                            'discovered': gv.geckos[dev_name].discovered,
                            'wws_status': gv.geckos[dev_name].wwsConnected,
                            'ble_status': gv.geckos[dev_name].bleConnected}
                             })
                            
                            
    # Send MEssage to WWS Saying Your Alive
    routing_key='sensors.gateway.'+ gv.gatewayID
    data = {'from': gv.gatewayID ,
            'status': 'Alive',
            'time':datetime.datetime.now().timestamp(),
            'seenGeckos':seenGeckos,
            'myGeckos': myGeckos}
    gv.wws.q.put((data,routing_key))
    
#    # Continue Heartbeat
#    heartbeatTimer = threading.Timer(10,gateway_heartbeat)
#    heartbeatTimer.start()
                    
def report_status():
    if gv.mainloop.is_running():
        current_time = datetime.datetime.now()
        print("Gateway Running, ",str(current_time))
        gateway_heartbeat()
        try:
            print("WWS Tx Working = ", gv.wws.Tx_channel.is_open)
        except:
            print("Not able to check WWS Tx Status")
        try:
            print("WWS Rx Working = ", gv.wws.Rx_channel.is_open)
        except:
            print("Not able to check WWS Rx Status")
        ii = 0
        
        # Go through and check status of all geckos and print out
        for dev_name in gv.geckos:
            # Skip Geckos you shouldn't be connecting to
            if gv.geckos[dev_name].myGecko == 0 or gv.geckos[dev_name].discovered == 0:
                continue
            
            dev_address = gv.geckos[dev_name].dev_address
            manager = dbus.Interface(gv.bus.get_object(BLUEZ_SERVICE_NAME, dev_address), 'org.freedesktop.DBus.Properties')
            # Check if it's connected
            try:
                gv.geckos[dev_name].bleConnected = manager.Get('org.bluez.Device1','Connected')
#                print(manager.GetAll('org.bluez.Device1')) # This gets all device properties
            except:
                gv.geckos[dev_name].bleConnected = False
                
            
            if gv.geckos[dev_name].bleConnected == 1:
                # If connected get RSSI and send to WWS
                try:
                    RSSI = manager.Get('org.bluez.Device1','RSSI')
                    
                    # Try to send to WWS
                    if gv.use_WWS:
                        try:
                            routing_key='sensors.gecko.'+dev_name+'.RSSI'
                            
                            # Put the Data on the Queue
                            gv.wws.q.put((RSSI,routing_key))
                        except Exception as e:
                            print(e)
                            print("Couldn't Send RSSI to WWS")
                except:
                    pass

                        
            # Try to reconnect if Gecko is not connected
            try:
                if manager.Get('org.bluez.Device1','Connected') == False:
                    # Try to onboard gecko
                    onboard_gecko(dev_address)
                    
                    if gv.geckos[dev_name].discovered == 0:
                        continue

                    try:
                        gv.geckos[dev_name].bleConnected = manager.Get('org.bluez.Device1','Connected')
                    except:
                        gv.geckos[dev_name].bleConnected = 0
            except Exception as e:
                    print("Onboarding failed")
                    print(e)
                    
            dev_name = dev_address[dev_address.find('dev_')+4:]
            
            # Check for WWS Notification Error
            if gv.use_WWS and not gv.geckos[dev_name].wwsConnected:
                try:
                    # Get Gecko Properties if it should be connected to WWS but isn't
                    uart_obj = gv.bus.get_object(BLUEZ_SERVICE_NAME, dev_address+'/service000b/char000e')
                    uart_prop = dbus.Interface(uart_obj, 'org.freedesktop.DBus.Properties')
                    props = uart_prop.GetAll('org.bluez.GattCharacteristic1') #this throws an error if BLE not connected, keep it!
                except Exception as e:
                    print(e)
                else:
                    # Only create data callback if it doesn't already exist
                    print("WWS Failure Detected, 'Notifying' Status: ",props['Notifying'])
                    
            # Print Out Status and Reset WWS/File indicators
            try:
                # Print to screen
                print(dev_name,"; Connected to BLE: ",gv.geckos[dev_name].bleConnected,"; Connected to WWS: ",gv.geckos[dev_name].wwsConnected,"; Saving to File: ",gv.geckos[dev_name].saveLocal)
                try:
                    print("RSSI = ", RSSI)
                except:
                    pass
                ii += 1
                # Reset status indicators
                gv.geckos[dev_name].wwsConnected = 0
                gv.geckos[dev_name].saveLocal = 0
            except:
                print("Couldn't reset Status Indicators")
        print("Currently ",ii," Geckos Connected")

if __name__ == '__main__':
    # Set up the main loop.
    DBusGMainLoop(set_as_default=True)

    # Initialize Global Variables
    gv.variables_init()
    
    # Initialize BLE Adapter
    gv.adapter = bluezutils.find_adapter(gv.ADAPTER_NAME)
    
    # Open file to save data
    gv.datafile = open("Datafile.txt","a")
    
    # Clear BLE List
    delete_gecko_devs()

    # Import Old Device List (you have to do this or delete_gecko_devs()
#    import_old_geckos()
    
    # Check for Hard Coded Geckos
    for MAC in gv.hardCodedGeckos:
        dev_address = gv.GECKO_ADDRESS+MAC
        gv.geckos[MAC] = gv.gecko_info(MAC,dev_address,discovered=0,myGecko=1)
    
    # Start Discovery
    discover_devices()
    time.sleep(1)
        
    # Pause Discovery and Onboard
    gv.adapter.StopDiscovery()
    time.sleep(1)
    
    ############### Start Threads #####################
    myThread = threading.Thread(target=gv.mainloop.run)
    myThread.daemon = True
    myThread.start()
    
    # Open WWS Channel
    gv.wws = wws_controller.wws()
    
    # Start Discovery and Onboarding
    mainThread = threading.Thread(target=main_sequence)
    mainThread.start()
    
#    # Start Heartbeat
#    heartbeatTimer = threading.Timer(10,gateway_heartbeat)
#    heartbeatTimer.start()
    
    print("\n\n****** Press 'q' and then ENTER to Quit ******")
    
    while myThread.isAlive():
        a = sys.stdin.read(1)
        if a[0] == 'q':
            gv.mainloop.quit()
            mainThread.join()
            myThread.join()
            try:
                gv.wws.Rx_channel.stop_consuming()
            except:
                pass
            print("Exiting Program\n")
        else:
            print("Type 'q' then ENTER to quit")

    
    ### Disconect BLE ###
    try:
        for dev_name in gv.geckos:
            gecko_device = gv.bus.get_object(BLUEZ_SERVICE_NAME, gv.geckos[dev_name].dev_address)
            manager = dbus.Interface(gecko_device, 'org.bluez.Device1')
            manager.Disconnect()
            print([gv.geckos[dev_name].dev_address,"Disconnected"])
    except:
        traceback.print_exc(file=sys.stdout)
        print("Some Geckos could not be disconnected")
        
    print("Goodbye!")
    # Close Datafile
    if gv.save_local == 1:
        gv.datafile.close()

