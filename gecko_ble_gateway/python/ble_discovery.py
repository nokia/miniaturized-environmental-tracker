#!/usr/bin/python
# 	Colyright 2018-2022 Nokia
#
# 	All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# 	Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).


from __future__ import absolute_import, print_function, unicode_literals

from optparse import OptionParser, make_option
import dbus
import dbus.mainloop.glib
try:
  from gi.repository import GObject
except ImportError:
  import gobject as GObject
import bluezutils
import time
import threading
import sys
import subprocess
import config as gv
from gecko_RxGateway import *
NORDIC_RX_UUID =        '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
BLUEZ_SERVICE_NAME = 'org.bluez'
compact = False

######### This will delete the existing BL device List ########
def delete_gecko_devs():
    # Get List of All Devices on Current Adapter
    print("***Finding Old Geckos***")
    old_devs = bluezutils.find_device_addresses(gv.keyword, adapter_pattern=gv.ADAPTER_NAME)
    
    #Remove Devices
    for name in old_devs:
        dev_address = name.replace(':','_')
        print(dev_address)
        gecko_device = gv.bus.get_object(BLUEZ_SERVICE_NAME, dev_address)
        gv.adapter.RemoveDevice(gecko_device)
        print("Removed ",dev_address)

#############################################################
        
######### This will Import Previously Found Geckos ########
def import_old_geckos():
    old_devs = bluezutils.find_device_addresses(gv.keyword, adapter_pattern=gv.ADAPTER_NAME)
    
    print("***Finding Old Geckos***")
    for name in old_devs:
        dev_address = name.replace(':','_')
        MAC = dev_address[dev_address.find('dev_')+4:]
        # Add the Gecko to the Gecko List if it isn't already there
        if MAC not in gv.geckos:
            gv.geckos[MAC] = gv.gecko_info(MAC,dev_address,discovered=1)
            print(gv.geckos[MAC].dev_address)

#############################################################

##################### Discovery Functions ########################
def print_normal(address, properties):
    for key in properties.keys():
        value = properties[key]
        if type(value) is dbus.String:
            value = str(value).encode('ascii', 'replace')

    properties["Logged"] = True

def skip_dev(old_dev, new_dev):
    if not "Logged" in old_dev:
        return False
    if "Name" in old_dev:
        return True
    if not "Name" in new_dev:
        return True
    return False

def interfaces_added(path, interfaces):
    try:
        properties = interfaces["org.bluez.Device1"]
    except:
        properties = []
    
    if not properties:
        return
    
    if path in properties:
        dev = properties[path]

        if compact and skip_dev(dev, properties):
            return
        properties[path].update(properties.items())
    else:
        properties[path] = properties

    if "Address" in properties[path]:
        address = properties["Address"]
        
        if "Name" in properties[path]:
            print_normal(address, properties[path])
            try:
                # Check for devices taht start with *keyword*
                if properties[path]["Name"].startswith(gv.keyword):
                    MAC = str(properties[path]["Address"]).replace(':','_')
                    dev_address = gv.GECKO_ADDRESS+MAC
                    
                    # Add the Gecko to the Gecko List if it isn't already there
                    if MAC not in gv.geckos:
                        gv.geckos[MAC] = gv.gecko_info(MAC,dev_address,discovered=1,myGecko=gv.connectAll)
                        print("New Gecko Found! ", gv.geckos[MAC].dev_address)
                        # Onboard the Gecko if it belongs to this gateway
                    else:
                        # This has not been tested...
                        gv.geckos[MAC].discovered = 1
                        print("Found Known Gecko, ", MAC)        
                        
            except Exception as e:
                print(e)
    else:
        address = "<unknown>"


def discover_devices():
    option_list = [
        make_option("-i", "--device", action="store",
                type="string", dest="dev_id"),
        make_option("-u", "--uuids", action="store",
                type="string", dest="uuids",
                help="Filtered service UUIDs [uuid1,uuid2,...]"),
        make_option("-r", "--rssi", action="store",
                type="int", dest="rssi",
                help="RSSI threshold value"),
        make_option("-p", "--pathloss", action="store",
                type="int", dest="pathloss",
                help="Pathloss threshold value"),
        make_option("-t", "--transport", action="store",
                type="string", dest="transport",
                help="Type of scan to run (le/bredr/auto)"),
        make_option("-c", "--compact",
                action="store_true", dest="compact"),
        ]
    parser = OptionParser(option_list=option_list)

    (options, args) = parser.parse_args()

    options.dev_id = gv.ADAPTER_NAME
    gv.adapter = bluezutils.find_adapter(options.dev_id)
    print("\n\n### Using The Following Adapter ###")
    print("Adapter ID:" ,options.dev_id)
    print("GV Adapter:" , gv.adapter)

    if options.compact:
        compact = True;

    gv.bus.add_signal_receiver(interfaces_added,
            dbus_interface = "org.freedesktop.DBus.ObjectManager",
            signal_name = "InterfacesAdded")

    om = dbus.Interface(gv.bus.get_object("org.bluez", "/"),
                "org.freedesktop.DBus.ObjectManager")

    scan_filter = dict()

    if options.uuids:
        uuids = [NORDIC_RX_UUID]
        uuid_list = options.uuids.split(',')
        for uuid in uuid_list:
            uuids.append(uuid)

        scan_filter.update({ "UUIDs": uuids })

    if options.rssi:
        scan_filter.update({ "RSSI": dbus.Int16(options.rssi) })

    if options.pathloss:
        scan_filter.update({ "Pathloss": dbus.UInt16(options.pathloss) })

    if options.transport:
        scan_filter.update({ "Transport": options.transport })

    gv.adapter.SetDiscoveryFilter(scan_filter)
    gv.adapter.StartDiscovery()
    
    
############################################################
        
if __name__ == '__main__':
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

    # Initialize Global Variables
    gv.variables_init()
    
    #### Start BL Discovery
    delete_gecko_devs()
#    import_old_geckos()
    print("\n*** These devices were found ***")
    discover_devices()
    
    #### First Scan for Devices ####
    myThread = threading.Thread(target=gv.mainloop.run)
    myThread.daemon = True
    myThread.start()
    
    threadTimer = threading.Timer(2,gv.mainloop.quit)
    threadTimer.start()
    myThread.join()
    ################################
    
    #### List devices found ####
    print("Gecko List")
    
    for address in gv.geckos:
        print(address)
            
    #############################  
        

    

