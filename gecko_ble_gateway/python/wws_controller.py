# 	Colyright 2018-2022 Nokia
#
# 	All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# 	Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).

# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 14:57:34 2019

@author: megglest
"""

import numpy as np
import time
import pika
import json
import datetime
import random
import config as gv
import queue
import time
import threading

##############################################################################
class wws(object):
    def __init__(self):
        if gv.gatewayID == None:
            self.wwsID = str(random.randint(0,2**32))
        else:
            self.wwsID = gv.gatewayID
        self.q = queue.Queue()
        self.spawn_wwsTx()
        self.spawn_wwsRx()
        
    # Shutdown WWS Rx/Tx
    def WWS_shutdown(self):
        self.wwsRx_Thread.stop()
        self.wwsTx_Thread.stop()

    #%% WWS Callback on Rx
    def WWS_Rx_callback(self, ch, method, properties, body):
        print(" [x] Received %s %r" % (method.routing_key, body))
        
        ##### Try to unpack message #####
        try:
            parsed_json = json.loads(body)
            if "segmentation_requested" in parsed_json :
                segments = parsed_json['segmentation_requested']
            if "file" in parsed_json :
                file = parsed_json["file"][5:]
                file = file
        except Exception as e:
            print("Error in Recieving JSON")
            print(e)
                
    
    def spawn_wwsRx(self):
        # Call wwsRx initialization Function (this starts the listener)
        self.wwsRx_Thread = threading.Thread(target=self.open_WWS_Rx) 
        self.wwsRx_Thread.daemon = True
        self.wwsRx_Thread.start()
        
    def spawn_wwsTx(self):
        # Call wwsRx initialization Function (this starts the listener)
        self.wwsTx_Thread = threading.Thread(target=self.WWS_Message_Sender) 
        self.wwsTx_Thread.daemon = True
        self.wwsTx_Thread.start()
        
    def check_wws(self):
        # Check if Rx is Still Open
        try:
            self.wwsRx_Thread
        except:
            try:
                self.spawn_wwsRx()
            except:
                print("Could not Connect to WWS Rx!")
                print(sys.exc_info()[0])
        else:
            if self.wwsRx_Thread.is_alive():
                self.wwsRxReady = True
            else:
                try:
                    self.spawn_wwsRx()
                except:
                    print("Could not Connect to WWS Rx!")
                    print(sys.exc_info()[0])

    def WWS_Message_Sender(self):
        # Open the Tx Channel
        try:
            self.open_WWS_Tx()
        except Except as e:
            print("Could Not Open WWS TX Connection. Message not Sent")
            print(e)
                    
        while gv.mainloop.is_running():
            try:
                if not self.q.empty():
                    msg,route = self.q.get()
                    #Just send one message
                    self.Tx_channel.basic_publish(exchange='ingress_exchange',routing_key=route, body=json.dumps(msg))
                else:
                    time.sleep(.004)
            except Except as e:
                print("Could Not Send Message")
                print(e)
                # Check if WWS is Down
                if self.Tx_channel.is_closed():
                    try:
                        self.open_WWS_Tx()
                    except Except as e:
                        print("Could Not Open WWS TX Connection. Message not Sent")
                        print(e)
                        time.sleep(.1)
        
    #%% WWS Rx Init
    def open_WWS_Rx(self):
        connection = pika.BlockingConnection(pika.URLParameters('amqp://stream_bridge_user1:WWS2016@'+gv.wwsIP+':5672/%2Ftest'))
        
        self.Rx_channel = connection.channel()
        
        #######
        myQueue = 'GeckoGateway_'+self.wwsID+'Rx'
        
        try:
            self.Rx_channel.queue_delete(myQueue)
            self.Rx_channel.queue_unbind(exchange='egress_exchange', queue=myQueue)
            print("Queue Unbound")
        except Exception as e:
            print(e)
            pass
        
        self.Rx_channel.queue_declare(queue=myQueue,exclusive=False,durable=False)
        
        self.Rx_channel.queue_bind(exchange='egress_exchange',
                           queue=myQueue,
                           routing_key='sensors.gecko.gateway.#')
        
        self.Rx_channel.basic_consume(myQueue, self.WWS_Rx_callback, auto_ack=True)
         
        print(' [*] Waiting for WWS messages')
        try:
            self.Rx_channel.start_consuming()
        except:
            print('WWS Rx Offline\n')
        
    
    #%% WWS Tx Init
    def open_WWS_Tx(self):
        
        try:
            credentials = pika.PlainCredentials('stream_bridge_user1', 'WWS2016')
            
            parameters = pika.ConnectionParameters(gv.wwsIP,
                                                   5672,
                                                   '/test',
                                                   credentials,
                                                   heartbeat=600,
                                                   blocked_connection_timeout=300);
                                                   
            connection = pika.BlockingConnection(parameters)
            self.Tx_channel = connection.channel()
            myQueue = 'GeckoGateway_'+self.wwsID+'Tx'
            self.Tx_channel.queue_declare(queue=myQueue)
            print("Connected to WWS!!")
            self.wws_connected = True
        except:
            print("Could not connect to WWS")
            self.wws_connected = False
            
            
            
if __name__ == '__main__':
    trx = wws()
    