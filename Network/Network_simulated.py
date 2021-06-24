import argparse
import json
import math
import sys
import time
from haversine import haversine, Unit
from threading import Thread
import re
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
import subprocess as sp

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', required=True)
    parser.add_argument('-e', '--export', dest='export', action='store_true')
    parser.add_argument('--no-export', dest='export', action='store_false')
    parser.add_argument('-s', '--silence', dest='silence', action='store_true')
    parser.add_argument('--no-silence', dest='silence', action='store_false')
    parser.set_defaults(export=False)
    parser.set_defaults(export=False)

    cli_args = parser.parse_args()
    return yaml.load(open(cli_args.config)), cli_args.export, cli_args.silence



def current_time_millis():
    return int(round(time.time() * 1000))

def calc_network(distance,droneID,radious,GlobalTx):
    #Values derived from graph analysis on the real Network sensor
    if distance < radious:
        if distance == 0:
            sig = 0
            Latency = 1
            RxBit = 50
            TxBit = 50
        else:
            if distance < 20:
                sig = distance + 30 
            else:
                sig = (29 * math.log(distance-20,10))+30
            if sig < 0:
                sig = 0
            if sig > 90:
                sig = 90
            RxBit =(-50/183)*distance+50
            if distance < 20:
                Latency = 1
                TxBit = (-50/183)*distance+50
            elif distance < 40:
                Latency = 5
                TxBit = 25
            elif distance < 60:
                Latency = 10
                TxBit = 25
            elif distance < 100:
                Latency = 50
                TxBit = (-50/183)*distance+50
            else:
                Latency = "error"
        if sig < 70 and RxBit >= 2.0:
            NetworkQuality = "HIGH"
        elif sig < 80 and RxBit > 1.0:
            NetworkQuality = "MEDIUM"
        elif sig > 80 or RxBit == "error":
            NetworkQuality = "LOW"
        NetStats = {
        "Station": "00:00:00:00:00:00",
        "IP": "0.0.0.0",
        "Latency": str(Latency),
        "Signal": str(sig),
        "TxByte": str(GlobalTx),
        "RxBit": str(RxBit) + " MBit/s",
        "TxBit" : str(TxBit)+ " MBit/s",
        "NetworkQuality": NetworkQuality,
        "Relay": droneID
        }

        return NetStats
    else:
        return {}

class NetworkSensor(Node):

    def __init__(self):

        self.rate = 1
        self.sensor_topic = '/sensor/network'
        self.telem_topic = '/telem'
        self.info_topic = '/info'
        self.coords = None
        self.relayCoords = None
        self.times = 0
        self.otherDrones = {}
        self.GlobalTx = 0

        cfg, export, self.silence = parse_args()

        if cfg is None:
            print('[ERROR] Provided config file is empty.')
            sys.exit()
        if cfg.get('droneId') is None:
            print('[ERROR] Provided config file does not contain drone ID.')
            sys.exit()

        self.drone_id = cfg.get('droneId')
        
        if cfg.get('sensorTopic') is not None:
            self.sensor_topic = cfg.get('sensorTopic')
        
        if cfg.get('telemTopic') is not None:
            self.telem_topic = cfg.get('telemTopic')
        if cfg.get('rate') is not None:
            self.rate = cfg.get('rate') / 1000

        if cfg.get('Radious') is not None:
            self.Radious = cfg.get('Radious')

        self.rate = 0.1
        super().__init__('network', namespace=self.drone_id + '/sensor')
        #self.relay = [('groundStation','0.0.0.0')]
        self.relay = {"groundStation" : [("groundStation","0.0.0.0")], "Relay" :[("groundStation","0.0.0.0")] }

        self.subscription = \
            self.create_subscription(String, self.telem_topic, lambda msg: self.drone_telem_callback(msg.data), 10)
        
        self.subscription2 = \
            self.create_subscription(String, self.info_topic, lambda msg: self.drone_info_callback(msg.data), 10)
        

        self.publisher = None
        self.timer = None

        thread = Thread(target=self.init_pub)
        thread.start()

    def init_pub(self):
        self.get_logger().info('Waiting for drone "%s" telem data...' % self.drone_id)
        i = 0
        while self.coords is None:
            if i > 20:
                self.get_logger().info(
                    'Timed out waiting for drone "%s" data. Is fleetman drone running? Exiting.' % self.drone_id)
                rclpy.shutdown()
                sys.exit()
            time.sleep(1)
            i += 1

        self.get_logger().info('Start network sensor for drone "%s"' % self.drone_id)
        self.publisher = self.create_publisher(String, self.sensor_topic, 10)
        self.timer = self.create_timer(self.rate, self.pub_network_callback)


    
    def pub_network_callback(self):
        msg = String()
        msg.data = json.dumps(
            {'droneId': self.drone_id, 'sensorId': 'simulated','type': 'network',
             'timestamp': current_time_millis(), 'value': self.get_network()})
        if not self.silence:
            self.get_logger().info(msg.data)
        self.publisher.publish(msg)


    def drone_telem_callback(self, msg):
        telem = json.loads(msg)
        if telem.get('droneId') == self.drone_id and telem.get('position').get('lat') is not None:
            if (self.times == 0):
                self.otherDrones['groundStation'] = (telem.get('position').get('lat'), telem.get('position').get('lon'))
                self.times = self.times + 1
            self.coords = (telem.get('position').get('lat'), telem.get('position').get('lon'))
        elif telem.get('droneId') != self.drone_id and telem.get('position').get('lat') is not None:
            self.otherDrones[telem.get('droneId')] = (telem.get('position').get('lat'), telem.get('position').get('lon'))

    def drone_info_callback(self, msg): 
           info = json.loads(msg)
           if info.get('droneId') == self.drone_id and info.get('connectionsId') is not None and info.get('connectionsIp') is not None:
               tempArray = []
               droneIDs = info.get('connectionsId')
               droneIPs = info.get('connectionsIp')
               numberOfConnections = len(droneIDs)
               if(len(droneIDs) != len(droneIPs)):
                   tempArray = []
               else:
                   for i in range(numberOfConnections):
                       tempArray.append((droneIDs[i],droneIPs[i]))
               self.relay['groundStation'] = tempArray
           elif info.get('droneId') == self.drone_id and info.get('RelayId') is not None and info.get('RelayIp') is not None:
               tempArray = []
               droneIDs = info.get('RelayId')
               droneIPs = info.get('RelayIp')
               numberOfConnections = len(droneIDs)
               if(len(droneIDs) != len(droneIPs)):
                   tempArray = []
               else:
                   for i in range(numberOfConnections):
                       tempArray.append((droneIDs[i],droneIPs[i]))
               self.relay['Relay'] = tempArray

    def get_network(self):
        self.GlobalTx += 1
        returnValue = []
        for request in self.relay.keys():
            returnValueTemp = []
            
            for connection in self.relay[request]:
                network = calc_network(self.distance(self.otherDrones[connection[0]]),connection[0],self.Radious,self.GlobalTx)
                returnValueTemp.append(network)
            
            returnValue.append(returnValueTemp)

        RelayValues = returnValue[1]
        mapper = {"HIGH" : 3 , "MEDIUM" : 2, "LOW" : 1}
        quality = "HIGH"
        for relay in RelayValues:
            quality2 = relay.get('NetworkQuality')
            if quality2 is not None:
                if mapper[quality2] < mapper[quality]:
                    quality = quality2
            else:
                quality = "Disconnected"

        self.changeTelemRate(quality)

        dictionaryReturn = {}
        dictionaryReturn['groundStation'] = returnValue[0]
        dictionaryReturn['Relay'] = quality
        return [dictionaryReturn]

    def distance(self,other_coords):
        distance = haversine(self.coords, other_coords, unit=Unit.METERS)
        return distance


    def changeTelemRate(self,quality):
        if quality == "HIGH" and self.rate != 0.2:
            sp.run("ros2 param set /" + self.drone_id + " telemetryRateMs 200",shell = True)
            self.destroy_timer(self.timer)
            self.rate = 0.2
            self.timer = self.create_timer(self.rate, self.pub_network_callback)
        elif quality == "MEDIUM" and self.rate != 0.4:
            sp.run("ros2 param set /" + self.drone_id + " telemetryRateMs 400",shell = True)
            self.destroy_timer(self.timer)
            self.rate = 0.4
            self.timer = self.create_timer(self.rate, self.pub_network_callback)
        elif quality == "LOW" and self.rate != 0.6:
            sp.run("ros2 param set /" + self.drone_id + " telemetryRateMs 600",shell = True)
            self.destroy_timer(self.timer)
            self.rate = 0.6
            self.timer = self.create_timer(self.rate, self.pub_network_callback)

def main(args=None):
    rclpy.init(args=args)
    net_sensor = NetworkSensor()
    rclpy.spin(net_sensor)

    if net_sensor.coords is not None:
        net_sensor.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
