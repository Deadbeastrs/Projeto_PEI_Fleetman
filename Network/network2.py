import argparse
import json
import math
import sys
import time
from threading import Thread
import re
#import matplotlib.pyplot as plt
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

def calc_network(table,relay,interface):
    #Get all the connected Nodes------------------------------------------
    try:
        res = sp.check_output('iw dev ' + interface + ' station dump | grep -E \'Station|tx bytes:|tx packets:|signal:|tx bitrate:|rx bitrate:\'',shell = True)
        res = res.decode("utf-8")
        ips = []
        latencys = []
        var = "([0-9a-z]{2}:[0-9a-z]{2}:[0-9a-z]{2}:[0-9a-z]{2}:[0-9a-z]{2}:[0-9a-z]{2})"
        mac = re.findall(var,res)
    except:
        return [{}]

    TxByte = re.findall("(?<=tx bytes:\\s)\\d+",res)
    #TxPackets = re.findall("(?<=tx packets:\\s)\\d+",res)
    Signal = re.findall("(?<=signal:\\s{2}\\t)(-?\\d+)",res)
    #RxBit = re.findall("(?<=rx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)
    TxBit = re.findall("(?<=tx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)
    
    #Create the array with ips in order with the mac array----------------
    check = False
    for m in mac:
        for dev in table:
            if m == dev.get('Device').get('mac'):
                ips.append(dev.get('Device').get('ip'))
                check = True
        if check == False:
            ips.append("Cant resolve")
        check = False

    RxBit = re.findall("(?<=rx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)
    
    #Get the latency between the Nodes-----------------------------------
    for ipaddr in ips:
        try:
            latency = sp.check_output('ping -c 1 -w 1 ' + ipaddr ,shell = True)
            latency.decode("utf-8")
            latency = re.findall("(?<=time=)\d*.?\d*",str(latency))
            latencys.append(latency[0])
        except:
            latencys.append("error")

    res = sp.check_output('iw dev ' + interface + ' station dump | grep -E \'Station|tx bytes:|tx packets:|signal:|tx bitrate:|rx bitrate:\'',shell = True)
    res = res.decode("utf-8")
    RxBit = re.findall("(?<=rx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)

    if len(RxBit) != len(Signal):
        return [{}]

    Stations = []


    #Calculate estimate network quality and return-------------------
    NetworkQuality = ""
    for i in range(len(mac)):
        if abs(int(Signal[i])) < 70 and float(RxBit[i].split(" ")[0]) >= 2.0:
            NetworkQuality = "HIGH"
        elif abs(int(Signal[i])) < 80 and float(RxBit[i].split(" ")[0]) > 1.0:
            NetworkQuality = "MEDIUM"
        elif abs(int(Signal[i])) > 80 or latencys[i] == "error":
            NetworkQuality = "LOW"
        
        Stations.append( {
            "Station": mac[i],
            "IP": ips[i],
            "Latency": latencys[i],
            #"TxPackets" : TxPackets[i],
            "Signal": str(abs(int(Signal[i]))),
            "TxByte": TxByte[i],
            "RxBit": RxBit[i],
            "TxBit" : TxBit[i],
            "NetworkQuality": NetworkQuality
            })
    if relay[0] == "All":
        return Stations
    else:
        for station in Stations:
            if station.get("IP") == relay[1]:
                station["Relay"] = relay[0]
                return [station]
        return [{}]

class NetworkSensor(Node):

    def __init__(self):

        self.rate = 1
        self.sensor_topic = '/sensor/network'
        self.telem_topic = '/telem'
        self.info_topic = '/info'
        self.coords = None
        #self.startingCoords = None
        self.times = 0
        #self.logger = open("logger.txt","w")

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

        if cfg.get('Table') is None:
            print('[ERROR] Provided config file does not contain the network interface of other drones.')
            sys.exit()

        if cfg.get('Table') is None:
            print('[ERROR] Provided config file does not contain the ip addresses of other drones.')
            sys.exit()
        if cfg.get('groundStation_IP') is None:
            print('[ERROR] Provided config file does not contain the ip addresses of GroundStation.')
            sys.exit()

        super().__init__('network', namespace=self.drone_id + '/sensor')
        self.relay = ("GroundStation",cfg.get('groundStation_IP'))
        self.devices = cfg.get('Table')
        self.interface = cfg.get('interface')
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
        value = self.get_network()
        
        msg.data = json.dumps(
            {'droneId': self.drone_id, 'sensorId': 'real','type': 'network',
             'timestamp': current_time_millis(), 'value': value})
	#self.logger.write(msg.data)
        if not self.silence:
            self.get_logger().info(msg.data)
        self.publisher.publish(msg)

    def drone_telem_callback(self, msg):
        telem = json.loads(msg)
        if telem.get('droneId') == self.drone_id and telem.get('position').get('lat') is not None:
            if (self.times == 0):
                #self.startingCoords = (telem.get('position').get('lat'), telem.get('position').get('lon'))
                self.times = self.times + 1
            self.coords = (telem.get('position').get('lat'), telem.get('position').get('lon'))

    def drone_info_callback(self, msg): 
           info = json.loads(msg)
           if info.get('droneId') == self.drone_id and info.get('target') is not None and info.get('targetIP') is not None:
               self.relay = (info.get('target'), info.get('targetIP'))


    def get_network(self):
        network = calc_network(self.devices,self.relay,self.interface)
        return network


def main(args=None):
    rclpy.init(args=args)
    net_sensor = NetworkSensor()
    rclpy.spin(net_sensor)

    if net_sensor.coords is not None:
        net_sensor.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()

    
