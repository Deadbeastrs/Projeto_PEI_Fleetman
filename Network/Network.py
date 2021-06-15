import argparse
import json
import math
import sys
import time
from threading import Thread
import re
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

def calc_network(id,ip,mac,interface,times):
    lista = []

    for i in range(times):
        #Mesure Rxbit,Txbit and Signal and get the avarage value to make it more stable
        try:
            res = sp.check_output('iw dev ' + interface + ' station get ' + mac + '| grep -E \'Station|tx bytes:|tx packets:|signal:|tx bitrate:|rx bitrate:|expected throughput:\'',shell = True)
            res = res.decode("utf-8")
            ips = []
            latencys = []
           
        except:
            print("DEVICE NOT CONNECTED")
            return {}
        TxByte = re.findall("(?<=tx bytes:\\s)\\d+",res)
        Signal = re.findall("(?<=signal:\\s{2}\\t)(-?\\d+)",res)
        TxBit = re.findall("(?<=tx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)
        RxBit = re.findall("(?<=rx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)
        #Prevent error when RxBit is Unkwon  
        if len(RxBit) != len(Signal) :
            return {}

        Station = {
        "Station": mac,
        "IP": ip,
        "Latency": 0,
        #"Expected_throughput": Throughput[0],
        "Signal": str(abs(int(Signal[0]))),
        "TxByte": TxByte[0],
        "RxBit": RxBit[0],
        "TxBit" : TxBit[0],
        
        }
        
        
        
        lista.append(Station)

    Station =  {
        "Station": mac,
        "IP": ip,
        "Latency": 0,
        #"Expected_throughput": 0,
        "Signal": 0,
        "TxByte": lista[0].get('TxByte'),
        "RxBit": 0,
        "TxBit" : 0,
        "Relay" : id,
        }
    latencys = []

    #Retrive the latency by perforing pings
    for i in range(2):
        try:
            latency = sp.check_output('ping -c 1 -w 1 ' + ip ,shell = True)
            latency.decode("utf-8")
            latency = re.findall("(?<=time=)\d*.?\d*",str(latency))
            latencys.append(latency[0])
        except:
            latencys.append("250")
    sum = 0
    for i in range(0, len(latency)):    
        sum = sum + float(latency[i]);
    latency = sum / 2

    #Retrive interface current throughput in kb/s (works but its too slow for drone data)
    #try:
    #    res2 = sp.check_output('sar -n DEV 1 1 | grep ' + interface + ' | tail -n1',shell = True)
    #    res2 = res2.decode("utf-8")
    #    res2 = res2.split(" ")
    #    throughput = str(res2[31])
    #except:
    #    throughput = "unkwon"

    #Station['Interface_Throughput'] = throughput + " kB/s"

    #Sum all the Rxbit, TxBit and Signal parameters        
    for Message in lista:
        Station['Signal'] = Station['Signal'] + float(Message.get('Signal'))
        Station['RxBit'] = Station['RxBit'] +float(Message.get('RxBit').split(" ")[0]) 
        Station['TxBit'] = Station['TxBit'] + float(Message.get('TxBit').split(" ")[0])
    


    #Divide the Parameters to get the avarage
    RxBit = round(Station['RxBit']/times,2)
    TxBit = round(Station['TxBit']/times,2)
    Signal = Station['Signal'] /times
    Station['Latency'] = str(round(latency,2)) + " ms"
    Station['Signal'] = str(Signal)
    Station['RxBit'] = str(RxBit) + " MBit/s"
    Station['TxBit'] = str(TxBit) + " MBit/s"

    #res3 = sp.check_output('ifconfig ' + interface,shell = True)
    #res3 = res3.decode("utf-8")
    #Erros = re.findall("TX errors .+",res3)
    #TxErrors = Erros[0].split(" ")[2]
    #TxCollisions = Erros[0].split(" ")[13]
    #Station['TxCollisions'] = TxCollisions
    #Station['TxErrors'] = TxErrors

    NetworkQuality = ""

    #Make a estimate of the network quality based on some parameters
    if (Signal) <= 60 and (latency) <= 16 and (TxBit) >= 5 and RxBit >= 5:
        NetworkQuality = "HIGH"
    elif (Signal) <= 75 and (latency) <= 33 and (TxBit) >= 2 and RxBit >= 2:
        NetworkQuality = "MEDIUM"
    else:
        NetworkQuality = "LOW"
    Station['NetworkQuality'] = NetworkQuality
    return Station


class NetworkSensor(Node):

    def __init__(self):

        self.rate = 1
        self.sensor_topic = '/sensor/network'
        self.telem_topic = '/telem'
        self.info_topic = '/info'
        self.coords = None
        self.times = 0

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

        
        if cfg.get('groundStation_IP') is None:
            print('[ERROR] Provided config file does not contain the ip addresses of GroundStation.')
            sys.exit()

        if cfg.get('groundStation_MAC') is None:
            print('[ERROR] Provided config file does not contain the mac addresses of GroundStation.')
            sys.exit()

        super().__init__('network', namespace=self.drone_id + '/sensor')

        self.relay = {"groundStation": [["groundStation",cfg.get('groundStation_IP'),cfg.get('groundStation_MAC')]], "Relay": [["groundStation",cfg.get('groundStation_IP'),cfg.get('groundStation_MAC')]]}
        
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
        #while self.coords is None:
        #    if i > 20:
        #        self.get_logger().info(
        #            'Timed out waiting for drone "%s" data. Is fleetman drone running? Exiting.' % self.drone_id)
        #        rclpy.shutdown()
        #        sys.exit()
        #    time.sleep(1)
        #    i += 1

        self.get_logger().info('Start network sensor for drone "%s"' % self.drone_id)
        self.publisher = self.create_publisher(String, self.sensor_topic, 10)
        self.timer = self.create_timer(self.rate, self.pub_network_callback)



    def pub_network_callback(self):
        #publish the network quality in the topic
        msg = String()
        value = self.get_network()
        
        msg.data = json.dumps(
            {'droneId': self.drone_id, 'sensorId': 'real','type': 'network',
             'timestamp': current_time_millis(), 'value': value})

        if not self.silence:
            self.get_logger().info(msg.data)
        self.publisher.publish(msg)



    def drone_telem_callback(self, msg):
        #Receive drone telemetry messages
        telem = json.loads(msg)
        if telem.get('droneId') == self.drone_id and telem.get('position').get('lat') is not None:
            if (self.times == 0):
                self.times = self.times + 1
            self.coords = (telem.get('position').get('lat'), telem.get('position').get('lon'))



    def drone_info_callback(self, msg): 
           info = json.loads(msg)

           #Retrive messages asking for a specific connection
           if info.get('droneId') == self.drone_id and info.get('connectionsId') is not None and info.get('connectionsIp') is not None and info.get('connectionsMac') is not None:
               tempArray = []
               dronesIDs = info.get('connectionsId')
               dronesIPs = info.get('connectionsIp')
               dronesMac = info.get('connectionsMac')
               numberOfConnections = len(dronesIDs)
               if(len(dronesIDs) != len(dronesIPs)):
                   tempArray = []
               else:
                   for i in range(numberOfConnections):
                       tempArray.append([dronesIDs[i],dronesIPs[i],dronesMac[i]])
               self.relay['groundStation'] = tempArray

           #Retrive the message that indicates who the relay drones are
           elif info.get('droneId') == self.drone_id and info.get('RelayId') is not None and info.get('RelayIp') is not None and info.get('RelayMac'):
               tempArray = []
               dronesIDs = info.get('RelayId')
               dronesIPs = info.get('RelayIp')
               dronesMac = info.get('RelayMac')
               numberOfConnections = len(dronesIDs)
               if(len(dronesIDs) != len(dronesIPs)):
                   tempArray = []
               else:
                   for i in range(numberOfConnections):
                       tempArray.append([dronesIDs[i],dronesIPs[i],dronesMac[i]])
               self.relay['Relay'] = tempArray

    
    def get_network(self):
        returnValue = {}
        returnMessageGs = []
        returnMessageRelay = ""

        #Retrive the network quality parameters to send to the ground Station
        for i in range(len(self.relay['groundStation'])):
            returnMessageGs.append(calc_network(self.relay['groundStation'][i][0],self.relay['groundStation'][i][1],self.relay['groundStation'][i][2],self.interface,50))
        returnValue['groundStation'] = returnMessageGs


        mapper = {"HIGH" : 3 , "MEDIUM" : 2, "LOW" : 1}
        quality = "HIGH"

        #Retrive the network quality between the relay drones
        for i in range(len(self.relay['Relay'])):
            returnMessageRelay = calc_network(self.relay['Relay'][i][0],self.relay['Relay'][i][1],self.relay['Relay'][i][2],self.interface,50)
            print(returnMessageRelay)
            if returnMessageRelay != {}:
                returnMessageRelay = returnMessageRelay.get('NetworkQuality')
                
                if mapper[returnMessageRelay] < mapper[quality]:
                    quality = returnMessageRelay
            else:
                quality = "Disconnected"

        #Change telemetry when network quality declines or gets better
        self.changeTelemRate(quality)

        returnValue['Relay'] = quality

        return [returnValue]

    def changeTelemRate(self,quality):

        #Change telemetry sending rate and sensor sending rate when network quality changes 
        if quality == "HIGH" and self.rate != 0.4:
            sp.run("ros2 param set /" + self.drone_id + " telemetryRateMs 400",shell = True)
            self.destroy_timer(self.timer)
            self.rate = 0.4
            self.timer = self.create_timer(self.rate, self.pub_network_callback)
        elif quality == "MEDIUM" and self.rate != 0.5:
            sp.run("ros2 param set /" + self.drone_id + " telemetryRateMs 500",shell = True)
            self.destroy_timer(self.timer)
            self.rate = 0.5
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
