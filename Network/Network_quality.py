import sys
import subprocess as sp
import re
import time
import psutil
import json

def main(interface,ip,mac,times):
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
            return ""
        TxByte = re.findall("(?<=tx bytes:\\s)\\d+",res)
        Signal = re.findall("(?<=signal:\\s{2}\\t)(-?\\d+)",res)
        TxBit = re.findall("(?<=tx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)
        RxBit = re.findall("(?<=rx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res) 
        Throughput = re.findall("(?<=expected throughput:\\s)((\d+.\d+))",res) 
        Station = {
        "Station": mac,
        "IP": ip,
        "Latency": 0,
        "Expected_throughput": Throughput[0][0],
        "Signal": str(abs(int(Signal[0]))),
        "TxByte": TxByte[0],
        "RxBit": RxBit[0],
        "TxBit" : TxBit[0],
        }

        #Prevent error when Rxbit is unkwon
        if len(RxBit) != len(Signal):
            return {}
        lista.append(Station)

    Station =  {
        "Station": mac,
        "IP": ip,
        "Expected_throughput": 0,
        "Signal": 0,
        "TxByte": lista[0].get('TxByte'),
        "RxBit": 0,
        "TxBit" : 0,
        }


    #retrive the interface loss percentage, the avg jitter and the round trip time with mtr command
    res = sp.check_output('mtr -r -j -n -o LMDNBAW ' + ip + ' -I ' + interface + ' -c 10 --interval 1',shell = True)
    res = res.decode("utf-8")
    dicionary = json.loads(res)
    Loss = dicionary.get('report').get('hubs')[0].get('Loss%')
    Jitter = dicionary.get('report').get('hubs')[0].get('Javg')
    latency = dicionary.get('report').get('hubs')[0].get('Avg')

    #Sum all the Rxbit, TxBit and Signal parameters    
    for Message in lista:
        Station['Signal'] = Station['Signal'] + float(Message.get('Signal'))
        Station['RxBit'] = Station['RxBit'] +float(Message.get('RxBit').split(" ")[0]) 
        Station['TxBit'] = Station['TxBit'] + float(Message.get('TxBit').split(" ")[0])
        Station['Expected_throughput'] = Station['Expected_throughput'] + float(Message.get('Expected_throughput')) 
 

    #Divide the Parameters to get the avarage
    RxBit = round(Station['RxBit']/times,2)
    TxBit = round(Station['TxBit']/times,2)
    Expected_throughput = round(Station['Expected_throughput']/times,2)
    Signal = Station['Signal'] /times
    Station['Avarage_RTT'] = str(round(latency,2)) + " ms"
    Station['Expected_throughput'] = str(Expected_throughput) + "MBps" 
    Station['Signal'] = str(Signal)
    Station['RxBit'] = str(RxBit) + " MBit/s"
    Station['TxBit'] = str(TxBit) + " MBit/s"
    Station['Avg_Jitter'] = str(Jitter) + " ms"
    Station['Loss%'] = str(Loss) + '%'

    NetworkQuality = ""

    #Use the above parameters to make a estimate of the network quality
    if (Signal) < 60 and (latency) < 50 and (RxBit) > 5 and Jitter < 30 and Loss < 1 and TxBit > 5 and Expected_throughput > 5:
        NetworkQuality = "HIGH"
    elif (Signal) < 80 and (latency) < 100 and (RxBit) > 3 and Jitter < 50 and Loss < 2 and TxBit > 5 and Expected_throughput > 5:
        NetworkQuality = "MEDIUM"
    else:
        NetworkQuality = "LOW"

    Station['Estimated_Network_Quality'] = NetworkQuality
    return Station

if __name__ == '__main__':
    if sys.argv[1] == "-h":
        print("USE argv[1] = interface, argv[2] = ip, argv[3] = mac, argv[4] = avarage of x mesurments,argv[5] = times this programs runs")
    else:
        for i in range(int(sys.argv[5])):
            interface = sys.argv[1]
            ip = sys.argv[2]
            mac = sys.argv[3]
            times = sys.argv[4]
            x = main(interface,ip,mac,int(times))
            print(x)
            print("")
        
