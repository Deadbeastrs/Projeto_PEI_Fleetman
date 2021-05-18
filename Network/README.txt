In the fleetman project place the following files in the respective folders

network.yml -> fleet-manager/configs/sensors
Network.yml -> fleet-manager/tools/sensors/configs
Network_sim_radious.yml -> fleet-manager/tools/sensors/configs
network.py -> fleet-manager/tools/sensors
Network_simulated.py -> fleet-manager/tools/sensors

Needed programs:
-ping
-iw
-matplotlib
-haversine

Include the following in the drone dockerfile

RUN apt update

#Install iw
RUN apt-get install iw -y

RUN apt-get install python3-pip -y

#Install ping
RUN apt-get install iputils-ping -y

#Install haversine
RUN pip3 install haversine

#Install haversine
RUN pip3 install matplotlib
