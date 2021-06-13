# How to run

## Camera Module

### Encoder

To run the encoder, execute the Video/scripts/run_x264_encoder.sh with the arguments -n `DroneId`.
 
```sh
./run_x264_encoder.sh -n <droneId>
```

### Decoder and server

To run the decoder, execute the Video/scripts/run_x264_decoder.sh with the arguments -n `DroneId`, corresponding to the encoder id.

```sh
./run_x264_decoder.sh -n <droneId>
```

To run the flask server on the same device as the decoder. Run the Video/scripts/run_video_server.sh with arguments -a `address` -p `port`.
```sh
./run_video_server.sh -a <ipAddress> -p <port>
```
Launch the dashboard, add a drone and when questioned about the ip, enter `address`:`port` configured in the flask server.

## Network Sensor

The network sensor runs on the drone.
First launch the drone container on the drone located on the fleet-manager repository, on the scripts folder.
```sh
./launch_drone_container.sh -c ../configs/<desired_cfg> <droneId>
```
Second, open another terminal session, enter the docker container and change the configuration file appropriately (change Groundstation Ip and Mac address) 
```sh
docker exec -it <droneId> bash
cd src/sensors/configs && vim Network.yml
```
Run the sensor:
```sh
python3 Network.py -c configs/Network.yml
```

