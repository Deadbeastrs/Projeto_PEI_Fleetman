# ROS2 H264 Image Transport

## Usage

First open the scripts folder, run the ./run_x264_encoder.sh -n <droneid> on the jetson with the webcam.
Second run the ./run_x264_decoder -n <droneid> on the jetson responsible for the decoding.
Third run the ./run_video_serber -a <address> -p <port> also on the decoding jetson.

The address must be on a different interface than the one used in the ad-hoc network.

For watching the video stream without the dashboard for testing purposes, edit the server/preview.html file in the folowing way:

Change the ip and port on there lines: 

```html
<img src="<address>:<port>/video_feed/<droneid>"
```

```html
var socket = io("ws://<address>:<port>");
```
