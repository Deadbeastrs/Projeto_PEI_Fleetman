from threading import Thread, Event
import rclpy
from flask import Flask, Response, render_template, jsonify
from flask import request
from flask_socketio import SocketIO
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import json
import time
import signal
import sys
from std_srvs.srv import SetBool
from flask_cors import CORS, cross_origin
    
class drone:
    frame = None
    event = Event()
    bridge = CvBridge()
    subscription_image = None
    subscritpion_control = None
    drone_id = None
    status = None
    node = None
    cli = None
    req = SetBool.Request()

    def __init__(self,drone_id,node):
        self.node = node
        self.cli = node.create_client(SetBool, drone_id + '_enc/quality_info')
        self.drone_id = drone_id
        self.status = False

    def get_frame(self):
        self.event.wait()
        self.event.clear()
        return self.frame

    def changeQuality(self,quality):
        self.subscritpion_control.publish(quality)
    
    def start(self):
        self.subscription = node.create_subscription(Image, self.drone_id + "_dec/decoded", self.on_image, 100)
        self.subscritpion_control = node.create_publisher(String, self.drone_id + "_enc/control", 10)
        self.status = True

    def stop(self):
        node.destroy_subscription(self.subscription)
        node.destroy_publisher(self.subscritpion_control)
        self.status = False

    def on_image(self,msg):
        try:
            #print(msg.header)
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            ret, jpeg = cv2.imencode('.jpg', image)
            self.frame = jpeg.tobytes()
            self.event.set()
            
        except:
            print("Lost Frames")
    
    def getQuality(self):
        future = self.cli.call(self.req)
        return future.message

def signal_handler(sig, frame):
    [drone_list[a].stop() for a in drone_list.keys()]
    sys.exit(0)

server = Flask(__name__)
socketio = SocketIO(server,cors_allowed_origins='*')
rclpy.init()
global node
node = rclpy.create_node("listener_stream")
print(node.get_node_names())

signal.signal(signal.SIGINT, signal_handler)

drone_list = {}
drone_list['drone01'] = drone("drone01",node)
drone_list['drone02'] = drone("drone02",node)
drone_to_ids = {}
drone_to_ids['drone01'] = []
drone_to_ids['drone02'] = []
ros_node = None
start1 = True
CORS(server)

def gen(stop):
    while True:
        if not stop() :
            print("IM OUT!")
            sys.exit(0)
        rclpy.spin_once(node)
    
def stream(drone):
    while True:
        if drone_list[drone].frame != None:
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + drone_list[drone].frame + b'\r\n\r\n')

@server.route('/video_feed/<drone>')
def video_feed(drone):
    return Response(stream(drone), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def test_connect():
    print("New Client")

@socketio.on('drone')
def drone_s(drone):
    if drone_to_ids == None :
        print("No drone with id " + str(drone))
        return

    if drone_to_ids[drone] == []:
        if not drone_list[drone].status :
            drone_list[drone].start()

    drone_to_ids[drone].append(request.sid)

@socketio.on('disconnect')
def test_disconnect():
    drone = [l for l in drone_to_ids.keys() if request.sid in drone_to_ids[l]]
    if drone == []:
        return
    drone_to_ids[drone[0]].remove(request.sid)
    if drone_to_ids[drone[0]] == []:
        drone_list[drone[0]].stop()
        print("Disconnected Stream from -> " + drone[0])

@server.route('/send/<drone>', methods = ['POST'])
def send_feed(drone):
    quality = request.args.get('quality')
    newmsg = String()
    newmsg.data = quality
    drone_list[drone].changeQuality(newmsg)
    return "Done"

@server.route('/stop/<drone>', methods = ['GET'])
def stop_feed(drone):
    drone_list[drone].stop()
    return "Done"

@server.route('/getQuality/<drone>', methods = ['GET'])
def getQuality(drone):
    return drone_list[drone].getQuality()

if __name__ == '__main__':
    ros_node = Thread(target = gen, args=(lambda: start1,))
    ros_node.start()
    socketio.run(server,host=sys.argv[1], port=int(sys.argv[2]), debug=False)

