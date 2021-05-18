
from threading import Thread, Event
import rclpy
from flask import Flask, Response, render_template
from flask import request
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import json


frames = []
event = Event()
bridge = CvBridge()
subs = []

class videoInstance:
    

def on_image(msg):
    global frame
    #print("ON IMAGE")
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    ret, jpeg = cv2.imencode('.jpg', image)
    frame = jpeg.tobytes()
    event.set()


#rospy.init_node("listener_stream")
#rospy.Subscriber("raw_image", Image, on_image) # "raw_image" is the default topic; if needed, change the topic name to yours
rclpy.init()
global node 
node = rclpy.create_node("listener_stream")

subscritpionQ = node.create_publisher(String, "ola/control", 10)
print("Subscription")

""" while rclpy.ok():
    rclpy.spin_once(node)
    print("Spinning")
 """

server = Flask(__name__)



def get_frame():
    event.wait()
    event.clear()
    #print("Get Frame")
    return frame

def gen():
    while True:
        rclpy.spin_once(node)
    
def stream(drone):
    while True:
        frame = get_frame()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@server.route('/')
def index():
    return render_template('index.html')

@server.route('/video_feed/<drone>')
def video_feed(drone):
    subs.add(node.create_subscription(Image, "teste/decoded", on_image, 100))
    return Response(stream(drone),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@server.route('/send', methods = ['POST'])
def send_feed():
    print("Vou enviar")
    dic = json.loads(request.data.decode('utf-8'))
    quality = dic['quality']
    print(quality)
    newmsg = String()
    newmsg.data = quality
    subscritpionQ.publish(newmsg)
    return "Done"

if __name__ == '__main__':
    ros_node = Thread(target = gen, args = ())
    ros_node.start()
    server.run(host='127.0.0.1', port=8090, debug=True)
    
    
