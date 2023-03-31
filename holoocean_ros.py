import cv2
import rospy
import holoocean
import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation

# import ros messages
from rti_dvl.msg import DVL
from sensor_msgs.msg import Imu
from bar30_depth.msg import Depth
from sonar_oculus.msg import OculusPing

# import from this package
from pid import Control
from utils import parse_keys
from bruce_slam.CFAR import CFAR

print(holoocean.util.get_holoocean_path())

depth_control = Control()
pressed_keys = list()
force = 25

def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))

def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

scene = "test" # OpenWater-HoveringImagingSonar"
config = holoocean.packagemanager.get_scenario(scene)
depth_command = 0
ticks_per_second = 200
step = 0

# init a rosnode
rospy.init_node("holoocean")

# define some publishers
dvl_pub = rospy.Publisher("/rti/body_velocity/raw",DVL,queue_size=20)
imu_pub = rospy.Publisher("/vectornav/IMU",Imu,queue_size=200)
depth_pub = rospy.Publisher("/bar30/depth/raw",Depth,queue_size=200)
sonar_pub = rospy.Publisher("/sonar_oculus_node/M750d/ping",OculusPing,queue_size=10)

with holoocean.make(scene) as env:
    while True:
        if 'q' in pressed_keys:
            break
        
        # get the command from the keyboard, keeping depth at the same level
        command = parse_keys(pressed_keys, force, depth_command)

        # send command and tick the env
        env.act("auv0", command)
        state = env.tick()

        # update the timestamp based on the number of ticks
        stamp = rospy.Time.from_sec(step / 200) 
        step += 1 # overload???

        if "DVLSensor" in state:

            # package into a ROS message
            dvl_msg = DVL()
            dvl_msg.velocity.x = state["DVLSensor"][0]
            dvl_msg.velocity.y = -state["DVLSensor"][1]
            dvl_msg.velocity.z = state["DVLSensor"][2]
            dvl_msg.header.stamp = stamp
            dvl_pub.publish(dvl_msg)

        if "IMUSensor" in state:
            pass

        if "PoseSensor" in state:
            # convert the pose sensor to an IMU message
            r,p,y = Rotation.from_matrix(state["PoseSensor"][:3, :3]).as_euler("xyz")
            x,y,z,w = Rotation.from_euler("xyz",[r+(np.pi/2),p,-y]).as_quat()
            imu_msg = Imu()
            imu_msg.orientation.x = x
            imu_msg.orientation.y = y
            imu_msg.orientation.z = z
            imu_msg.orientation.w = w
            imu_msg.header.stamp = stamp
            imu_pub.publish(imu_msg)

            # conver the post sensor to a depth message
            depth_command = depth_control.control_depth(state["PoseSensor"][2][3],-1)
            depth_msg = Depth()
            depth_msg.depth = state["PoseSensor"][2][3]
            depth_pub.publish(depth_msg)
            
        if "HorizontalSonar" in state:
            img = np.array(state["HorizontalSonar"])
            #img = img ** 3. # you can apply alpha to images here
            #img = img / np.max(img)
            img = np.array(img * 255).astype(np.uint8)
            params = [cv2.IMWRITE_JPEG_QUALITY,80]
            status, compressed = cv2.imencode(".jpg",img,params) # compress sonar images
            sonar_msg = OculusPing() # package as a sonar ping message
            sonar_msg.ping.data = compressed.tobytes()
            sonar_msg.header.stamp = stamp
            sonar_pub.publish(sonar_msg)




