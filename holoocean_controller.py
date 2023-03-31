import cv2
import rospy
import holoocean
import numpy as np
from pynput import keyboard

import matplotlib.pyplot as plt

from pid import Control
from utils import generate_map, parse_keys
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

plt.ion()
step = 0
xs = []
ys = []

with holoocean.make(scene) as env:
    while True:
        if 'q' in pressed_keys:
            break
        command = parse_keys(pressed_keys, force, depth_command)

        #send to holoocean
        env.act("auv0", command)
        state = env.tick()

        if "VelocitySensor" in state:
            '''xs.append(step)
            ys.append(state["VelocitySensor"][0])
            step += 1
            plt.plot(xs,ys)
            plt.draw()
            plt.pause(0.0001)
            plt.clf()
            plt.ylim(1,-1)'''
            pass
            

        if "PoseSensor" in state:
            depth_command = depth_control.control_depth(state["PoseSensor"][2][3],-1)
            
        if "HorizontalSonar" in state:

            map_x, map_y = generate_map(config)
            img = np.array(state["HorizontalSonar"] * 255).astype(np.uint8)
            horizontal_sonar_img = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
            detector = CFAR(40, 20, 0.1, None)
            threshold = 85
            peaks = detector.detect(img, "SOCA")
            peaks &= img > threshold
            peaks_r = cv2.remap(peaks, map_x, map_y, cv2.INTER_LINEAR)
            locs = np.c_[np.nonzero(peaks_r)]


            for loc in locs:
                cv2.circle(horizontal_sonar_img, (loc[1],loc[0]),5, (255), -1)


            #img = np.array(state["VerticalSonar"] * 255).astype(np.uint8)
            #vertical_sonar_img = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

            #cv2.imshow('Frame',horizontal_sonar_img)
            #cv2.imshow('Frame_2',vertical_sonar_img)

            #cv2.imwrite("test.png",img)
 
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break



