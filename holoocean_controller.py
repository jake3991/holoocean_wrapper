import cv2
import holoocean
import numpy as np
from pynput import keyboard

from bruce_slam.CFAR import CFAR


from utils import generate_map

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


def parse_keys(keys, val):
    command = np.zeros(8)

    if 'i' in keys: # up
        command[0:4] += val
    if 'k' in keys: # down
        command[0:4] -= val
    if 'j' in keys: # rotate left
        command[[4,7]] += val
        command[[5,6]] -= val
    if 'l' in keys: # rotate right 
        command[[4,7]] -= val
        command[[5,6]] += val

    if 'w' in keys: # forward
        command[4:8] += val
    if 's' in keys: # backward
        command[4:8] -= val
    if 'a' in keys: # strafe left
        command[[4,6]] += val
        command[[5,7]] -= val
    if 'd' in keys: #strafe right
        command[[4,6]] -= val
        command[[5,7]] += val

    return command

scene = "test" # OpenWater-HoveringImagingSonar"
config = holoocean.packagemanager.get_scenario(scene)

with holoocean.make(scene) as env:
    while True:
        if 'q' in pressed_keys:
            break
        command = parse_keys(pressed_keys, force)

        #send to holoocean
        env.act("auv0", command)
        state = env.tick()

        if "DepthSensor" in state:
            print(state["DepthSensor"])

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

            cv2.imshow('Frame',horizontal_sonar_img)
            #cv2.imshow('Frame_2',vertical_sonar_img)

            cv2.imwrite("test.png",img)
 
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break



