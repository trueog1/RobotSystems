from picarx_improved import Picarx
import time
#import logging
from vilib import Vilib
import numpy as np
import cv2 

'''logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)'''

class Sense(object):
    def __init__(self, camera = False):
        self.px = Picarx()
        self.reference = np.array(self.px.grayscale._reference)
        if camera == True:
            Vilib.camera_start()
            Vilib.display()
            time.sleep(0.5)
            self.path = "picarx"
            self.image_name = "image"
            self.px.set_cam_tilt_angle(-30)

    def read_stat(self):
        return self.px.grayscale.read() 

    def take_photo(self):
        Vilib.take_photo(photo_name = self.image_name, path = self.path)  
        time.sleep(0.5) 

class Interp(object):
    def __init__(self, sensitivity = [0, 3600], polarity = False, t= 60):
        self.polarity = polarity
        self.low_sense, self.high_sense = sensitivity
        self.robot_position = 0
        self.img_threshold = t
        self.color = 255
        self.img_start = 350
        self.img_cutoff = 450

    def locating_line_g(self, gs_v):
        if self.polarity == True:
            gs_v = [gs - min(gs_v) for gs in gs_v]
        else:
            gs_v = [abs(gs - max(gs_v)) for gs in gs_v]

        left, middle, right = gs_v

        if right >= left:
            self.robot_position = (middle - right)/max(right, middle)
            
            if self.robot_position < 0:
                self.robot_position = -1 * self.robot_position
                return
            else:
                self.robot_position = 1 - self.robot_position
                return
        
        elif left > right:
            self.robot_position = (middle - left)/max(left, middle)

            if self.robot_position < 0:
                self.robot_position = self.robot_position

            else:
                self.robot_position = self.robot_position - 1
                return

    def line_locating_c(self, image_name, path):
        img = cv2.imread(f'{path}/{image_name}.jpg')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = img[self.img_start:self.img_cutoff, :]
        img_height, img_width = img.shape
        half_width = img_width / 2

        if self.polarity == True:
            _, thresh = cv2.threshold(img, thresh = self.img_threshold, maxval = self.color, type = cv2.THRESH_BINARY_INV)
        
        else:
            _, thresh = cv2.threshold(img, thresh = self.img_threshold, maxval =self.color, type = cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        big_c = max(contours, key=cv2.contourArea)
        M = cv2.moments(big_c)

        if M['m00'] != 0:
            cX = int(M["m10"] / M["m00"])
            #cY = int(M["m01"] / M["m00"])
            self.robot_position = (cX - half_width)/ half_width

    def robot_location(self):
        return self.robot_position
               

class Control(object):
    def __init__(self, threshold, kp = 30.0, ki = 0.0):
        #self.px = Picarx()
        self.threshold = threshold
        self.kp = kp
        self.ki = ki
        self.e = 0.0
        self.angle = 0.0

    def auto_steering(self, position, px):
        if abs(position) > self.threshold:
            self.e = self.e + position
            self.angle = (self.kp * position) + (self.ki * self.e)
            px.set_dir_servo_angle(self.angle)
            return self.angle

if __name__ == "__main__":
    #px = Picarx()
    running = True
    
    while running == True:
        value = input("Enter line following type ('a': greyscale 'b': camera 'c': quit):")
        threshold = input("Enter threshold value:")
        threshold = float(threshold)
        polarity = input("Enter 'False' if line is darker than floor and 'True' if line is lighter than floor:")

        if value == 'a':
            sense = Sense()
            think = Interp(polarity = polarity)
            act = Control(threshold= threshold)
            time.sleep(1)
            sense.px.forward(5)
            while True:
                think.locating_line_g(sense.read_stat())
                robot_position = think.robot_location()
                act.auto_steering(robot_position, sense.px)

        if value == 'b':
            img_t = input("Enter camera threshold value:")
            img_t = float(img_t)
            sense = Sense(camera = True)
            think = Interp(polarity = polarity, t = img_t)
            act = Control(threshold= threshold)
            time.sleep(1)
            sense.px.forward(2)
            while True:
                sense.take_photo()
                think.line_locating_c(sense.image_name, sense.path)
                robot_position = think.robot_location()
                act.auto_steering(robot_position, sense.px)