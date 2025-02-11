from picarx_improved import Picarx
import time
#import logging
try:
    from vilib import Vilib
except:
    pass
import numpy as np
import cv2 
import rossros as ros

'''logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)'''

class Sense(object):
    def __init__(self, px, camera = False):
        self.px = px
        self.reference = np.array(self.px.grayscale._reference)
        if camera == True:
            Vilib.camera_start()
            #Vilib.display()
            time.sleep(0.5)
            self.path = "picarx"
            self.image_name = "image"
            self.px.set_cam_tilt_angle(-30)

    def read_gray_stat(self):
        return self.px.grayscale.read() 

    def take_photo(self):
        Vilib.take_photo(photo_name = self.image_name, path = self.path)  
        time.sleep(0.1) 

    def get_ultrasonic(self):
        return self.px.get_distance()

class Interp(object):
    def __init__(self, sensitivity = [0, 3600], polarity = False, t= 60):
        self.polarity = polarity
        self.low_sense, self.high_sense = sensitivity
        self.robot_position = 0
        self.img_threshold = t
        self.color = 255
        self.img_start = 350
        self.img_cutoff = 425

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
                return self.robot_position
            else:
                self.robot_position = 1 - self.robot_position
                return self.robot_position
        
        elif left > right:
            self.robot_position = (middle - left)/max(left, middle)

            if self.robot_position < 0:
                self.robot_position = self.robot_position
                return self.robot_position

            else:
                self.robot_position = self.robot_position - 1
                return self.robot_position

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
    def __init__(self, px, threshold, stop = 10, kp = 30.0, ki = 0.0):
        self.px = px
        self.threshold = threshold
        self.kp = kp
        self.ki = ki
        self.e = 0.0
        self.angle = 0.0
        self.stop_distance = stop

    def auto_steering(self, position):
        if abs(position) > self.threshold:
            self.e = self.e + position
            self.angle = (self.kp * position) + (self.ki * self.e)
            self.px.set_dir_servo_angle(self.angle)
            return self.angle
        
    def ultrasonic_stop(self, distance):
            if distance < self.stop_distance:
                self.px.stop()

            else:
                self.px.forward(35)
            

if __name__ == "__main__":
    px = Picarx()
    value = input("Enter line following type ('a': greyscale 'b': camera 'c': quit):")
    threshold = input("Enter threshold value:")
    threshold = float(threshold)
    polarity = input("Enter 'False' if line is darker than floor and 'True' if line is lighter than floor:")

    sense_delay = 0.01
    interp_delay = 0.02
    control_delay = 0.1
    full_time = 15
    check_time = 0.01
    print_delay = 0.25

    if value == 'a':
        sense = Sense(px = px, camera = False)
        think = Interp(polarity = polarity)
        act = Control(px = px, threshold= threshold)
        '''while True:
            think.locating_line_g(sense.read_stat())
            robot_position = think.robot_location()
            act.auto_steering(robot_position, sense.px)'''

    if value == 'b':
        img_t = input("Enter camera threshold value:")
        img_t = float(img_t)
        sense = Sense(px = px, camera = True)
        think = Interp(polarity = polarity, t = img_t)
        act = Control(px = px, threshold= threshold)
        '''while True:
            sense.take_photo()
            print(f'photo')
            think.line_locating_c(sense.image_name, sense.path)
            robot_position = think.robot_location()
            act.auto_steering(robot_position, sense.px)'''
        
    time.sleep(1)
    sense.px.forward(35)
    si_bus = ros.Bus(sense.read_gray_stat(), "Grayscale Value")
    ic_bus = ros.Bus(think.locating_line_g(sense.read_gray_stat()), "Position Calcs")
    ultrasonic_bus = ros.Bus(sense.get_ultrasonic(), "Ultrasonic Bus")
    ic_bus_u = ros.Bus(act.ultrasonic_stop(sense.get_ultrasonic()), "Ultrasonic Data")
    terminate_bus = ros.Bus(0, "Termination Bus")

    read_grayscale = ros.Producer(sense.read_gray_stat,si_bus,sense_delay,terminate_bus,"Read Grayscale values")

    read_ultrasonic = ros.Producer(sense.get_ultrasonic,ultrasonic_bus,sense_delay,terminate_bus,"Read Ultrasonic values")

    find_position = ros.ConsumerProducer(think.locating_line_g, si_bus, ic_bus, interp_delay, terminate_bus, "Calculate distance from line")

    determine_stop = ros.ConsumerProducer(act.ultrasonic_stop, ultrasonic_bus, ic_bus_u, interp_delay, terminate_bus, "Calculate distance")

    steering = ros.Consumer(act.auto_steering, ic_bus, control_delay, terminate_bus, "Lets ride")

    print_buses = ros.Printer((si_bus, ic_bus, ic_bus_u, terminate_bus),print_delay,terminate_bus,"Print raw data","Data bus readings are: ")

    terminate_timer = ros.Timer(terminate_bus,full_time,check_time,terminate_bus,"Termination Timer")

    producer_consumer_list = [read_grayscale,
                              find_position,
                              steering,
                              print_buses,
                              terminate_timer,
                              determine_stop,
                              read_ultrasonic
                              ]
    
    ros.runConcurrently(producer_consumer_list)