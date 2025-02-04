from picarx_improved import Picarx
import time
#import logging
try:
    from vilib import Vilib
except:
    pass
import numpy as np
import cv2 
import concurrent.futures
from readerwriterlock import rwlock

'''logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)'''

class Sense(object):
    def __init__(self, s_delay, camera = False):
        self.px = Picarx()
        self.reference = np.array(self.px.grayscale._reference)
        #self.si_bus = si_bus
        self.s_delay = s_delay

        if camera == True:
            Vilib.camera_start()
            #Vilib.display()
            time.sleep(0.5)
            self.path = "picarx"
            self.image_name = "image"
            self.px.set_cam_tilt_angle(-30)

    def read_gray_stat(self):
        return self.px.grayscale.read() 

    def take_photo(self, si_bus):
        while True:
            Vilib.take_photo(photo_name = self.image_name, path = self.path)
            si_bus.write(f'{self.path}/{self.image_name}')  
            time.sleep(self.s_delay) 

    def set_bus_grayscale(self, si_bus):
        while True:
            si_bus.write(self.read_gray_stat())
            print(f'Red grayscale value')
            time.sleep(self.s_delay)

class Interp(object):
    def __init__(self, s_delay, c_delay, sensitivity = [0, 3600], polarity = False, t= 60):
        self.polarity = polarity
        self.low_sense, self.high_sense = sensitivity
        self.robot_position = 0
        self.img_threshold = t
        self.color = 255
        self.img_start = 350
        self.img_cutoff = 425
        #self.si_bus = si_bus
        self.s_delay = s_delay
        #self.ic_bus = ic_bus
        self.c_delay = c_delay

    def locating_line_g(self, si_bus, ic_bus):

        while True:
            gs_v = si_bus.read()
            print(f'Gray Transfer')
            time.sleep(self.s_delay)

            if self.polarity == True:
                gs_v = [gs - min(gs_v) for gs in gs_v]
            else:
                gs_v = [abs(gs - max(gs_v)) for gs in gs_v]

            left, middle, right = gs_v

            if right >= left:
                robot_position = (middle - right)/max(right, middle)
                
                if robot_position < 0:
                    robot_position = -1 * robot_position
                    time.sleep(self.s_delay)
                    continue
                else:
                    robot_position = 1 - robot_position
                    time.sleep(self.s_delay)
                    continue
            
            elif left > right:
                robot_position = (middle - left)/max(left, middle)

                if robot_position < 0:
                    robot_position = robot_position
                    time.sleep(self.s_delay)
                    continue

                else:
                    robot_position = robot_position - 1
                    time.sleep(self.s_delay)
                    continue

            ic_bus.write(self.robot_position)
            print(f'Changed Position')
            time.sleep(self.c_delay)

    def locating_line_c(self, si_bus, ic_bus):
        while True:
            file_n = si_bus.read()
            time.sleep(self.s_delay)
            img = cv2.imread(f'{file_n}.jpg')
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
                robot_position = (cX - half_width)/ half_width
                time.sleep(self.s_delay)

            ic_bus.write(robot_position)
            print(f'Changed Position')
            time.sleep(self.c_delay)
               

class Control(object):
    def __init__(self, threshold, c_delay, kp = 30.0, ki = 0.0):
        #self.px = Picarx()
        self.threshold = threshold
        self.kp = kp
        self.ki = ki
        self.e = 0.0
        self.angle = 0.0
        #self.ic_bus = ic_bus
        self.c_delay = c_delay

    def auto_steering(self, px, ic_bus):
        while True:
            position = ic_bus.read()
            time.sleep(self.c_delay)
            if abs(position) > self.threshold:
                self.e = self.e + position
                self.angle = (self.kp * position) + (self.ki * self.e)
                px.set_dir_servo_angle(self.angle)
                time.sleep(self.c_delay)
                print(f'Move')
                continue

class Bus(object):
    def __init__(self):
        self.message = None
        self.lock = rwlock.RWLockWrite()

    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return message

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

def handle_exception(future):
    exception = future.exception()
    if exception:
        print(f'Exception in worker thread: {exception}')


if __name__ == "__main__":
    #px = Picarx()
    running = True 
    
    while running == True:
        value = input("Enter line following type ('a': greyscale 'b': camera 'c': quit):")
        threshold = input("Enter threshold value:")
        threshold = float(threshold)
        polarity = input("Enter 'False' if line is darker than floor and 'True' if line is lighter than floor:")

        sense_interp_bus = Bus()
        interp_control_bus = Bus()

        sense_delay = 0.1
        control_delay = 0.1

        if value == 'a':
            sense = Sense(s_delay = sense_delay, camera = False)
            think = Interp(s_delay = sense_delay, c_delay = control_delay, polarity = polarity)
            act = Control(threshold= threshold, c_delay = control_delay)
            time.sleep(1)
            sense.px.forward(30)
            '''while True:
                think.locating_line_g(sense.read_gray_stat())
                robot_position = think.robot_location()
                act.auto_steering(robot_position, sense.px)'''
            with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
                eSensor = executor.submit(sense.set_bus_grayscale, sense_interp_bus)
                eSensor.add_done_callback(handle_exception)
                eInterp = executor.submit(think.locating_line_g, sense_interp_bus, interp_control_bus)
                eInterp.add_done_callback(handle_exception)
                eControl = executor.submit(act.auto_steering, sense.px, interp_control_bus)
                eControl.add_done_callback(handle_exception)

        if value == 'b':
            img_t = input("Enter camera threshold value:")
            img_t = float(img_t)
            sense = Sense(s_delay = sense_delay, camera = True)
            think = Interp(s_delay = sense_delay, c_delay = control_delay, polarity = polarity, t = img_t)
            act = Control(threshold= threshold, c_delay = control_delay)
            time.sleep(1)
            sense.px.forward(30)
            '''while True:
                sense.take_photo()
                print(f'photo')
                think.line_locating_c(sense.image_name, sense.path)
                robot_position = think.robot_location()
                act.auto_steering(robot_position, sense.px)'''

            with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
                eSensor = executor.submit(sense.take_photo, sense_interp_bus)
                eInterpreter = executor.submit(think.locating_line_c, sense_interp_bus, interp_control_bus)
                eControl = executor.submit(act.auto_steering, interp_control_bus)