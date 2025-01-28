from picarx_improved import Picarx
import time
#import logging

'''logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)'''

class Sense(object):
    def __init__(self):
        self.px = Picarx()

    def read_stat(self):
        return self.px.grayscale.read()    

class Interp(object):
    def __init__(self, sensitivity = [0, 3600], polarity = False):
        self.polarity = polarity
        self.low_sense, self.high_sense = sensitivity
        self.robot_position = 0

    def locating_line(self, gs):
        if self.polarity == True:
            gs = [gs - min(gs) for gs in gs]
        else:
            gs = [abs(gs - max(gs)) for gs in gs]

        left, middle, right = gs

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

    def robot_position(self ):
        return self.robot_position
               

class Control(object):
    def __init__(self, threshold, kp, ki):
        self.px = Picarx()
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
    px = Picarx()
    sn = Sense()
    sn.read_stat