from picarx_improved import Picarx
import time
import logging

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level = logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

class Sense(object):
    def __init__(self):
        self.px = Picarx()

    def read_stat(self):
        return self.px.grayscale.read()    

class Interp(object):
    def __init__(self, sensitivity = [0, 3600], polarity = False):
        ''' Initialize Interpreter
        
        param sensitivity: Indicates range of acceptable light -> dark values
        type sensitivity: list[int, int]
        param polarity: False indicates light floor, dark line, True indicates dark floor, light line
        type polarity: Bool      
        '''
        self.polarity = polarity
        self.low_sense, self.high_sense = sensitivity
        self.robot_position = 0

    def locating_line(self, gs):
        if self.polarity == True:
            gs = [gs - min(gs) for gs in gs]
        else:
            gs = [abs(gs - max(gs)) for gs in gs]

        left, middle, right = gs
        tolerance = 50
        center_left = left - middle
        center_right = right - middle
        total = left - right

        if right > left:
            self.robot_position = (right - middle)/max(right, middle)

    
    def robot_position(self, kp = 10, ):
        return self.robot_position
               

class Control(object):
    def __init__(self):
        self.px = Picarx()

if __name__ == "__main__":
    px = Picarx()
    sn = Sense()
    sn.read_stat