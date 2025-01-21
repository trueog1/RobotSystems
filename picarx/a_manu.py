from .picarx_improved import Picarx
import time

def forward_and_backward(angle):
    px = Picarx()
    px.set_dir_servo_angle(angle)
    px.forward(50)
    time.sleep(5)
    px.backward(50)
    time.sleep(5)