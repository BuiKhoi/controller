import board
import busio
# import adafruit_pca9685
from adafruit_servokit import ServoKit

import time
from threading import Thread

EXECUTE_TIMEOUT = 200
CAMERA_STEP = 2

def millis():
    return int(time.time() * 1000)

class RobotController:
    def __init__(self):
        self.speed = 0
        self.last_speed = -1

        self.steer = 0
        self.last_steer = -1

        self.cam_pitch = 0
        self.cam_yaw = 0

        self.running = True
        self.pca = PCAHelper()

        self.ex_thread = Thread(target=self.execute_thread, args=[])
        self.ex_thread.start()

    def execute_thread(self):
        while self.running:
            #check for speed
            self.last_speed, self.speed = self.check_item(self.last_speed, self.speed, 0)

            #check for steer
            self.last_steer, self.steer = self.check_item(self.last_steer, self.steer, 0)

            #Execute these values
            self.pca.execute_values(self)

            #Wait for a certain of time
            time.sleep(EXECUTE_TIMEOUT/1000)

    def check_item(self, last_time, true_value, false_value):
        if last_time != -1:
            if (millis() - last_time) > EXECUTE_TIMEOUT:
                return -1, false_value
            else:
                return last_time, true_value
        else:
            return -1, false_value

    def process_command(self, cmd_text):
        if cmd_text == 'lfwd':
            self.speed = 60
            self.last_speed = millis()
            self.steer = 100
            self.last_steer = millis()

        elif cmd_text == 'fwd':
            self.speed = 100
            self.last_speed = millis()

        elif cmd_text == 'rfwd':
            self.speed = 60
            self.last_speed = millis()
            self.steer = -100
            self.last_steer = millis()

        elif cmd_text == 'lbck':
            self.speed = -60
            self.last_speed = millis()
            self.steer = 100
            self.last_steer = millis()

        elif cmd_text == 'bck':
            self.speed = -100
            self.last_speed = millis()

        elif cmd_text == 'rbck':
            self.speed = -60
            self.last_speed = millis()
            self.steer = -100
            self.last_steer = millis()

        elif cmd_text == 'cup':
            self.cam_pitch = min(self.cam_pitch + CAMERA_STEP, 100)
        elif cmd_text == 'cleft':
            self.cam_yaw = max(self.cam_yaw - CAMERA_STEP, -100)
        elif cmd_text == 'cright':
            self.cam_yaw = min(self.cam_yaw + CAMERA_STEP, 100)
        elif cmd_text == 'cdown':
            self.cam_pitch = max(self.cam_pitch - CAMERA_STEP, -100)
        elif cmd_text == 'ccenter':
            self.cam_pitch = 0
            self.cam_yaw = 0
        

class PCAHelper:
    def __init__(self):
        # self.i2c = busio.I2C(board.SCL, board.SDA)
        # self.pca = adafruit_pca9685.PCA9685(self.i2c)
        self.kit = ServoKit(channels=16)
        for i in range(4):
            self.kit.servo[i].set_pulse_width_range(1000, 2000)

    def execute_values(self, robot):
        print('Speed and steer: {}, {}'.format(robot.speed, robot.steer))
        print('Cam pitch and yaw: {}, {}'.format(robot.cam_pitch, robot.cam_yaw))
        self.kit.servo[0].angle = self.map(-robot.speed, -100, 100, 60, 120)
        self.kit.servo[1].angle = self.map(-robot.steer, -100, 100, 0, 180)
        self.kit.servo[2].angle = self.map(robot.cam_pitch, -100, 100, 0, 180)
        self.kit.servo[3].angle = self.map(robot.cam_yaw, -100, 100, 0, 180)

    def map(self, value, value_min, value_max, target_min, target_max):
        value = value - value_min
        ratio = value / (value_max - value_min)
        new_value = ratio * (target_max - target_min)
        new_value = target_min + new_value
        return int(new_value)