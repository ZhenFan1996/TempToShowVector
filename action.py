import anki_vector
import cv2
import numpy as np
from PIL import Image
import os
from anki_vector.util import degrees
import environment as env
import threading
import time


class env_Thread(threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):                 
       env.init()

class robot_Thread(threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):                 
       robot_init()

def robot_init():
   with anki_vector.AsyncRobot(enable_nav_map_feed=True) as robot:
      robot.camera.init_camera_feed()
      robot.motors.set_wheel_motors(50, 50)
      x =1
      while True:        
        proximity_data = robot.proximity.last_sensor_reading
        photo = robot.camera.latest_image
        image = photo.raw_image
        img = np.array(image)       
        cv2.imshow('img',img)     
        env.follow((x,0))
        x+=1
        cv2.waitKey(10)


if __name__ == '__main__':
  env_thread = env_Thread(1,'env',1)
  robot_Thread = robot_Thread(2,'robot',2)

  env_thread.start()
  robot_Thread.start()



        
        
