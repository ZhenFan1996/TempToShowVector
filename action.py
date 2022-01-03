import anki_vector
import cv2
import numpy as np
from PIL import Image
import os
import environment as env
import threading
import time
from anki_vector.util import degrees, distance_mm, speed_mmps,Angle
import asyncio
import random


event = threading.Event()
path = [(0,0)]
lastX ,lastY = 0,0

class env_Thread(threading.Thread):
    def __init__(self, event):
        threading.Thread.__init__(self)
        self.event = event
    def run(self):                 
       env.init(self.event)

class robot_Thread(threading.Thread):
    def __init__(self, event):
        threading.Thread.__init__(self)
        self.event = event
    def run(self):                 
       robot_init(robot)

def robot_init(robot):
      while True:
        #scanner(robot)
        proximity_data = robot.proximity.last_sensor_reading
        img_show(robot)
        pos = (robot.pose.position.x,robot.pose.position.y)
        position_transform(pos,100)
        env.update(path)
       

def scanner(robot):
  robot.motors.set_wheel_motors(100,100)
  while True:   
     wall = observe(robot)
     if wall is False:
       robot.motors.set_wheel_motors(100, 100)
      


def observe(robot):
    proximity_data = robot.proximity.last_sensor_reading
    if(proximity_data.distance.distance_mm <100):
        robot.motors.stop_all_motors()
        dir = random.choice([0,1])
        if dir ==0:
          robot.behavior.turn_in_place(degrees(90))
        else:
          robot.behavior.turn_in_place(degrees(-90))
        return True
    return False


def img_show(robot):
    photo = robot.camera.latest_image
    image = photo.raw_image
    img = np.array(image)       
    cv2.imshow('img',img)
    cv2.waitKey(3)
#def path_nav(robot):
#    global path,lastX,lastY
#    x,y = path[-1]
#    posX = robot.pose.position.x
#    posY = robot.pose.position.y
#    flag = False
#    if(posX - lastX > 50):
#        print((posX,lastX))
#        y +=1
#        lastX = posX
#        flag = True
#    if(posX - lastX < -50):
#        print((posX,lastX))
#        y-=1
#        lastX = posX
#        flag = True
#    if (posY - lastY > 50):
#        print((posY,lastY))
#        x-=1
#        lastY = posY
#        flag = True
#    if posY - lastY < -50:
#        print((posY,lastY))
#        x+=1
#        lastY = posY
#        flag = True
#    if flag:
#        path.append((x,y))
#    print(path[-1])

def position_transform(position,size):
    new = (int(position[0] // size),-int(position[1] // size))
    if path[-1] != new:
        print(new)
        path.append(new)
    #print(path[-1])


def initialize():
	robot = anki_vector.Robot()
	robot.connect()
	robot.camera.init_camera_feed()
	robot.behavior.set_head_angle(degrees(0))
	return robot


if __name__ == '__main__':
  robot = initialize()
  env_thread = env_Thread(event)
  robot_Thread = robot_Thread(robot)
  go_Thread = threading.Thread(target= scanner,args =[robot])
  env_thread.start()
  robot_Thread.start()
  go_Thread.start()



        
        
