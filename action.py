import anki_vector
import cv2
import numpy as np
from PIL import Image
import os
from anki_vector.util import degrees
import environment as env
import threading
import time
from anki_vector.util import degrees, distance_mm, speed_mmps,Angle
import asyncio 


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
       robot_init(self.event)

def robot_init(event):
   with anki_vector.AsyncRobot(enable_nav_map_feed=True) as robot:
      robot.camera.init_camera_feed()
      pose = anki_vector.util.Pose(x=1200, y=-1200, z=0, angle_z=anki_vector.util.Angle(degrees=0))
      robot.behavior.go_to_pose(pose)     
      curr = time.time()
      while True:        
        proximity_data = robot.proximity.last_sensor_reading
        photo = robot.camera.latest_image
        image = photo.raw_image
        img = np.array(image)       
        cv2.imshow('img',img)  
        if time.time() - curr > 0.01:
            curr = time.time()
            path_nav(robot)
            env.follow(path)
        cv2.waitKey(5)



def go():
    with anki_vector.AsyncRobot(enable_nav_map_feed=True) as robot:
     for _ in range(4):
         print("Drive Vector straight...")
         robot.behavior.drive_straight(distance_mm(200), speed_mmps(50))
         print("Turn Vector in place...")
         robot.behavior.turn_in_place(degrees(90))


def path_nav(robot):
    global path,lastX,lastY   
    x,y = path[-1]
    posX = robot.pose.position.x
    posY = robot.pose.position.y
    flag = False
    if(posX - lastX > 50):
        print((posX,lastX))
        y +=1
        lastX = posX
        flag = True
    if(posX - lastX < -50):
        print((posX,lastX))
        y-=1
        lastX = posX
        flag = True
    if (posY - lastY > 50):
        print((posY,lastY))
        x-=1
        lastY = posY
        flag = True
    if posY - lastY < -50:
        print((posY,lastY))
        x+=1
        lastY = posY
        flag = True
    if flag:
        path.append((x,y))
    print(path[-1])


if __name__ == '__main__':
  
  env_thread = env_Thread(event)
  robot_Thread = robot_Thread(event)

  env_thread.start()
  robot_Thread.start()



        
        
