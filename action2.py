import anki_vector
import cv2
import numpy as np
import threading
import time
from anki_vector.util import degrees, distance_mm, speed_mmps,Angle,Pose
from anki_vector.behavior import MIN_LIFT_HEIGHT_MM, MAX_LIFT_HEIGHT_MM
import asyncio
import keyboard

robot = anki_vector.AsyncRobot(serial= '008014c1',enable_nav_map_feed= True)
speed =  60
offset = 100
path_dis = 300
last_time = time.time()

stack = []

class node():
    def __init__(self,x,y,degree):
        self.x = x
        self.y = y 
        self.pos = (x,y)
        self.canGo = [0,0,0,0]
        self.angle_z = Angle(degrees = degree)
        self.pose = robot.pose
        self.type = 0
        self.dirPose = [None] *4

    def isInAreaNode(self,position):
        x = position[0]
        y = position[1]
        if x > self.pos[0] -50 and x<self.pos[0]+50 and y >self.pos[1] -50 and y<self.pos[1]+50:
            return True
        return False


def dir_correct():
    dir = get_dir()
    degree = robot.pose.rotation.angle_z.degrees
    diff = 0
    de = 0
    if dir==0: # 上 0
     diff = degree
     de= 0
    elif dir ==1: #右 -90
     diff = degree+90
     de =- 90
    elif dir==3: #左 90 
     diff = degree-90
     de =90
    elif dir ==2: #下 180或-180
     if(degree<0):
         diff = degree+180
         de = 180
     else:
         diff= degree-180
         de= -180
    if abs(diff) >4:
       robot.motors.stop_all_motors()
       turn_future = robot.behavior.turn_in_place(angle = degrees(de),is_absolute =True,speed = degrees(90))
       turn_future.result()
       move_future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
       move_future.result()
       last_time = time.time()

def observe_dfs():
    global last_time
    proximity_data = robot.proximity.last_sensor_reading
    if (proximity_data.distance.distance_mm <offset):
         print('前方遇到障碍')
         robot.motors.stop_all_motors()
         last_time = time.time()
         return True
    return False

def get_dir():
    degree = robot.pose.rotation.angle_z.degrees
    if degree <= 45 and degree > -45:
        return 0 # x++ 上
    elif degree <= -45 and degree > -135:
        return 1 # y--右
    elif degree <= 135 and degree > 45:
        return 3 # y++ 左
    elif (degree <= 180 and degree > 135) or (degree >= -180 and degree < -135):
        return 2 # x--下


def pathExist():
    proximity_data = robot.proximity.last_sensor_reading
    if (proximity_data.distance.distance_mm >path_dis):
       return True
    return False


def stack_print():
    for n in stack:
        print(n.pos)

def dir_print(dir):
    if dir == 0:
       return '上'
    elif dir ==1:
       return '右'
    elif dir ==2:
       return '下'
    elif dir ==3:
       return '左'

def check(forward_obstacle):
     pose = robot.pose
     n = node(pose.position.x,pose.position.y,pose.rotation.angle_z.degrees)
     print(n.pos,n.angle_z.degrees)
     if not forward_obstacle:
       robot.motors.stop_all_motors()
       proximity_data = robot.proximity.last_sensor_reading
       #print('当前离障碍距离为',proximity_data.distance.distance_mm)
       if proximity_data.distance.distance_mm < offset:
           n.canGo[get_dir()] = 0
       else:
           n.canGo[get_dir()] = 1     
     count = 0
     right = robot.behavior.turn_in_place(degrees(-90),speed =Angle(degrees=90))
     time.sleep(0.1)
     right.result()
     time.sleep(0.1)
     print('此时朝向右方,机器人方向为',dir_print(get_dir()),robot.pose.rotation.angle_z.degrees)
     if pathExist():
         dir = get_dir()
         n.canGo[dir] = 1 
         print('右方有路,绝对方向为',dir_print(dir))
         count+=1
     left = robot.behavior.turn_in_place(degrees(180),speed =Angle(degrees=90))
     time.sleep(0.1)
     left.result()
     time.sleep(0.1)
     print('此时朝向左方,机器人方向为',dir_print(get_dir()),robot.pose.rotation.angle_z.degrees)
     if pathExist():
         dir = get_dir()
         n.canGo[dir] = 1
         print('左方有路，绝对方向为',dir_print(dir))
         count+=1
     back = robot.behavior.turn_in_place(degrees(-90),speed =Angle(degrees=90))
     back.result()
     if count > 0 :
         n.type=1
         stack.append(n)
         print('该点入栈')
         stack_print()        
     return n

def base_on_canGo_dir(n):
    for i in range(4):
        if n.canGo[i] ==1:
            return i
    return -1

def go_to_node(ori,aim):
    print((aim.x,aim.y),(ori.x,ori.y),'开始节点回溯')
    diffX = aim.x - ori.x
    diffY = aim.y - ori.y
    toDir = -1
    if abs(diffY)>abs(diffX):
       if diffY >0 :
           toDir = 3
       else:
           toDir = 1
    else:
        if diffX >0:
           toDir = 0
        else:
           toDir = 2
    turn_future =robot.behavior.turn_in_place(degrees(-90*(toDir-get_dir())))
    turn_future.result()
    pose = robot.pose
    future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
    future.result()
    while abs(pose.position.x-aim.pos[0])>100 or abs(pose.position.y-aim.pos[1])>100:
        pose = robot.pose
        print(pose.position.x_y_z,'目标为',(aim.x,aim.y))
    print('到达目标节点')
    robot.motors.stop_all_motors()

def move():
    global last_time
    keyboard.wait('a')
    #future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
    #future.result()
    is_go_back = False
    lift_future =robot.behavior.set_lift_height(MAX_LIFT_HEIGHT_MM)
    lift_future.result()
    while True:
      dir_correct()
      forward_obstacle = observe_dfs()
      pose = robot.pose
      if forward_obstacle or time.time() -last_time>3:
          n = check(forward_obstacle)
          while n!= None:
           print('当前节点的canGo状况',n.canGo)           
           pri_dir = base_on_canGo_dir(n)
           if pri_dir !=-1:
             dir = get_dir() 
             future = robot.behavior.turn_in_place(degrees(-90*(pri_dir-dir)),speed =Angle(degrees=90))
             future.result()
             n.canGo[get_dir()] =-1
             if n.type ==1 and is_go_back:
                 stack.append(n)
                 print('该点入栈')
                 stack_print()
             n = None
             is_go_back = False
           else:
             ori = n
             if n==stack[-1]:
                 stack.pop()
             n = stack.pop()
             go_future = robot.behavior.go_to_pose(n.pose)
             go_future.result()
             #go_to_node(ori,n)
             is_go_back = True
             lift_future =robot.behavior.set_lift_height(MAX_LIFT_HEIGHT_MM)
             lift_future.result()
          last_time = time.time()
          future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
          future.result()


   



if __name__ == '__main__':
  robot.connect()
  robot.camera.init_camera_feed()
  move()
