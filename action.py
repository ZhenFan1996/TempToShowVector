import anki_vector
import cv2
import numpy as np
from PIL import Image
import os
import environment as env
import threading
import time
from anki_vector.util import degrees, distance_mm, speed_mmps,Angle,Pose
from collections import deque
import asyncio
import random
import detect
import predict_func


event = threading.Event()
path = [(0,0)]
lastX ,lastY = 0,0
last = (0,0)
count =0
nodes = []
stack =[]
lastNode = None


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



class node():
    def __init__(self,x,y,angle_z):
        self.x =x
        self.y =y
        self.canGo = [0,0,0,0]
        self.pathDic = {}
        self.type = 0
        self.lastNodes = [None]*4
        self.pose = Pose(x=x,y=y,z=0,angle_z=angle_z)

    def type_set(type):
        self.type = type

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False

    def __hash__(self):
        return hash(self.item)


def robot_init(robot):
      last_time = time.time()
      while True:
        proximity_data = robot.proximity.last_sensor_reading
        #img_show(robot)
        pos = (robot.pose.position.x,robot.pose.position.y)
        position_transform(pos,50)
        #env.update(path)
       

def scanner(robot):
  robot.motors.set_wheel_motors(100,100)
  while True:   
     wall = observe(robot)
     if wall is False:
       robot.motors.set_wheel_motors(100, 100)
      
def scanner_dir(robot):
  global last
  robot.motors.set_wheel_motors(50,50)
  while True:
     dir = get_dir(robot)
     diff =0
     if dir == 0:     
        diff = robot.pose.position.x - last[0]
     elif dir ==1:
        diff = robot.pose.position.y - last[1]
     elif dir ==2:
        diff = last[1] - robot.pose.position.y
     elif dir ==3:
        diff =  last[0] - robot.pose.position.x
     if diff > 100:
        robot.motors.stop_all_motors()
        robot.behavior.turn_in_place(degrees(-90))
        proximity_data = robot.proximity.last_sensor_reading
        if(proximity_data.distance.distance_mm < 100):
            robot.behavior.turn_in_place(degrees(90))
            robot.motors.set_wheel_motors(50,50)
        else:
            robot.motors.set_wheel_motors(50,50)
     last = (robot.pose.position.x,robot.pose.position.y)
     print(diff)
     

def scanner_time(robot):
    last_time = time.time()
    while True:
        dir_correct(robot)
        observe_test(robot)
        if time.time()-last_time >2.0:
           robot.motors.stop_all_motors()
           robot.behavior.turn_in_place(degrees(90))
           print('4s later, turn left')
           print(robot.proximity.last_sensor_reading.distance.distance_mm)
           if(robot.proximity.last_sensor_reading.distance.distance_mm<50):
              pos = (robot.pose.position.x,robot.pose.position.y)
              new = (int(pos[0] // 50),-int(pos[1] // 50))
              env.obstacle_update(new,get_dir(robot))
              robot.behavior.turn_in_place(degrees(-90))
              print('obstacle,turn right')
              robot.motors.set_wheel_motors(50,50)
           else:
              robot.motors.set_wheel_motors(50,50)
           last_time = time.time()
        


def get_dir(robot):
    degree = robot.pose.rotation.angle_z.degrees
    if degree <= 30 and degree >= -30:
        return 0 # x++ 上
    elif degree <= -60 and degree >= -120:
        return 1 # y--右
    elif degree <= 120 and degree >= 60:
        return 3 # y++ 左
    elif (degree <= 180 and degree >= 160) or (degree >= -180 and degree <= -160):
        return 2 # x--下
    else:
        return -1


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


def observe_test(robot):
    proximity_data = robot.proximity.last_sensor_reading
    if(proximity_data.distance.distance_mm <50):
        pos = (robot.pose.position.x,robot.pose.position.y)
        new = (int(pos[0] // 50),-int(pos[1] // 50))
        env.obstacle_update(new,get_dir(robot))
        robot.motors.stop_all_motors()
        #print('obstacle find,trun right')
        robot.behavior.turn_in_place(degrees(-90))
        robot.motors.set_wheel_motors(50,50)



def img_show(robot):
    global count
    photo = robot.camera.latest_image
    image = photo.raw_image
    img = np.array(image)
    img = img[:,:,::-1].copy()
    temp = './temp.jpg'
    cv2.imwrite(temp,img)
    res,frame = predict_func.run(temp)
    print(res)
    cv2.imshow('f',frame)
    key = cv2.waitKey(1)



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


## 得到angel，如果偏差太大，矫正到本位
def dir_correct(robot):
    dir = get_dir(robot)
    degree = robot.pose.rotation.angle_z.degrees
    diff = 0
    if dir==0: # 上 0
     diff = degree
    elif dir ==1: #右 -90
     diff = degree+90
    elif dir==3: #左 90 
     diff = degree-90
    elif dir ==2: #下 180或-180
     if(degree<0):
         diff = degree+180
     else:
         diff= degree-180
    if abs(diff) >1.5:
       robot.motors.stop_all_motors()
       robot.behavior.turn_in_place(degrees(-diff))
       robot.motors.set_wheel_motors(60,60)

## 获取当前机器人的基本信息，返回信息第一项为坐标，第二项为朝向，第三项为当前角度
def getInfo(robot):
    pos =(robot.pose.position.x,robot.pose.position.y)
    dir = get_dir(robot)
    degree = robot.pose.rotation.angle_z.degrees
    return pos,dir,degree


def dir_plus(dir):
    if dir <3:
        return dir+1
    else:
        return 0

def dir_print(dir):
    if dir == 0:
       return '上'
    elif dir ==1:
       return '右'
    elif dir ==2:
       return '下'
    elif dir ==3:
       return '左'

def findroad(robot,dir,node):
    proximity_data = robot.proximity.last_sensor_reading
    if(proximity_data.distance.distance_mm > 100):
        node.canGo[dir] = 1 # 1代表前路可走
    robot.behavior.turn_in_place(degrees(-90))
    dir = dir_plus(dir)
    count =0 
    for i in range(3):
      if i !=1:
        proximity_data = robot.proximity.last_sensor_reading
        if(proximity_data.distance.distance_mm > 200):
           node.canGo[dir] = 1 # 1代表前路可走
           print('机器人' + dir_print(dir)+ '方有路')
           count +=1
      else:
        node.canGo[dir] = -1 ## 表示已经走过了的通路
      robot.behavior.turn_in_place(degrees(-90))
      dir = dir_plus(dir)
    if (count >0):
          node.type =1 # 表示该点为特殊拐点
          print('发现拐点，并且拐点入栈')
          stack.append(node)

def anti_dir(dir):
    if dir ==0:
        return 2
    elif dir ==1:
        return 3
    elif dir ==2:
        return 0
    elif dir ==3:
        return 1
    return -1




def observe_dfs(pos,dir,degree):
    proximity_data = robot.proximity.last_sensor_reading
    if(proximity_data.distance.distance_mm <100):
        print('检测到障碍')
        robot.motors.stop_all_motors()
        n = node(pos[0],pos[1],Angle(degrees = degree))
        n.type =1 #遇到障碍进行转弯的点为特殊拐点
        n.canGo[dir] = 0
        n.canGo[anti_dir(dir)] =-1 # 表示反方向是已经走过的通路
        robot.behavior.turn_in_place(degrees(-90)) # 右转
        proximity_data = robot.proximity.last_sensor_reading
        if(proximity_data.distance.distance_mm >200):
            print(dir_plus(dir))
            n.canGo[dir_plus(dir)] = 1 # 表示右侧可以走
            print('右侧可以走')
        robot.behavior.turn_in_place(degrees(180)) ## 反方向180度转到左侧
        proximity_data = robot.proximity.last_sensor_reading
        if(proximity_data.distance.distance_mm >200):
            print(anti_dir(dir_plus(dir)))
            n.canGo[anti_dir(dir_plus(dir))] =1 # 表示左侧可以走
            print('左侧可以走')
        robot.behavior.turn_in_place(degrees(-90)) # 转回原来的朝向
        if(len(nodes)!=0):
            n.lastNodes[anti_dir(dir)] = nodes[-1]
            nodes[-1].lastNodes[dir] =n
        print(n.canGo)
        nodes.append(n)
        stack.append(n)
        return n
    return None


def priority_find(node):
    for i in range(4):
        if(node.canGo[i]==1):
            return i
    return -1


def dfs(robot):
    last_time = time.time()
    while True:
        pos,dir,degree = getInfo(robot)
        #dir_correct(robot)
        print(degree)
        n = observe_dfs(pos,dir,degree)
        while n!=None:
            last_time = time.time()
            pri_dir = priority_find(n)
            print(pri_dir)
            if pri_dir !=-1:
                n = None
                robot.behavior.turn_in_place(degrees(-90*(pri_dir-dir)))  # 转到目标方向
                print('转到'+ dir_print(pri_dir) )
                robot.motors.set_wheel_motors(62,60)
            else:
                n = n.lastNodes[anti_dir(get_dir(robot))]
                robot.behavior.go_to_pose(n.pose)
        if time.time()-last_time >4.0:
           print(degree)
           robot.motors.stop_all_motors()           
           n = node (pos[0],pos[1],Angle(degrees=degree))
           findroad(robot,dir,n)
           if(len(nodes)!=0):
               n.lastNodes[anti_dir(dir)] = nodes[-1]
               nodes[-1].lastNodes[dir] =n
           nodes.append(n)        
           robot.motors.set_wheel_motors(62,60)
           last_time = time.time()





def initialize():
	robot = anki_vector.Robot(serial= '008014c1')
	robot.connect()
	robot.camera.init_camera_feed()
	robot.behavior.set_head_angle(degrees(0))
	return robot




if __name__ == '__main__':
  robot = initialize()
  env_thread = env_Thread(event)
  robot_Thread = robot_Thread(robot)
  go_Thread = threading.Thread(target= dfs,args =[robot])
  #observe_Thread = threading.Thread(target =observe_test,args =[robot])
  #env_thread.start()
  #robot_Thread.start()
  go_Thread.start()
  #observe_Thread.start()



        
        
