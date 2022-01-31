import anki_vector
import cv2
import numpy as np
import threading
import time
from anki_vector.util import degrees, distance_mm, speed_mmps,Angle,Pose
from anki_vector.behavior import MIN_LIFT_HEIGHT_MM, MAX_LIFT_HEIGHT_MM
import asyncio
import keyboard
import predict_func
import numpy as np

r1 = '008014c1'
r2 = '0070132a'
robot = anki_vector.Robot(serial= r1,enable_nav_map_feed= True)
speed =  90
offset = 100
path_dis =300
last_time = time.time()

stack = []
res =[]
isFind = False
start = False

class node():
    def __init__(self):
        self.pose = robot.pose
        self.pos = (self.pose.position.x,self.pose.position.y)
        self.canGo = [0,0,0,0]
        self.type = 0
        self.dirPose = [None] *4
        self.dirPose[get_dir()] = robot.pose
        self.time = time.time()

    def isInAreaNode(self,pose):
        x = pose.position.x
        y = pose.position.y
        return  abs(x-self.pos[0])<100 and abs(y-self.pos[1])<100


class custom():
    def __init__(self,x,y,c):
        self.pos =(x,y)
        self.canGo = c

def isNodeExist(toCheck):
    for n in stack:
        if n.isInAreaNode(toCheck.pose)and abs(n.time-toCheck.time)>15:
            return n
    return None


def img_show():
    global isFind
    photo = robot.camera.latest_image
    image = photo.raw_image
    img = np.array(image)
    img = img[:,:,::-1].copy()
    temp = './temp.jpg'
    cv2.imwrite(temp,img)
    res,frame = predict_func.run(temp)
    #print(res)
    if(len(res)!=0):
        isFind = True
        print('找到了')
    cv2.imshow('f',frame)
    key = cv2.waitKey(1)

def stop(go_Thread):
    while True:
      if isFind:
          go_Thread.join()
          print('结束了')
          break


def getNum(list):
    if len(list) == 1:
        return list[0]
    list.sort()
    listteil = list[1:]
    return np.mean(listteil)
    
    

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
       turn_future = robot.behavior.turn_in_place(angle = degrees(de),is_absolute =True)
       #turn_future.result()
       move_future = robot.motors.set_wheel_motors(speed,speed)
       #move_future.result()
       last_time = time.time()

def observe_dfs():
    global last_time
    proximity_data = robot.proximity.last_sensor_reading
    #list =[]
    #for i in range(100):
    #    proximity_data = robot.proximity.last_sensor_reading
    #    list.append(proximity_data.distance.distance_mm)
    #res = np.mean(list)
    if (robot.proximity.last_sensor_reading.distance.distance_mm <offset):
         print('前方遇到障碍')
         robot.motors.stop_all_motors()
         last_time = time.time()
         return True
    return False

def get_dir():
    #list =[]
    #for i in range(20):
    #    list.append(robot.pose.rotation.angle_z.degrees)
    degree = robot.pose.rotation.angle_z.degrees
    if degree <= 10 and degree >= -10:
        return 0 # x++ 上
    elif degree <= -80 and degree > -100:
        return 1 # y--右
    elif degree <= 100 and degree > 80:
        return 3 # y++ 左
    elif (degree <= 180 and degree > 170) or (degree >= -180 and degree < -170):
        return 2 # x--下
    else:
        return 0


def pathExist():
    #list =[]
    #for i in range(30):
    # proximity_data = robot.proximity.last_sensor_reading
    # list.append(proximity_data.distance.distance_mm)
    # #time.sleep(0.01)
    #res = np.mean(list)
    #print('前方距离',res)
    if (robot.proximity.last_sensor_reading.distance.distance_mm >path_dis):
       return True
    return False


def stack_print():
    for n in stack:
        print(n.pos,n.canGo)

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
     n = node()
     n.canGo[anti_dir(get_dir())] = -1
     res = isNodeExist(n)
     if res == None:     
      if not forward_obstacle:
       robot.motors.stop_all_motors()
       #list =[]
       #for i in range(10):
       # proximity_data = robot.proximity.last_sensor_reading
       # list.append(proximity_data.distance.distance_mm)
        #time.sleep(0.001)
       #print('当前离障碍距离为',proximity_data.distance.distance_mm)
       if robot.proximity.last_sensor_reading.distance.distance_mm< offset:
           n.canGo[get_dir()] = 0
       else:
           n.canGo[get_dir()] = 1     
      count = 0
      #time.sleep(0.05)
      right = robot.behavior.turn_in_place(degrees(-90))
      #right.result()
      #time.sleep(0.05)
     #print('此时朝向右方,机器人方向为',dir_print(get_dir()),robot.pose.rotation.angle_z.degrees)
      if pathExist():
         dir = get_dir()
         n.canGo[dir] = 1
         n.dirPose[dir] =robot.pose
         #print('右方有路,绝对方向为',dir_print(dir))
         count+=1
      #time.sleep(0.05)
      left = robot.behavior.turn_in_place(degrees(180))
      #left.result()
      #time.sleep(0.1)
     #print('此时朝向左方,机器人方向为',dir_print(get_dir()),robot.pose.rotation.angle_z.degrees)
      if pathExist():
         dir = get_dir()
         n.canGo[dir] = 1
         n.dirPose[dir] =robot.pose
         #print('左方有路，绝对方向为',dir_print(dir))
         count+=1
      #time.sleep(0.05)
      back = robot.behavior.turn_in_place(degrees(-90))
      #back.result()
      #time.sleep(0.05)
      if count > 0 :
         n.type=1
         if len(stack)>0 and checkRepeat(150,n,stack[-1]):
             stack.pop()
             #print('重复了，抛出老点')
         stack.append(n)
         print('该点入栈,坐标为',n.pos)
         stack_print()
     else:
         res.canGo[anti_dir(get_dir())] =-1
         print('存在已经有的节点')
         return res
     return n

def checkRepeat(offset,n1,n2):
    diffX = abs(n1.pos[0]-n2.pos[0])
    diffY = abs (n1.pos[1]-n2.pos[1])
    if diffX <offset and diffY<offset:
       return True
    return False

def base_on_canGo_dir(n):
    for i in range(4):
        if n.canGo[i] ==1:
            return i
    return -1

def go_to_node(ori,aim):
    print('开始回溯')
    print((aim.position.x,aim.position.y),(ori.position.x,ori.position.y),'开始节点回溯')
    diffX = aim.position.x - ori.position.x
    diffY = aim.position.y - ori.position.y
    toDir = -1
    distance = 0
    if abs(diffY)>abs(diffX):
       distance = abs(diffY)-20
       if diffY >0 :
           toDir = 3
       else:
           toDir = 1
    else:
        distance = abs(diffX)-20
        if diffX >0:
           toDir = 0
        else:
           toDir = 2
    turn_future =robot.behavior.turn_in_place(degrees(-90*(toDir-get_dir())),speed =Angle(degrees=120))
    #turn_future.result()
    move_future = robot.behavior.drive_straight(distance_mm(distance),speed_mmps(100))
    #move_future.result()
    print('到达目标节点')
    robot.motors.stop_all_motors()

def status_print():
    for n in stack :
        for pose in n.dirPose:
            if pose!=None:
              print(pose.position.x_y_z)


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

def move():
    global last_time,start
    keyboard.wait('a')
    #future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
    #future.result()
    is_go_back = False
    lift_future =robot.behavior.set_lift_height(MAX_LIFT_HEIGHT_MM)
    #lift_future.result()
    while True:
      if(start and len(stack)==0):
          break
      start = True
      #time.sleep(0.01)
      forward_obstacle = observe_dfs()
      dir_correct()
      if forward_obstacle or time.time() -last_time>3:
          stack_print()
          n = check(forward_obstacle)
          while n!= None:
           print('当前节点的canGo状况',n.canGo)           
           pri_dir = base_on_canGo_dir(n)
           if pri_dir !=-1:
             future = robot.behavior.turn_in_place(degrees(-90*(pri_dir-get_dir())))
             #future.result()
             n.canGo[pri_dir] =-1
             if n.type ==1 and is_go_back:
                 stack.append(n)
                 print('该点在回溯过程中入栈')
                 stack_print()
             n = None
             is_go_back = False
           else:
             if len(stack)==0:
                 break
             if n==stack[-1]and len(stack)>2:
                 stack.pop()
             n = stack.pop()
             pri_dir = base_on_canGo_dir(n)
             if pri_dir!=-1:
              print('当前方向为',dir_print(get_dir()))
              go_to_node(robot.pose,n.dirPose[pri_dir])
             else:
              print(n.pos)
              #go_future = robot.behavior.go_to_pose(n.pose)
              #go_future.result()
              go_to_node(robot.pose,n.pose)
             is_go_back = True
             #lift_future =robot.behavior.set_lift_height(MAX_LIFT_HEIGHT_MM)
             #lift_future.result()
          last_time = time.time()
          future = robot.motors.set_wheel_motors(speed,speed)
          #future.result()
          #time.sleep(0.1)
    robot.disconnect()
    print('遍历结束')


def detect():  
    while True:
        img_show()

test_data = [custom(0,0,[1,1,0,0])
      ,custom(989.5,-14.1,[0,1,1,0])
      ,custom(1047.2,-705.2,[0,0,1,1])
      ,custom(30.2,-629.9,[1,1,0,1])
      ,custom(53.3,-914.7,[1,0,0,1])
      ,custom(40.8,-321.8,[1,1,0,1])]

def seek(test_data):
    #pose = robot.pose
    #for n in nl:
    #    if n.isInAreaNode():
    #        start =n
    #        break
    keyboard.wait('a')
    for i in range(len(nl)):
        search(nl[i])      
        if i != len(nl)-1:
           go_to_node2(nl[i],nl[i+1])
        else:
           go_to_node2(nl[i],nl[0])

def go_to_node2(ori,aim):
    print('开始回溯')
    print((aim.pos[0],aim.pos[1]),(ori.pos[0],ori.pos[1]),'开始移动')
    diffX = aim.pos[0] - ori.pos[0]
    diffY = aim.pos[1] - ori.pos[1]
    toDir = -1
    distance = 0
    if abs(diffY)>abs(diffX):
       distance = abs(diffY)-20
       if diffY >0 :
           toDir = 3
       else:
           toDir = 1
    else:
        distance = abs(diffX)-20
        if diffX >0:
           toDir = 0
        else:
           toDir = 2
    print('目标方向是',dir_print(toDir))
    turn_future =robot.behavior.turn_in_place(degrees(-90*(toDir-get_dir())),speed =Angle(degrees=120))
    #turn_future.result()
    move_future = robot.behavior.drive_straight(distance_mm(distance),speed_mmps(100))
    #move_future.result()
    print('到达目标节点')
    robot.motors.stop_all_motors()

def search(node):
    ori_dir = get_dir()
    for i in range(len(node.canGo)):
        if node.canGo[i] ==1 and i != anti_dir(ori_dir):
           future = robot.behavior.turn_in_place(degrees(-90*(i-get_dir())),speed =Angle(degrees=120))
           future.result()
           time.sleep(3)


def test():
    future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
    future.result()
    while True:
      proximity_data = robot.proximity.last_sensor_reading
      print(proximity_data.distance.distance_mm)
      if (proximity_data.distance.distance_mm<100):
          robot.motors.stop_all_motors()
          right = robot.behavior.turn_in_place(degrees(-90),speed =Angle(degrees=90))
          right.result()
          future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
          future.result()




if __name__ == '__main__':
  robot.connect()
  robot.camera.init_camera_feed()
  #detect_Thread = threading.Thread(target = detect,args =[])
  #detect_Thread.start()
  #seek_Thread = threading.Thread(target = seek,args =[nl])
  #stop_Thread = threading.Thread(target = stop,args =[seek_Thread])
  #seek_Thread.start()
  #stop_Thread.start()
  move()
