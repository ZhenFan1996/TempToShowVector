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
import random

r1 = '008014c1'
r2 = '0070132a'
robot = anki_vector.AsyncRobot(serial= r1,enable_nav_map_feed= True)
robotH = anki_vector.AsyncRobot(serial= r2,enable_nav_map_feed= True)
speed =  90
##Standard value when detecting an obstacle
# if it is less than this value the robot decides that there is an obstacle ahead
offset = 90
##The standard value for determining access, 
#if it is greater than this value, the robot determines that there is access ahead
path_dis =240
last_time = time.time()
scanning = True


stack = []
turnNodes =[]
isFind = False
start = False
trun_offset =[(0,0),(-25,-25),(-50,0),(-25,25)]



class node():
    def __init__(self):
        """
        class node represent a turing node
        Attributes:
        pose: A boolean indicating if we like SPAM or not.
        pos: An integer count of the eggs we have laid.
        canGo:represents the four directions in which the robot can go
        , with 0 being untravelled, 1 being travelled and -1 being travelled
        type:Type of node, 0 for normal node, 1 for turining point
        dirPose:Set of positions in four directions
        time:Time of node generation
        """
        self.pose = robot.pose 
        self.pos = (self.pose.position.x,self.pose.position.y)
        self.canGo = [0,0,0,0]
        self.type = 0
        self.dirPose = [None] *4
        self.dirPose[get_dir()] = robot.pose
        self.time = time.time()

    def isInAreaNode(self,pose):
        '''
        Determine if the parameter position is within the node
        Args:
        pose:location information
        '''
        x = pose.position.x
        y = pose.position.y
        return  abs(x-self.pos[0])<80 and abs(y-self.pos[1])<80


def isNodeExist(toCheck):
    '''
    Iterate over all turining points in the stack and determine if the input node is a turning point
    '''
    for n in stack:
        if n.isInAreaNode(toCheck.pose)and abs(n.time-toCheck.time)>15:
            return n
    return None


def img_show():
    '''
    The robot's camera collects the images and processes them for recognition.
    '''
    global isFind
    photo = robot.camera.latest_image
    image = photo.raw_image
    img = np.array(image)
    img = img[:,:,::-1].copy()
    temp = './temp.jpg'
    cv2.imwrite(temp,img)
    res,frame = predict_func.run(temp)
    if(len(res)!=0 and res[0]['score']>0.8):
        isFind = True
        print('Found!')
        robot.motors.stop_all_motors()
        robot.behavior.say_text('Win win!')
    cv2.imshow('f',frame)
    key = cv2.waitKey(1)


    
    

def dir_correct():
    '''
    Calibrate the robot's orientation and make sure it goes in a straight line.
    '''
    dir = get_dir()
    degree = robot.pose.rotation.angle_z.degrees
    diff = 0
    de = 0
    if dir==0: # north 0
     diff = degree
     de= 0
    elif dir ==1: #east -90
     diff = degree+90
     de =- 90
    elif dir==3: #west 90 
     diff = degree-90
     de =90
    elif dir ==2: #south 180 or -180
     if(degree<0):
         diff = degree+180
         de = 180
     else:
         diff= degree-180
         de= -180
    if abs(diff) >4:
       robot.motors.stop_all_motors()
       turn_future = robot.behavior.turn_in_place(angle = degrees(de),is_absolute =True,speed = degrees(120))
       turn_future.result()
       move_future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
       move_future.result()
       last_time = time.time()



def observe_dfs():
    '''
    Observe the robot for obstacles in front of it
    '''
    global last_time
    proximity_data = robot.proximity.last_sensor_reading
    list =[]
    for i in range(100):
        proximity_data = robot.proximity.last_sensor_reading
        list.append(proximity_data.distance.distance_mm)
    res = np.mean(list)
    if (res <offset):
         print('obstacles ahead')
         robot.motors.stop_all_motors()
         last_time = time.time()
         return True
    return False

def get_dir():
    '''
    Determining the current direction of the robot
    Returns:
    0 - north, 1 -east, 2-west,3-south
    '''
    list =[]
    for i in range(100):
        list.append(robot.pose.rotation.angle_z.degrees)
    degree = np.mean(list)
    if degree <= 15 and degree >= -15:
        return 0 # x++ north
    elif degree <= -75 and degree > -105:
        return 1 # y--east
    elif degree <= 105 and degree > 75:
        return 3 # y++ south
    elif (degree <= 180 and degree > 165) or (degree >= -180 and degree < -165):
        return 2 # x--west
    else:
        return -1

def degree_dir(degree):
    '''
    Determining the current direction of input degree
    Returns:
    0 - north, 1 -east, 2-west,3-south
    '''
    if degree <= 10 and degree >= -10:
        return 0 # x++ 上
    elif degree <= -80 and degree > -100:
        return 1 # y--右
    elif degree <= 100 and degree > 80:
        return 3 # y++ 左
    elif (degree <= 180 and degree > 170) or (degree >= -180 and degree < -170):
        return 2 # x--下
    else:
        return -1

def pathExist():
    '''
    Detects if there is access ahead
    Returns:
    True: Access road ahead
    False: No Acces ahead
    '''
    list =[]
    for i in range(30):
     proximity_data = robot.proximity.last_sensor_reading
     list.append(proximity_data.distance.distance_mm)
     time.sleep(0.01)
    res = np.mean(list)
    print('distance is',res)
    if (res >path_dis):
       return True
    return False


def stack_print():
    for n in stack:
        print(n.pos,n.canGo)

def dir_print(dir):
    if dir == 0:
       return 'north'
    elif dir ==1:
       return 'east'
    elif dir ==2:
       return 'south'
    elif dir ==3:
       return 'west'

def check(forward_obstacle):
     '''
     Check if the point at the current position is a turining point
     '''
     global scanning
     n = node()
     n.canGo[anti_dir(get_dir())] = -1
     res = isNodeExist(n)
     if res == None:     
      if not forward_obstacle:
       robot.motors.stop_all_motors()
       list =[]
       for i in range(10):
        proximity_data = robot.proximity.last_sensor_reading
        list.append(proximity_data.distance.distance_mm)
        time.sleep(0.001)
       if np.mean(list)< offset+30:
           n.canGo[get_dir()] = 0
       else:
           n.canGo[get_dir()] = 1     
      count = 0
      time.sleep(0.05)
      right = robot.behavior.turn_in_place(degrees(-90),speed =Angle(degrees=120))
      right.result()
      time.sleep(0.05)
      if pathExist():
         dir = get_dir()
         n.canGo[dir] = 1
         n.dirPose[dir] =robot.pose
         count+=1
      time.sleep(0.05)
      left = robot.behavior.turn_in_place(degrees(180),speed =Angle(degrees=120))
      left.result()
      time.sleep(0.1)
      if pathExist():
         dir = get_dir()
         n.canGo[dir] = 1
         n.dirPose[dir] =robot.pose
         count+=1
      time.sleep(0.05)
      back = robot.behavior.turn_in_place(degrees(-90),speed =Angle(degrees=120))
      back.result()
      time.sleep(0.05)
      if count > 0 :
         n.type=1
         if len(stack)>0 and checkRepeat(150,n,stack[-1]):
             stack.pop()
         stack.append(n)
         turnNodes.append(n)
         print('The point is pushed into the stack, and the coordinates are',n.pos)
         stack_print()
     else:
         res.canGo[anti_dir(get_dir())] =-1
         print('Node exist')
         flag = False
         for n in stack:
             for val in n.canGo:
                 if val ==1:
                     flag =True
         if flag:
            return res
         else:
            res.type == 2
            scanning = False
            return res
     return n

def checkRepeat(offset,n1,n2):
    '''
    Check if two nodes are agreeing nodes
    '''
    diffX = abs(n1.pos[0]-n2.pos[0])
    diffY = abs (n1.pos[1]-n2.pos[1])
    if diffX <offset and diffY<offset:
       return True
    return False

def base_on_canGo_dir(n):
    '''
    Determine the most preferred direction to go for this node, or return -1 if none
    '''
    for i in range(4):
        if n.canGo[i] ==1:
            return i
    return -1

def off_get(dir1,dir2):
    diff =dir1-dir2
    if diff > 0:
        pass

def go_to_node(ori,aim):
    '''
    Movement of position according to input
    '''
    print((aim.position.x,aim.position.y),(ori.position.x,ori.position.y),'Start backtracking')
    degreeO = ori.rotation.angle_z.degrees
    degreeA = ori.rotation.angle_z.degrees
    dirO = degree_dir(degreeO)
    dirA = degree_dir(degreeA)
    diffDir = dirA-dirO
    move_offset = 0
    if diffDir<0:
        move_offset = 15
    elif diffDir >0:
        move_offset = -15
    diffX = aim.position.x - ori.position.x + move_offset
    diffY = aim.position.y - ori.position.y + move_offset

    toDir = -1
    distance = 0
    if abs(diffY)>abs(diffX):
       distance = abs(diffY)
       if diffY >0 :
           toDir = 3
       else:
           toDir = 1
    else:
        distance = abs(diffX)
        if diffX >0:
           toDir = 0
        else:
           toDir = 2
    turn_future =robot.behavior.turn_in_place(degrees(-90*(toDir-get_dir())),speed =Angle(degrees=120))
    turn_future.result()
    move_future = robot.behavior.drive_straight(distance_mm(distance),speed_mmps(100))
    move_future.result()
    print('Arrive')
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
    '''
    Traversal logic of the maze, seeking at the end of the traversal
    '''
    global last_time,start,isFind
    seekstart = None
    keyboard.wait('a')
    is_go_back = False
    lift_future =robot.behavior.set_lift_height(MAX_LIFT_HEIGHT_MM)
    lift_future.result()
    while True:
      if isFind:
          return
      start = True
      time.sleep(0.01)
      forward_obstacle = observe_dfs()
      dir_correct()
      if forward_obstacle or time.time() -last_time>3:
          stack_print()
          n = check(forward_obstacle)
          if not scanning :
              seekstart = n
              break
          while n!= None:
           print('Available Direction',n.canGo)           
           pri_dir = base_on_canGo_dir(n)
           if pri_dir !=-1:
             future = robot.behavior.turn_in_place(degrees(-90*(pri_dir-get_dir())),speed =Angle(degrees=120))
             future.result()
             n.canGo[pri_dir] =-1
             if n.type ==1 and is_go_back:
                 stack.append(n)
                 print('The Node is pushed to the stack during backtracking')
                 stack_print()
             n = None
             is_go_back = False
           else:
             if len(stack)==0:
                 break
             if n==stack[-1]and len(stack)>2:
                 stack.pop()
             n = stack.pop()
             go_future = robot.behavior.go_to_pose(n.pose)
             go_future.result()
             time.sleep(0.1)
             lift_future =robot.behavior.set_lift_height(MAX_LIFT_HEIGHT_MM)
             lift_future.result()
             is_go_back = True
          last_time = time.time()
          future = robot.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
          future.result()
          #time.sleep(0.1)
    print('Scanning end')
    robot.motors.stop_all_motors()
    keyboard.wait('c')
    print('start to seek')
    seek(seekstart)


def detect():
    '''
    Targeting at short intervals
    '''
    t = time.time()
    while True:
      if isFind:
         return
      if time.time()-t> 0.6:
           img_show()




def seek(start):
    '''
    seek logic
    '''
    startIndex = -1
    for i in range(len(turnNodes)):
        if(turnNodes[i]==start):
            startIndex =i
            break
    curr = startIndex
    while True:
       if isFind:
           return
       search(turnNodes[curr])      
       if curr != len(turnNodes)-1:
           #go_to_node(turnNodes[curr],turnNodes[curr+1])
           move_future = robot.behavior.go_to_pose(turnNodes[curr+1].pose)
           move_future.result()
           curr +=1
       else:
           move_future = robot.behavior.go_to_pose(turnNodes[0].pose)
           move_future.result()
           curr =0



def search(node):
    '''
    Steer and watch for the node's canGo property
    '''
    ori_dir = get_dir()
    for i in range(len(node.canGo)):
        if node.canGo[i] ==1 or node.canGo[i] ==-1 and i != anti_dir(ori_dir):
           future = robot.behavior.turn_in_place(degrees(-90*(i-get_dir())),speed =Angle(degrees=120))
           future.result()
           time.sleep(1)

def hide():
    '''
    logic for hider
    '''
    keyboard.wait('a')
    future = robotH.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
    future.result()
    while True:
        dir_correctH()
        print(robotH.proximity.last_sensor_reading.distance.distance_mm )
        if robotH.proximity.last_sensor_reading.distance.distance_mm < 150:
            robotH.motors.stop_all_motors()
            turn_future = robotH.behavior.turn_in_place(degrees(-90),speed =Angle(degrees = 120))
            turn_future.result()
            future = robotH.motors.set_wheel_motors(speed,speed,speed*4,speed*4)
            future.result()


def see_Vector():
    global isFind
    while True:
       if isFind:
          robot.motors.stop_all_motors()
          robot.behavior.say_text('Win win!')
          return




if __name__ == '__main__':
  robot.connect()
  robotH.connect()
  robot.camera.init_camera_feed()
  detect_Thread = threading.Thread(target = detect,args =[])
  detect_Thread.start()
  seek_Thread = threading.Thread(target = move,args =[])
  seek_Thread.start()
  hide_Thread = threading.Thread(target = hide,args =[])
  hide_Thread.start()
