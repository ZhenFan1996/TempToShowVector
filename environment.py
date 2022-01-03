import pygame 
import os
import numpy as np
import random
import sys
import threading
import time

BLACK = np.array((0,0,0))
WHITE = np.array((200,200,200))
W = np.array((255,255,255))
GREY = np.array((220,220,220))
RED = np.array((255,0,0))
window_height = 600
window_width = 600
matz = np.zeros([window_height // 10,window_width // 10])
blockSize =30
flag = False
path = []


def main():
    global window, clock
    pygame.init()
    window = pygame.display.set_mode((window_height, window_width))
    clock = pygame.time.Clock()
    window.fill(GREY)
    drawGrid()
    ori = (0,0)       
    while True:    
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        pygame.display.flip()
        aim = randomRun(ori)
        run(convertPos(ori),convertPos(aim))
        ori = aim
        pygame.display.flip()
        clock.tick(5)

def init(eventT):
    global window, clock,ori,flag
    pygame.init()
    window = pygame.display.set_mode((window_height, window_width))
    clock = pygame.time.Clock()
    window.fill(GREY)
    drawGrid()
    while True:       
        MOVE_EVENT = pygame.USEREVENT + 1
        if flag == True:
          move_event = pygame.event.Event(MOVE_EVENT,{"path": path})
          pygame.event.post(move_event)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == MOVE_EVENT:
                path_draw(path)
                flag = False
        pygame.display.flip()
       
        
    

def drawGrid():
    for x in range(window_width):
        for y in range(window_height):
            rect = pygame.Rect(x * blockSize, y * blockSize,
                               blockSize, blockSize)
            pygame.draw.rect(window, WHITE, rect, 1)


def run(ori,aim):
     orie = pygame.Rect((ori[0]) * blockSize, (ori[1]) * blockSize,
                               blockSize, blockSize)
     rect1 = pygame.Rect((ori[0]) * blockSize, (ori[1]) * blockSize,
                               blockSize, blockSize)
     aime = pygame.Rect((aim[0]) * blockSize, (aim[1]) * blockSize,
                               blockSize, blockSize)
     rect2 = pygame.Rect((aim[0]) * blockSize, (aim[1]) * blockSize,
                               blockSize, blockSize)
     pygame.draw.rect(window,W,orie)
     pygame.draw.rect(window,RED,aime)
     pygame.draw.rect(window, WHITE, rect2, 1)
     pygame.draw.rect(window, WHITE, rect1, 1)


def update(path_Gived):
    global flag,path
    path = path_Gived
    flag = True

def path_draw(path_list):
    ori = (window_width // blockSize,0)
    for position in path_list:
     run(convertPos(ori),convertPos(position))
     ori = position
    pygame.display.flip()
    clock.tick(5)

def point_draw(path_list):
    if(len(path_list)==1):
        run((0,0),(0,0))
    else:
       run(path_list[-2],path_list[-1])
    pygame.display.flip()
    clock.tick(5)


def randomRun(ori):
    if(ori[0] == 0):
        if ori[1] == 0:
           direction = random.choice([1,3])
        elif ori[1] == window_width:
           direction = random.choice([1,2])
        else:
           direction = random.choice([1,2,3])
    elif(ori[0] == window_height):
        if ori[1] == 0:
           direction = random.choice([0,3])
        elif ori[1] == window_width:
           direction = random.choice([0,2])
        else:
           direction = random.choice([0,2,3])
    elif(ori[1] == 0):
        if ori[0] == 0:
           direction = random.choice([1,3])
        elif ori[0] == window_width:
           direction = random.choice([0,3])
        else:
           direction = random.choice([0,1,3])
    elif(ori[1] == window_width):
        if ori[0] == 0:
           direction = random.choice([1,2])
        elif ori[0] == window_width:
           direction = random.choice([0,2])
        else:
           direction = random.choice([0,1,2])
    else:
        direction = random.choice([0,1,2,3])

    if direction == 0:
        return [ori[0] - 1,ori[1]]
    elif direction == 1:
        return [ori[0] + 1,ori[1]]
    elif direction == 2:
        return [ori[0],ori[1] - 1]
    elif direction == 3:
        return [ori[0],ori[1] + 1]

def convertPos(pos):
    ans = [0,0]
    ans[0]= pos[1] + 4
    ans[1] =window_width // blockSize - pos[0] -4
    return ans

if __name__ == "__main__":
    main()





