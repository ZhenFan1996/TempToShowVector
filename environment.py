import pygame 
import os
import numpy as np
import random
import sys 

BLACK = np.array((0,0,0))
WHITE = np.array((200,200,200))
W = np.array((255,255,255))
GREY= np.array((220,220,220))
RED = np.array((255,0,0))
window_height = 600
window_width = 600
matz = np.zeros([window_height//10,window_width//10])


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
        aim =randomRun(ori)
        run(ori,aim)
        ori = aim
        pygame.display.flip()
        clock.tick(80)

      


def drawGrid():
    blockSize = 10 
    for x in range(window_width):
        for y in range(window_height):
            rect = pygame.Rect(x*blockSize, y*blockSize,
                               blockSize, blockSize)
            pygame.draw.rect(window, WHITE, rect, 1)


def run(ori,aim):
     blockSize = 10 
     orie = pygame.Rect(ori[0]*blockSize, ori[1]*blockSize,
                               blockSize, blockSize)
     rect1 = pygame.Rect(ori[0]*blockSize, ori[1]*blockSize,
                               blockSize, blockSize)
     aime = pygame.Rect(aim[0]*blockSize, aim[1]*blockSize,
                               blockSize, blockSize)
     rect2 = pygame.Rect(aim[0]*blockSize, aim[0]*blockSize,
                               blockSize, blockSize)
     pygame.draw.rect(window,W,orie)
     pygame.draw.rect(window,RED,aime)
     pygame.draw.rect(window, WHITE, rect2, 1)
     pygame.draw.rect(window, WHITE, rect1, 1)
   

def randomRun(ori):
    if(ori[0]==0):
        if ori[1] ==0:
           direction = random.choice([1,3])
        elif ori[1] == window_width:
           direction = random.choice([1,2])
        else:
           direction = random.choice([1,2,3])
    elif(ori[0]== window_height):
        if ori[1] ==0:
           direction = random.choice([0,3])
        elif ori[1] == window_width:
           direction = random.choice([0,2])
        else:
           direction = random.choice([0,2,3])
    elif(ori[1]==0):
        if ori[0] ==0:
           direction = random.choice([1,3])
        elif ori[0] == window_width:
           direction = random.choice([0,3])
        else:
           direction = random.choice([0,1,3])
    elif(ori[1]==window_width):
        if ori[0] ==0:
           direction = random.choice([1,2])
        elif ori[0] == window_width:
           direction = random.choice([0,2])
        else:
           direction = random.choice([0,1,2])
    else:
        direction = random.choice([0,1,2,3])

    if direction == 0:
        return [ori[0]-1,ori[1]]
    elif direction == 1:
        return [ori[0]+1,ori[1]]
    elif direction == 2:
        return [ori[0],ori[1]-1]
    elif direction == 3:
        return [ori[0],ori[1]+1]

if __name__ == "__main__":
    main()





