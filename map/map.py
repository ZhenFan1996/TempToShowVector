import matplotlib.pyplot as plt
import matplotlib.animation as ani
import numpy as np
from matplotlib.patches import Rectangle



class Map:
    def __init__(self,size=1000):
        self.ax = plt.gca()
        self.ax.set_xlim([0,size])
        self.ax.set_ylim([0,size])
        self.size =size
        self.matz = np.zeros((size,size))
        plt.axis('equal') 
        plt.axis('off')


    def update(self,x,y,status):
        if(status == 1):
           rec = Rectangle((x, y), width=1, height=1, color='white')
        elif(status == -1):
           rec = Rectangle((x, y), width=1, height=1, color='black')
        self.ax.add_patch(rec)

    def add_obstacle(self,x,y):
        self.matz[x,y] = -1
        self.update(x,y,-1)

    def add_path(self,x,y):
        self.matz[x,y] =1
        self.update(x,y,0)

