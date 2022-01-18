import cv2
import numpy as np
import random
import os

def drawContours(winName,image,contours,draw_on_blank):
    if(draw_on_blank):
        temp = np.ones(image.shape,dtype = np.uint8)*255
        cv2.drawContours(temp,contours,-1,(0,0,0),2)
    else:
        temp = image.copy()
        cv2.drawContours(temp,contours,-1,(0,0,255),2)
    cv2.imshow(winName,temp)




#cv2.imshow('img',img)
def imgConvert(img,maxval):
  gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  gray = cv2.GaussianBlur(gray,(5,5),1)
  ret,binary = cv2.threshold(gray,maxval,255,cv2.THRESH_BINARY)
  kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
  binary = cv2.morphologyEx(binary,cv2.MORPH_RECT,(5,5),kernel)
#  cv2.imshow('morph',binary)
  erode = cv2.erode(binary,kernel)
  #cv2.imshow('erode',erode)
  dilate = cv2.dilate(erode,kernel)
 # cv2.imshow('dilate',dilate)

  close = cv2.morphologyEx(dilate,cv2.MORPH_CLOSE,kernel)

  #cv2.imshow('close',close)
  contours,hierarchy = cv2.findContours(close,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

  drawContours('find',img,contours,False)
  return contours
  #i =0
  #while True:
  # temp = np.ones(img.shape,dtype = np.uint8)*255
  # cv2.drawContours(temp,contours[i:i+1],-1,(0,0,0),2)
  # cv2.imshow('temp',temp)
  # key = cv2.waitKey(0)
  # if key&0xFF ==ord('q'):
  #     i+=1
  # elif key&0xFF == ord('s'):
  #     rand = random.random()
  #     cv2.imwrite('./contours/'+str(rand)+'.jpg',temp)

def getContour():
  contour = cv2.imread('./contours/3.jpg')
  gray = cv2.cvtColor(contour,cv2.COLOR_BGR2GRAY)
  ret,binary = cv2.threshold(gray,200,255,cv2.THRESH_BINARY)
  contours,hierarchy = cv2.findContours(binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  drawContours('c',contour,contours[1:2],True)
  img = cv2.imread('./image/5.jpg')
  test = imgConvert(img,60)
  drawContours('find1',contour,test[18:19],True)

  print(cv2.matchShapes(contours[1],test[18],1,0.0))
  cv2.waitKey(0)


def contour_get(cImg):
    gray = cv2.cvtColor(cImg,cv2.COLOR_BGR2GRAY)
    ret,binary = cv2.threshold(gray,200,255,cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours[1]

def find(img,maxval):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(5,5),1)
    ret,binary = cv2.threshold(gray,maxval,255,cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    binary = cv2.morphologyEx(binary,cv2.MORPH_RECT,(5,5),kernel)
    erode = cv2.erode(binary,kernel)
    dilate = cv2.dilate(erode,kernel)
    close = cv2.morphologyEx(dilate,cv2.MORPH_CLOSE,kernel)
    contours,hierarchy = cv2.findContours(close,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours

def cal():
  min = 10000
  max = -1
  for filename in os.listdir('./contours'):
     cImg = cv2.imread('./contours'+'/'+filename)
     c =contour_get(cImg)
     area = cv2.contourArea(c)
     if(area<min):
         min = area
     if(area>max):
         max = area
  return min,max

def compare(img):
    min_area ,max_area= cal()
    similar = ''
    res = None
    min_val_global = 2
    min_pos_global = -1
    for filename in os.listdir('./contours'):
     for val in[60]:
        cImg = cv2.imread('./contours'+'/'+filename)
        contourTotest= contour_get(cImg)
        contours = find(img,val)
        min_val =  2
        min_pos = -1
        for i in range(len(contours)):
            value = cv2.matchShapes(contourTotest,contours[i],1,0.0)
            if (value < min_val) and (cv2.contourArea(contours[i])>min_area)and(cv2.contourArea(contours[i])<=max_area):
                min_val = value
                min_pos =i
        if min_val <min_val_global:
            res = contours[min_pos]
            similar = filename
            min_val_global = min_val
            min_pos_global = min_pos          
    print(similar,min_val_global,min_pos_global)
    return similar,min_val_global,min_pos_global,res

#img = cv2.imread('./image/29.jpg')
#s,v,p,r=compare(img)
#res= cv2.imread('./contours/'+ s )
#x,y,w,h = cv2.boundingRect(r)
#cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
#cv2.imshow('res',res)
#cv2.imshow('img',img)
#temp = np.ones(img.shape,dtype = np.uint8)*255
#drawContours('find1',img,r,True)
#cv2.waitKey(0)





