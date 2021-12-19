import anki_vector
import cv2
import numpy as np
from PIL import Image
import os
from anki_vector.util import degrees
import environment as env


def turn_around(robot):
    for i in range(4):
        robot.behavior.turn_in_place(degrees(90*i))


with anki_vector.AsyncRobot(enable_nav_map_feed=True) as robot:
    robot.camera.init_camera_feed()
    #latest_nav_map = robot.nav_map.latest_nav_map
    #robot.motors.set_wheel_motors(50, 50)
    while True:
        proximity_data = robot.proximity.last_sensor_reading
        if proximity_data is not None:
           print('Proximity distance: {0}'.format(proximity_data.distance))
        photo = robot.camera.latest_image
        image = photo.raw_image
        img = np.array(image)       
        cv2.imshow('img',img)
        cv2.waitKey(2)



        
        
