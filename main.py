
import anki_vector
import numpy
import cv2
import time
from anki_vector.util import distance_mm, speed_mmps


def main():
    with anki_vector.Robot(serial= '0070132a') as robot:
    while True:
       robot.motors.set_wheel_motors(60,60,240,240)
       time.sleep(5)
       robot.motors.stop_all_motors()


if __name__ == "__main__":
    main()
