
import anki_vector
import numpy
import cv2 

def main():
    args = anki_vector.util.parse_command_args()
    with anki_vector.Robot(serial= '008014c1') as robot:
        pass



if __name__ == "__main__":
    main()
