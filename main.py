
import anki_vector
import numpy
import cv2 ;

def main():
    args = anki_vector.util.parse_command_args()
    with anki_vector.Robot(args.serial) as robot:
        print("Say 'Hello World'...")
        robot.behavior.say_text("Xiang zhi Cheng!")
        robot.behavior.say_text("Niu Bee")


if __name__ == "__main__":
    main()
