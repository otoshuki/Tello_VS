import cv2
from easytello import Tello
import time

def main():
    print("Setting up Tello")
    drone = Tello()
    time.sleep(1)
    print("Tello set up!")
    print("Starting stream service")
    drone.send_command("streamon")
    time.sleep(2)
    cap = cv2.VideoCapture("udp://"+drone.tello_ip+":11111")
    print("Started")
    while 1:
        _, frame = cap.read()
        cv2.imshow("DJI Tello", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
    drone.send_command("streamoff")


if __name__ == "__main__":
    main()
