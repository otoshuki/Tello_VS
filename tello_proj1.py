#Tello Project 1
#Guining Pertin - 25-07-2020

'''
As the first step towards a bigger project -
0. Calibrate the camera and get intrinsic parameters
1. Get ArUco marker location in world coords XYZ
2. Get optimal path to landing zone(+10 cm height)
3. Get optimal trajectory to landing zone(+10 cm height)
4. Atempt landing
'''

#Import libaries
import cv2
import cv2.aruco as aruco
import numpy
from easytello import Tello
import time
from threading import Thread

class Tello_controller:

    def __init__(self, speed = 50):
        '''
        Well, initialize
        '''
        #Class variables
        self.emergency_land_activated = False
        self.take_off_activated = False
        self.battery_log_delay = 2
        self.speed = speed
        #Initialize the drone and stream
        print("INIT: Connecting to Ryze Tello @192.168.10.1")
        print("Intructions: \t'Esc': Emergency Land")
        self.drone = Tello()
        time.sleep(1)
        print("LOG: Tello set up!")
        self.drone.send_command("streamon")
        time.sleep(1)
        self.cap = cv2.VideoCapture("udp://192.168.10.1:11111")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
        print("LOG: Stream service on")
        #Set up battery and emergency land threads
        self.battery_thread = Thread(target=self._battery_thread)
        self.battery_thread.daemon = True
        self.battery_thread.start()
        print("LOG: Battery thread started")
        #Set up ArUco detection
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.params = aruco.DetectorParameters_create()
        print("LOG: ArUco set up")
        #TBD: Get intrinsic camera parameters

    def run(self):
        '''
        Get stream, read aruco, get data, start controller
        '''
        print("INIT: Starting controller!")
        #One last warning before starting
        user_inp = input("Enter q to exit. Just press enter to start\n")
        if user_inp == "q":
            print("EXIT: Controller exited!")
            return 0
        #First frame takes time to load up
        #Take off
        self.drone.takeoff()
        time.sleep(1)
        self.take_off_activated = True
        print("LOG: Successful takeoff")
        #NOTE: First few frames take time to flush out
        #Run while emergency land is off
        while not self.emergency_land_activated:
            #Key press
            k = cv2.waitKey(1)
            #Detect marker
            _, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray,
                                        self.aruco_dict,
                                        parameters = self.params)
            self.detected = aruco.drawDetectedMarkers(frame, corners)
            #Show detection
            cv2.imshow("Tello Detected", self.detected)


            #Emergency land
            if k == 27:
                self.emergency_land()
                break
        #End statement
        print("EXIT: Exited successfully")

    def _battery_thread(self):
        '''
        Periodically print out battery status
        Emergency land if battery < 20%
        '''
        #Run while emergency land is off
        while not self.emergency_land_activated:
            self.battery = self.drone.get_battery()
            print("BATT: {}%".format(self.battery))
            #Warning
            #Check for failsafe
            if self.battery < 20:
                print("EMERGENCY: Battery < 20%")
                self.emergency_land()
            elif self.battery < 40:
                print("WARNING: Battery < 40%")
            time.sleep(self.battery_log_delay)

    def emergency_land(self):
        '''
        Activate landing for drone at the current position
        '''
        self.emergency_land_activated = True
        #First slow down movement
        self.drone.set_speed(10)
        #Land
        print("EMERGENCY: Landing Drone!")
        self.drone.land()
        #Shutdown stream
        time.sleep(1)
        print("EMERGENCY: Shutting stream")
        self.cap.release()
        self.drone.send_command("streamoff")
        cv2.destroyAllWindows()

def main():
    controller = Tello_controller()
    controller.run()

if __name__ == "__main__":
    main()
