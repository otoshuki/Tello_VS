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
import numpy as np

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
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
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
        #Set up camera matrix and distortion coeffs
        self.cv_file = cv2.FileStorage("tello_params.yaml", cv2.FILE_STORAGE_READ)
        self.cam_mtx = self.cv_file.getNode("camera_matrix").mat()
        self.dist_coeff = self.cv_file.getNode("dist_coeff").mat()[0]
        self.cv_file.release()
        #Publish own location


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
        img_id = 0
        while not self.emergency_land_activated:
            #Key press
            k = cv2.waitKey(1)
            #Detect marker
            _, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray,
                                        self.aruco_dict,
                                        parameters = self.params)
            #Draw the marker
            self.detected = aruco.drawDetectedMarkers(frame, corners)
            #Get the rotation and translation vectors
            if np.all(ids != None):
                rvec_list = []
                tvec_list = []
                #Get the estimated pose of the marker wrt camera, marker size = 9cm
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 9, self.cam_mtx, self.dist_coeff)
                #Also draw axes
                for i in range(len(ids)):
                    aruco.drawAxis(self.detected, self.cam_mtx, self.dist_coeff, rvecs[i], tvecs[i], 4.5)
                #Get the rotation matrix using Rodrigues formula
                r_mat, _ = cv2.Rodrigues(rvecs[0][0])
                #The only translation vector to use
                t_vec = tvecs[0][0]
                #Get transformation matrix
                self.M_mat = np.zeros((4,4))
                self.M_mat[:3, :3] = r_mat
                self.M_mat[:3, 3] = t_vec
                self.M_mat[3,3] = 1
                #Get inverse transformation, of camera wrt marker
                self.M_inv = np.linalg.inv(self.M_mat)
                #Get camera location wrt marker
                self.cam_coords = self.M_inv[:3, 3]
                # print("Camera coordinates:", self.cam_coords)
                self.cv_file = cv2.FileStorage("drone_loc.yaml", cv2.FILE_STORAGE_WRITE)
                self.cv_file.write("curr_loc", self.cam_coords)
            #Show detection
            cv2.imshow("Tello Detected", self.detected)
            #Take pictures with s
            if k == ord("s"):
                cv2.imwrite("tello_img{0}.jpg".format(img_id), frame)
                print("LOG: Saved image as tello_img{0}.jpg".format(img_id))
                img_id += 1
            #Emergency land
            if k == 27:
                self.emergency_land()
                break
        #End statement
        self.cv_file = cv2.FileStorage("drone_loc.yaml", cv2.FILE_STORAGE_WRITE)
        self.cv_file.write("curr_loc", np.array([0,0,0]))
        self.cv_file.release()
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
