#Project 1 Debugger
#Guining Pertin - 26-07-2020

'''
Script to debug detection and motion planning for Project 1
'''
import cv2
import cv2.aruco as aruco

def main():
    #Start camera
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    #Set up ArUco
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    params = aruco.DetectorParameters_create()
    #Set up camera matrix and distortion coeffs
    # cv_file = cv2.FileStorage("")
    while 1:
        #Detect marker
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict,
                                                    parameters = params)
        detected = aruco.drawDetectedMarkers(frame, corners)
        #Get the rotation and translation vectors
        if np.all(ids != None):
            rvec_list = []
            tvec_list = []
            for i in range(len(irds)):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers
        cv2.imshow("detected", detected)
        k = cv2.waitKey(1)
        if k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
