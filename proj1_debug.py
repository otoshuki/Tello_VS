#Project 1 Debugger
#Guining Pertin - 26-07-2020

'''
Script to debug detection and motion planning for Project 1
'''
#Import stuff
import cv2
import cv2.aruco as aruco
import numpy as np

def main():
    #Start camera
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    #Set up ArUco
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    params = aruco.DetectorParameters_create()
    #Set up camera matrix and distortion coeffs
    cv_file = cv2.FileStorage("debug_params.yaml", cv2.FILE_STORAGE_READ)
    cam_mtx = cv_file.getNode("camera_matrix").mat()
    print("Camera Matrix: ")
    print(cam_mtx)
    dist_coeff = cv_file.getNode("dist_coeff").mat()[0]
    print("Distortion Coefficients: ")
    print(dist_coeff)
    while 1:
        #Detect marker
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict,
                                                    parameters = params)
        detected = aruco.drawDetectedMarkers(frame, corners)
        #Get the rotation and translation vectors
        if np.all(ids != None):
            rvec_list = []
            tvec_list = []
            #Get the estimated pose of the marker wrt camera
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 9, cam_mtx, dist_coeff)
            #Also draw axes
            for i in range(len(ids)):
                aruco.drawAxis(detected, cam_mtx, dist_coeff, rvecs[i], tvecs[i], 4.5)
            #Get the rotation matrix using Rodrigues formula
            r_mat, _ = cv2.Rodrigues(rvecs[0][0])
            #The only translation vector to use
            t_vec = tvecs[0][0]
            #Get transformation matrix
            M_mat = np.zeros((4,4))
            M_mat[:3, :3] = r_mat
            M_mat[:3, 3] = t_vec
            M_mat[3,3] = 1
            #Get inverse transformation, of camera wrt marker
            M_inv = np.linalg.inv(M_mat)
            #Get camera location wrt marker
            cam_coords = M_inv[:3, 3]
            #Debug print
            # print("Homogeneous transformation:\n", M_mat)
            # print("Inverse transfromation:\n", M_inv)
            print("Camera coordinates:", cam_coords)
        #Show image
        cv2.imshow("detected", detected)
        k = cv2.waitKey(1)
        if k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
