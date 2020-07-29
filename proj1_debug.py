#Project 1 Debugger
#Guining Pertin - 26-07-2020

'''
Script to debug detection and motion planning for Project 1
'''
#Import stuff
import cv2
import cv2.aruco as aruco
import numpy as np

def draw_paths(img, cam_mtx, dist, R, t, curr_pos):
    # curr_pos = [20,20,5]
    # curr_pos[2]
    #First get the curve from curr_pos to goal (0,0)
    #For now, consider a straight path from curr_pos to final
    the_path = np.linspace(curr_pos, [0,0,5], num = 20)
    #Convert from world to camera coords
    #Create the projection matrix
    imgpts, _ = cv2.projectPoints(the_path, R, t, cam_mtx, dist)
    # print(imgpts[9].ravel())
    for i in range(len(the_path)):
        point_3d = the_path[i]
        img_point = imgpts[i].ravel().astype(np.int32)
        #Draw circle with size depending on dist from cam
        # cir_size = np.linalg.norm(curr_pos - point_3d)
        img = cv2.circle(img, tuple(img_point), 5, (255,0,0), 2)
    return img

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
    cv_file.release()
    #Drone position (optional)

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
            # print("Camera coordinates:", cam_coords)
            #Draw paths
            try:
                cv_file = cv2.FileStorage("drone_loc.yaml", cv2.FILE_STORAGE_READ)
                pos = cv_file.getNode("curr_loc").mat()
                pre_pos = pos
            except:
                pos = pre_pos
                print("error case")
                pre_pos = pos
            print("Drone coordinates: ", pos.ravel())
            detected = draw_paths(detected, cam_mtx, dist_coeff, rvecs[0][0], t_vec, pos.ravel())
        #Show image
        cv2.imshow("detected", detected)
        cv_file.release()
        k = cv2.waitKey(1)
        if k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
