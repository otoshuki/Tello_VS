import cv2
import glob

images = glob.glob("tello_calib_images/*.jpg")

cv_file = cv2.FileStorage("tello_params.yaml", cv2.FILE_STORAGE_READ)
mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()[0]
cv_file.release()

img = cv2.imread(images[0])
h, w = img.shape[:2]

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

id = 0
for fname in images:
    img = cv2.imread(fname)
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('undistort_{0}.jpg'.format(id),dst)
    id += 1
