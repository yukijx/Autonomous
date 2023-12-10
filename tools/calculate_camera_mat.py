import numpy as np
import cv2 as cv
import glob
import os
from argparse import ArgumentParser

ap = ArgumentParser()
ap.add_argument("-p", "--path", required=True, help="path to calibration images")
ap.add_argument("-n", "--name", required=True, help="name of camera")
args = ap.parse_args()

name = args.name
path = args.path

PATTERN_SIZE = (6, 10)
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((PATTERN_SIZE[0]*PATTERN_SIZE[1],3), np.float32)
objp[:,:2] = np.mgrid[0:PATTERN_SIZE[0],0:PATTERN_SIZE[1]].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob(os.path.join(path, '*.jpg'))
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, PATTERN_SIZE, None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, PATTERN_SIZE, corners2, ret)
        print(fname)
        cv.imshow('img', img)
        cv.waitKey(50)
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

np.save(os.path.join(path, name + '_intrinsic.npy'), mtx)
np.save(os.path.join(path, name + '_distortion.npy'), dist)

# img = cv.imread(images[-1])
# h, w = img.shape[:2]
# newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]
# cv.imshow('calibrated', dst)
# cv.waitKey(0)