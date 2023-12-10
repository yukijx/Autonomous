import numpy as np
import cv2
import os
from libs.Camera import Camera
from libs.ARTracker import ARTracker

os.chdir(os.path.dirname(os.path.abspath(__file__)))

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_SIZE = 0.142 # 200mm

def main():
    intrinsic = np.load('papalook_calibration_matrix.npy')
    distortion = np.load('papalook_distortion_coefficients.npy')
    camera = Camera(1, 640, 480, 'MJPG')
    camera.start()
    artracker = ARTracker()
    while True:
        frame = camera.get_frame()
        if frame is not None:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT)
            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE, intrinsic, distortion)
                for i in range(len(ids)):
                    angle, dist = artracker._get_marker_location(corners[i][0])
                    print(rvecs[i], tvecs[i])
                    print(angle, dist)
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, intrinsic, distortion, rvecs[i], tvecs[i], 0.1)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    camera.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()