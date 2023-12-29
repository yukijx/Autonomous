import numpy as np
import cv2
import os
from libs.Camera import Camera
from libs.ObjectTracker import ObjectTracker
from libs.utilities import get_marker_location
from time import perf_counter_ns
import signal

running = True

def signal_handler(sig, frame):
    print('Exiting...')
    global running
    running = False
signal.signal(signal.SIGINT, signal_handler)

os.chdir(os.path.dirname(os.path.abspath(__file__)))

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_SIZE = 0.10 # 200mm

CAMERA_INDEX = 4
WIDTH = 1920
HEIGHT = 1080
CAMERA_NAME = 'papalook'

def main():
    intrinsic = np.load(f'../tools/{CAMERA_NAME}_{WIDTH}_{HEIGHT}_intrinsic.npy')
    distortion = np.load(f'../tools/{CAMERA_NAME}_{WIDTH}_{HEIGHT}_distortion.npy')
    camera = Camera(CAMERA_INDEX, WIDTH, HEIGHT, 30, 'MJPG')
    camera.start()
    # artracker = ARTracker()
    last_frame = None
    while running:
        start = perf_counter_ns()
        frame = camera.get_frame()
        if frame is not None and frame is not last_frame:
            last_frame = frame
            corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT)
            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE, intrinsic, distortion)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):
                    angle, dist = get_marker_location(corners[i], ARUCO_SIZE, intrinsic, distortion)
                    # print(rvecs[i][0], tvecs[i][0])
                    # print(np.linalg.norm(rvecs[i][0]), np.linalg.norm(tvecs[i][0]))
                    print(angle, dist)
                    cv2.drawFrameAxes(frame, intrinsic, distortion, rvecs[i], tvecs[i], 0.1)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            last_frame_time = perf_counter_ns()
        end = perf_counter_ns()
    camera.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()