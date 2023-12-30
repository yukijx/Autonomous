import argparse
import os
import signal

import cv2
import numpy as np

from libs.Camera import Camera

# this is stupid lmao but it works
# when you press ctrl-c, it will stop capturing images
# and start processing them
# if you press ctrl-c again, it will exit
capturing = True


def signal_handler(sig, frame):
    global capturing
    if capturing:
        print('Stopping capture...')
        capturing = False
    else:
        print('Exiting...')
        exit(0)


signal.signal(signal.SIGINT, signal_handler)


os.chdir(os.path.dirname(os.path.abspath(__file__)))

# default aruco stuff
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

OUTPUT_DIR = '../cfg/'

# default values i used to calibrate my camera
WIDTH = 1920
HEIGHT = 1080
PATTERN_WIDTH = 10
PATTERN_HEIGHT = 7
SQUARE_SIZE = 0.021  # 2.1 cm
MAX_IMAGES = 60
BRIGHTNESS = 150
CLARITY_THRESHOLD = 160


def get_clarity(frame):
    return cv2.Laplacian(frame, cv2.CV_64F).var()


def find_chessboard(frame, pattern_width, pattern_height):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(
        gray, (pattern_width, pattern_height), None)
    if ret:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
    return ret, corners


def calculate_error(obj_points, img_points, rvecs, tvecs, mtx, dist):
    mean_error = 0
    for i in range(len(obj_points)):
        img_points2, _ = cv2.projectPoints(
            obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(img_points[i], img_points2,
                         cv2.NORM_L2) / len(img_points2)
        mean_error += error
    return mean_error


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-iw', '--width', type=int, default=WIDTH,
                    help='Width of image in pixels')
    ap.add_argument('-ih', '--height', type=int, default=HEIGHT,
                    help='Height of image in pixels')
    ap.add_argument('-pw', '--pattern_width', type=int, default=PATTERN_WIDTH,
                    help='Width of chessboard pattern in inner corners')
    ap.add_argument('-ph', '--pattern_height', type=int, default=PATTERN_HEIGHT,
                    help='Height of chessboard pattern in inner corners')
    ap.add_argument('-s', '--square_size', type=float,
                    default=SQUARE_SIZE, help='Size of chessboard squares in meters')
    ap.add_argument('-m', '--max_images', type=int, default=MAX_IMAGES,
                    help='Maximum number of images to use for calibration')
    ap.add_argument('-b', '--brightness', type=int,
                    default=BRIGHTNESS, help='Brightness of camera')
    ap.add_argument('-t', '--threshold', type=int, default=CLARITY_THRESHOLD,
                    help='Minimum clarity to accept image (100-400 or so)')
    ap.add_argument('-v', '--visualize', action='store_true',
                    help='Visualize images as they are processed')
    ap.add_argument('-c', '--camera', type=int,
                    default=0, help='Camera ID to use')
    ap.add_argument('-n', '--name', type=str, default='',
                    help='Name of camera for file saving')
    args = vars(ap.parse_args())

    # get arguments into variables
    width = args['width']
    height = args['height']
    pattern_width = args['pattern_width']
    pattern_height = args['pattern_height']
    square_size = args['square_size']
    max_images = args['max_images']
    brightness = args['brightness']
    clarity_threshold = args['threshold']
    visualize = args['visualize']
    camera_id = args['camera']
    name = args['name']

    camera = Camera(camera_id, width, height, 30, 'MJPG')
    camera.start()
    # camera.set_property(cv2.CAP_PROP_BRIGHTNESS, brightness)
    last_frame = None

    objp = np.zeros((pattern_width * pattern_height, 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_width, 0:pattern_height].T.reshape(-1, 2)

    objp = objp * square_size

    obj_points = []
    img_points = []
    clarities = []

    while capturing:
        frame = camera.get_frame()
        if frame is not None and frame is not last_frame:
            last_frame = frame

            # clarity = get_clarity(frame)
            # print(f'clarity = {clarity:3.2f}')

            # if clarity > clarity_threshold:
            ret, corners = find_chessboard(
                frame, pattern_width, pattern_height)

            if ret:
                clarity = get_clarity(frame)
                obj_points.append(objp)
                img_points.append(corners)
                clarities.append(clarity)
                print(f'clarity = {clarity:3.2f}')
                print('images: ', len(obj_points))
                if visualize:
                    frame = cv2.drawChessboardCorners(
                        frame, (pattern_width, pattern_height), corners, ret)

            if visualize:
                cv2.imshow('frame', frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
    camera.stop()
    cv2.destroyAllWindows()

    if len(obj_points) < 1:
        print('No images found')
        return
    elif len(obj_points) < 10:
        print('Not enough images found')
        return
    elif len(obj_points) < max_images:
        sampled_img_points = img_points
        sampled_obj_points = obj_points
    else:
        # sort by clarity
        indices = np.argsort(clarities)
        # get the best images
        sampled_img_points = [img_points[i] for i in indices[-max_images:]]
        sampled_obj_points = [obj_points[i] for i in indices[-max_images:]]

        # indices = np.random.choice(len(obj_points), max_images, replace=False)
        # sampled_img_points = [img_points[i] for i in indices]
        # sampled_obj_points = [obj_points[i] for i in indices]

    print('Calibrating...')

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        sampled_obj_points, sampled_img_points, (width, height), None, None)

    print(len(sampled_img_points), ' images used to calibrate camera')
    error = calculate_error(
        sampled_obj_points, sampled_img_points, rvecs, tvecs, mtx, dist)
    np.set_printoptions(suppress=True, precision=3)
    print('Error: ', error)
    print('Intrinsic matrix:\n', mtx)
    print('Distortion coefficients:\n', dist)
    np.save(OUTPUT_DIR + f'{name}_{width}_{height}_intrinsic', mtx)
    np.save(OUTPUT_DIR + f'{name}_{width}_{height}_distortion', dist)


if __name__ == '__main__':
    main()
