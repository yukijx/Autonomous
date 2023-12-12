import pytest
import cv2
import numpy as np
from time import sleep
from libs.ObjectTracker import ObjectTracker
from libs.Camera import MockedCamera

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMS = {
    'DEGREES_PER_PIXEL': 0.09375,
    'VDEGREES_PER_PIXEL': .125,
    'FOCAL_LENGTH': 435,
    'FOCAL_LENGTH30H': 590,
    'FOCAL_LENGTH30V': 470,
    'KNOWN_TAG_WIDTH': 20,
    'FORMAT': 'MJPG',
    'FRAME_WIDTH': 1280,
    'FRAME_HEIGHT': 720
}

# @pytest.fixture
def mocked_camera():
    return MockedCamera(0, 1280, 720, 'MJPG')

# @pytest.fixture
def ar_tracker():
    tracker = ObjectTracker()
    tracker.set_parameters(PARAMS)
    tracker.set_markers_to_track([0, 1])
    return tracker

# @pytest.fixture
def frame():
    MARKER_SIZE = 200
    M1_X = 200
    M1_Y = 100
    M2_X = 500
    M2_Y = 100
    frame = np.array([[[255,255,255]]*1280]*720, dtype=np.uint8)
    marker1 = cv2.aruco.generateImageMarker(ARUCO_DICT, 0, MARKER_SIZE, frame)
    marker1 = cv2.cvtColor(marker1, cv2.COLOR_GRAY2BGR)

    marker2 = cv2.aruco.generateImageMarker(ARUCO_DICT, 1, MARKER_SIZE, frame)
    marker2 = cv2.cvtColor(marker2, cv2.COLOR_GRAY2BGR)

    frame[M1_Y:M1_Y+MARKER_SIZE, M1_X:M1_X+MARKER_SIZE] = marker1
    frame[M2_Y:M2_Y+MARKER_SIZE, M2_X:M2_X+MARKER_SIZE] = marker2
    
    return frame

def test_tag_found(frame, mocked_camera, ar_tracker):
    # MARKER_SIZE = 200
    # X_LOCATION = 200
    # Y_LOCATION = 100
    # MARKER_IND = 0

    # mocked_camera = MockedCamera(0, 1280, 720, 'MJPG')

    # frame = np.array([[[255,255,255]]*1280]*720, dtype=np.uint8)
    # marker_im = cv2.aruco.generateImageMarker(ARUCO_DICT, MARKER_IND, MARKER_SIZE, frame)

    # marker_im = cv2.cvtColor(marker_im, cv2.COLOR_GRAY2BGR)

    # print(marker_im.shape)
    # print(frame.shape)

    # frame[Y_LOCATION:Y_LOCATION+MARKER_SIZE, X_LOCATION:X_LOCATION+MARKER_SIZE] = marker_im
    # mocked_camera._frame = frame

    # cv2.imshow('frame', frame)
    # cv2.waitKey(0)

    # tracker = ARTracker()
    # tracker.set_parameters(PARAMS)
    # tracker.set_markers_to_track([MARKER_IND])

    mocked_camera._frame = frame
    ar_tracker.start(mocked_camera)
    sleep(1)
    mocked_camera.stop()
    ar_tracker.stop()

    assert ar_tracker._marker_found == True

if __name__ == '__main__':
    test_tag_found(frame(), mocked_camera(), ar_tracker())

    # frame = frame()
    # # frame = np.array([[[255,255,255]]*1280]*720, dtype=np.uint8)

    # corners, ids, rejected = cv2.aruco.detectMarkers(frame, ARUCO_DICT)

    # print(corners, ids, rejected)

    # for id, corner in zip(ids, corners):
    #     print(id[0], corner[0])

    # out_frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    # cv2.imshow('frame', out_frame)
    # cv2.waitKey(0)    