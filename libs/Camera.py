import cv2
import threading
import os
from time import sleep
import numpy as np

class Camera:
    def __init__(self, camera_id=0, width=1280, height=720, framerate=30, format='MJPG'):
        self._camera_id = camera_id
        self._width = width
        self._height = height
        self._framerate = framerate
        self._format = format
        self._intrinsic = None
        self._distortion = None
        self._cap = None
        self._frame = None
        self._running = False
        self._frame_lock = threading.Lock()
        self._cap_thread = threading.Thread(target=self._cap_loop, name='camera read loop')

    def _open_camera(self, camera_id):
        if os.name == 'nt':
            backend = cv2.CAP_DSHOW
        else:
            # TODO: Figure out the best backend for the rover
            backend = cv2.CAP_V4L
        cap = cv2.VideoCapture(camera_id, backend)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FPS, self._framerate)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*self._format.upper()))
        return cap

    def __del__(self):
        self.stop()

    def load_intrinsic(self, path):
        self._intrinsic = np.load(path)

    def load_distortion(self, path):
        self._distortion = np.load(path)

    def get_intrinsic(self):
        return self._intrinsic
    
    def get_distortion(self):
        return self._distortion

    def set_property(self, prop, value):
        self._cap.set(prop, value)

    def get_property(self, prop):
        return self._cap.get(prop)

    def start(self):
        if not self._running:
            if self._cap is None:
                self._cap = self._open_camera(self._camera_id)
            self._running = True
            self._cap_thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._cap_thread.join()
            if self._cap is not None:
                self._cap.release()

    def _cap_loop(self):
        while self._running:
            # start = perf_counter_ns()
            grabbed, frame = self._cap.read()
            # end = perf_counter_ns()
            # print(((end-start)/1e6), 'ms to decode')
            if grabbed and frame is not self._frame:
                with self._frame_lock:
                    self._frame = frame

    def get_frame(self):
        with self._frame_lock:
            return self._frame
        
class MockedCamera(Camera):
    def __init__(self, simulation, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._simulation = simulation

    def start(self):
        if not self._running:
            self._running = True
            self._cap_thread.start()

    def generate_frame(self):
        rover_pos = self._simulation.rover._gps.get_real_position()
        rover_bearing = self._simulation.rover._gps.get_real_bearing()
        aruco_markers = self._simulation.aruco_markers

    def _cap_loop(self):
        while self._running:
            with self._frame_lock:
                sleep(1/self._framerate)
                # print('reading frame')
                # self._frame = cv2.imread('rover.png')