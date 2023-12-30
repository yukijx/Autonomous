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

    def set_intrinsic(self, intrinsic):
        self._intrinsic = intrinsic

    def set_distortion(self, distortion):
        self._distortion = distortion

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
    def __init__(self, renderer, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._renderer = renderer
        self._show_frame = True

    def start(self):
        if not self._running:
            self._running = True
            self._cap_thread.start()

    def load_frame(self):
        frame = self._renderer.get_frame()
        if frame is not None:
            # frame = cv2.undistort(frame, self._intrinsic, self._distortion)

            self._frame = frame

    def _cap_loop(self):
        while self._running:
            self.load_frame()
            # if self._show_frame and self._frame is not None:
            #     cv2.imshow('frame', self._frame)
            #     cv2.waitKey(1)
            sleep(1/self._framerate)