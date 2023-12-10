import cv2
import threading
from time import sleep

class Camera:
    def __init__(self, camera_id=0, width=1280, height=720, format='MJPG'):
        self._camera_id = camera_id
        self._width = width
        self._height = height
        self._format = format
        self._cap = None
        self._frame = None
        self._running = False
        self._frame_lock = threading.Lock()
        self._cap_thread = threading.Thread(target=self._cap_loop, name='camera read loop')

    def _open_camera(self, camera_id):
        cap = cv2.VideoCapture(camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*self._format.upper()))
        # cap.set(10, 0.1)
        return cap

    def __del__(self):
        self.stop()

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
            grabbed, frame = self._cap.read()
            if grabbed:
                with self._frame_lock:
                    self._frame = frame

    def get_frame(self):
        with self._frame_lock:
            return self._frame
        
class MockedCamera(Camera):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def start(self):
        if not self._running:
            self._running = True
            self._cap_thread.start()

    def _cap_loop(self):
        while self._running:
            with self._frame_lock:
                sleep(.1)
                # print('reading frame')
                # self._frame = cv2.imread('rover.png')