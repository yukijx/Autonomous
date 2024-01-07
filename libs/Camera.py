import cv2
import threading
import os
import json
from time import sleep
import numpy as np

class Camera:
    """
    Interface for Camera

    Attributes:
        _camera_id (int): Camera ID
        _width (int): Frame width
        _height (int): Frame height
        _framerate (int): Frame rate
        _format (str): Frame format
        _intrinsic (np.ndarray): Camera intrinsic matrix
        _distortion (np.ndarray): Camera distortion matrix
        _cap (cv2.VideoCapture): OpenCV capture object
        _frame (np.ndarray): Current frame
        _running (bool): Whether the camera is running
        _frame_lock (threading.Lock): Lock for the frame
        _cap_thread (threading.Thread): Thread for reading frames
    """
    def __init__(self, device, width, height, fps, v_fov=50, name="", matrix_name="default", format='MJPG', intrinsic=None, distortion=None, position=None, yaw=None) -> None:
        self._device = device
        self._width = width
        self._height = height
        self._fps = fps
        self._v_fov = v_fov
        self._name = name
        self._format = format

        if intrinsic is not None:
            self._intrinsic = intrinsic
        else:
            self._intrinsic = None

        if distortion is not None:
            self._distortion = distortion
        else:
            self._distortion = None

        if position is not None:
            self._position = position
        else:
            self._position = np.array([0, 0, 0], dtype=np.float32)

        if yaw is not None:
            self._yaw = yaw
        else:
            self._yaw = 0

        self._cap = None
        self._frame = None
        self._running = False
        self._frame_lock = threading.Lock()
        self._cap_thread = threading.Thread(target=self._cap_loop, name='camera read loop')

    def _open_camera(self, camera_id: int) -> cv2.VideoCapture:
        """
        Opens the camera
        
        Args:
            camera_id (int): Camera ID

        Returns:
            cv2.VideoCapture: OpenCV capture object
        """
        if os.name == 'nt':
            # use directshow on windows (seems to work better)
            backend = cv2.CAP_DSHOW
        else:
            # TODO: Figure out the best backend for the rover
            backend = cv2.CAP_V4L
        cap = cv2.VideoCapture(camera_id, backend)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*self._format.upper()))
        return cap

    def __del__(self):
        self.stop()

    def load_intrinsic(self, path: str) -> None:
        """
        Loads the intrinsic matrix from a file
        
        Args:
            path (str): Path to the intrinsic matrix
        """
        self._intrinsic = np.load(path)

    def load_distortion(self, path: str) -> None:
        """
        Loads the distortion matrix from a file

        Args:
            path (str): Path to the distortion matrix
        """
        self._distortion = np.load(path)

    def set_intrinsic(self, intrinsic: np.ndarray) -> None:
        """
        Sets the intrinsic matrix
        
        Args:
            intrinsic (np.ndarray): Intrinsic matrix
        """
        self._intrinsic = intrinsic

    def set_distortion(self, distortion: np.ndarray) -> None:
        """
        Sets the distortion matrix

        Args:
            distortion (np.ndarray): Distortion matrix
        """
        self._distortion = distortion

    def get_intrinsic(self) -> np.ndarray:
        """
        Gets the intrinsic matrix
        
        Returns:
            np.ndarray: Intrinsic matrix
        """
        return self._intrinsic
    
    def get_distortion(self) -> np.ndarray:
        """
        Gets the distortion matrix

        Returns:
            np.ndarray: Distortion matrix
        """
        return self._distortion

    def set_property(self, prop: int, value: int) -> None:
        """
        Sets a cv2 property of the camera
        
        Args:
            prop (int): Property to set
            value (int): Value to set the property to
        """
        self._cap.set(prop, value)

    def get_property(self, prop: int) -> int:
        """
        Gets a cv2 property of the camera

        Args:
            prop (int): Property to get

        Returns:
            int: Value of the property
        """
        return self._cap.get(prop)
    
    def set_position(self, position: np.ndarray) -> None:
        """
        Sets the position of the camera
        
        Args:
            position (np.ndarray): Position of the camera
        """
        self._position = position

    def set_yaw(self, theta: float) -> None:
        """
        Sets the yaw of the camera

        Args:
            theta (float): Yaw of the camera
        """
        self._yaw = theta

    def get_position(self) -> np.ndarray:
        """
        Gets the position of the camera
        
        Returns:
            np.ndarray: Position of the camera
        """
        return self._position
    
    def get_yaw(self) -> float:
        """
        Gets the yaw of the camera

        Returns:
            float: Yaw of the camera
        """
        return self._yaw

    def start(self) -> None:
        """
        Starts the camera
        """
        if not self._running:
            if self._cap is None:
                self._cap = self._open_camera(self._device)
            self._running = True
            self._cap_thread.start()

    def stop(self) -> None:
        """
        Stops the camera
        """
        if self._running:
            self._running = False
            self._cap_thread.join()
            if self._cap is not None:
                self._cap.release()
                self._cap = None

    def _cap_loop(self) -> None:
        """
        Thread loop for reading frames
        """
        while self._running:
            grabbed, frame = self._cap.read()
            if grabbed and frame is not self._frame:
                with self._frame_lock:
                    self._frame = frame

    def get_frame(self) -> np.ndarray:
        """
        Gets the current frame
        
        Returns:
            np.ndarray: Current frame
        """
        with self._frame_lock:
            return self._frame
        
class MockedCamera(Camera):
    """
    Mocked camera for testing, can be set up to read frames rendered by an OpenGL renderer
    
    Attributes:
        _renderer (Renderer): Renderer to get frames from
    """
    def __init__(self, renderer, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._renderer = renderer

    def start(self) -> None:
        """
        Starts the camera
        """
        if not self._running:
            self._running = True
            self._cap_thread.start()

    def load_frame(self) -> None:
        """
        Loads a frame from the renderer
        """
        frame = self._renderer.get_frame(self._device)
        if frame is not None:
            self._frame = frame

    def _cap_loop(self) -> None:
        """
        Thread loop for reading frames
        """
        while self._running:
            self.load_frame()
            sleep(1/self._fps)

def generate_default_matrices(width, height, v_fov) -> tuple[np.ndarray, np.ndarray]:
    """
    Generates default intrinsic and distortion matrices for simulation.
    ALWAYS USE PROPERLY CALIBRATED CAMERAS FOR REAL WORLD USE!
    
    Args:
        width (int): Width of camera
        height (int): Height of camera
        v_fov (float): Vertical field of view of camera
        
    Returns:
        tuple[np.ndarray, np.ndarray]: Intrinsic and distortion matrices
    """
    # generate default intrinsic and distortion matrices
    f = width / (4.0 * np.tan(np.radians(v_fov / 2)))
    intrinsic = np.array([
        [f, 0, width / 2],
        [0, f, height / 2],
        [0, 0, 1]
    ], dtype=np.float32)
    distortion = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    return intrinsic, distortion

def get_camera_matrices(name, width, height, v_fov) -> tuple[np.ndarray, np.ndarray]:
    """
    Gets the intrinsic and distortion matrices from a calibration file
    
    Args:
        name (str): Name of camera
        width (int): Width of camera
        height (int): Height of camera
        v_fov (float): Vertical field of view of camera
        
    Returns:
        tuple[np.ndarray, np.darray]: Intrinsic and distortion matrices
    """
    cwd = os.getcwd()
    ind = cwd.find('Autonomous')
    cwd = cwd[:ind+11]
    # get the intrinsic and distortion matrices from a calibration file
    default_intrinsic, default_distortion = generate_default_matrices(width, height, v_fov)
    try:
        intrinsic = np.load(f'{cwd}/cfg/{name}_{width}_{height}_intrinsic.npy')
    except FileNotFoundError:
        print(f'Could not find intrinsic matrix for {name} at {cwd}/cfg/{name}_{width}_{height}_intrinsic.npy'
            f'\nUsing default intrinsic matrix')
        intrinsic = default_intrinsic

    try:
        distortion = np.load(f'{cwd}/cfg/{name}_{width}_{height}_distortion.npy')
    except FileNotFoundError:
        print(f'Could not find distortion matrix for {name} at {cwd}/cfg/{name}_{width}_{height}_distortion.npy'
            f'\nUsing default distortion matrix')
        distortion = default_distortion

    return intrinsic, distortion

def get_cameras_from_file(camerafile: str, renderer=None) -> list:
    """
    Gets a list of cameras from a file
    
    Args:
        camerafile (str): Path to camera json file
        renderer (Renderer): Renderer to use for mocked cameras

    Returns:
        list[Camera]: List of cameras
    """
    cameras = []
    if not os.path.exists(camerafile):
        raise FileNotFoundError(f'Could not find camera file at {camerafile}')
    with open(camerafile) as f:
        data = json.load(f)
        indices = data['indices']
        camera_data = [data['cameras'][i] for i in indices]
        for camera in camera_data:
            name = camera['name']
            device = camera['device']
            width = camera['width']
            height = camera['height']
            fps = camera['fps']
            v_fov = camera['v_fov']
            matrix_name = camera['matrix_name']
            c_format = camera['format']
            x_offset = camera['x_offset']
            z_offset = -camera['z_offset']
            yaw = camera['yaw']
            intrinsic, distortion = get_camera_matrices(matrix_name, width, height, v_fov)
            position = np.array([x_offset, 0, z_offset], dtype=np.float32)
            camera_params = {
                'device': device,
                'width': width,
                'height': height,
                'fps': fps,
                'v_fov': v_fov,
                'name': name,
                'format': c_format,
                'intrinsic': intrinsic,
                'distortion': distortion,
                'position': position,
                'yaw': yaw
            }
            if not renderer:
                cameras.append(Camera(**camera_params))
            else:
                cameras.append(MockedCamera(renderer, **camera_params))
    return cameras

if __name__ == '__main__':
    from examples.renderer import Renderer
    renderer = Renderer(1920, 1080, 50)
    cameras = get_cameras_from_file('cfg/cameralist.json', renderer)
    renderer.add_cameras(cameras)
    print(len(cameras))
    for camera in cameras:
        camera.start()
    renderer.start()
    while True:
        for camera in cameras:
            frame = camera.get_frame()
            if frame is not None:
                cv2.imshow(camera._name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            for camera in cameras:
                camera.stop()
            break