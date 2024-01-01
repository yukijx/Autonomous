from libs.ObjectTracker import ObjectTracker
from libs.Wheels import WheelInterface, MockedWheelInterface
from libs.GPSInterface import GPSInterface, MockedGPSInterface
from libs.Navigation import Navigation
from libs.Lights import Lights
from libs.Camera import Camera

class Rover:
    def __init__(self):
        self._wheels = WheelInterface()
        self._lights = Lights()
        self._gps = GPSInterface(self._wheels)
        self._object_tracker = ObjectTracker(self._gps)
        self._navigation = Navigation(self._gps, self._wheels, self._object_tracker, self._lights)

    def __del__(self):
        self.stop()

    def stop(self):
        self._navigation.stop()
        self._gps.stop()
        self._wheels.stop()
        self._lights.stop()
        self._object_tracker.stop()

    def configure(self, config):
        self._gps.configure(config['GPS'])
        self._wheels.configure(config['WHEELS'])
        self._object_tracker.configure(config['TRACKER'])
        self._navigation.configure(config['NAVIGATION'])

    def start_gps(self, ip, port):
        self._gps.start(ip, port)

    def start_wheels(self, ip, port):
        self._wheels.start(ip, port)

    def start_lights(self, ip, port):
        self._lights.start(ip, port)

    def start_navigation(self, coordinates, looking_for_target):
        self._navigation.start(coordinates, looking_for_target)

    def start_tracking(self, cameras: "list[Camera]"):
        self._object_tracker.start(cameras)

    def add_ar_marker(self, id):
        self._object_tracker.set_markers_to_track([id])

    def halt(self):
        self._wheels.halt()
        pass

    def get_coordinates(self):
        return self._gps.latitude, self._gps.longitude
    

class MockedRover(Rover):
    def __init__(self):
        super().__init__()
        self._wheels = MockedWheelInterface()
        self._gps = MockedGPSInterface(self._wheels)
        self._object_tracker = ObjectTracker(self._gps)
        self._navigation = Navigation(self._gps, self._wheels, self._object_tracker, self._lights)