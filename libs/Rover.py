from libs.ObjectTracker import ObjectTracker
from libs.Wheels import WheelInterface, MockedWheelInterface
from libs.GPSInterface import GPSInterface, MockedGPSInterface
from libs.Navigation import Navigation
from libs.Lights import Lights

class Rover:
    def __init__(self):
        self._gps = GPSInterface()
        self._wheels = WheelInterface()
        self._lights = Lights()
        self._ar = ObjectTracker(self._gps)
        self._navigation = Navigation(self._gps, self._wheels, self._ar, self._lights)

    def __del__(self):
        self.stop()

    def stop(self):
        self._navigation.stop()
        self._gps.stop()
        self._wheels.stop()
        self._lights.stop()

    def start_gps(self, ip, port):
        self._gps.start(ip, port)

    def start_wheels(self, ip, port):
        self._wheels.start(ip, port)

    def start_lights(self, ip, port):
        self._lights.start(ip, port)

    def start_navigation(self, coordinates):
        self._navigation.start(coordinates)

    def add_ar_marker(self, id):
        pass

    def halt(self):
        # self._drive.halt()
        pass

    def get_coordinates(self):
        return self._gps.latitude, self._gps.longitude
    

class MockedRover(Rover):
    def __init__(self):
        super().__init__()
        self._wheels = MockedWheelInterface()
        self._gps = MockedGPSInterface(self._wheels)
        self._navigation = Navigation(self._gps, self._wheels, self._ar, self._lights)