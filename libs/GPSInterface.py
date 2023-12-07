from math import cos, sin
from threading import Thread
from time import sleep
import time

from numpy.random import Generator, PCG64

from libs.utilities import Location

MOCKED = False
class fake_gps:
    def __init__(self):
        self._latitude = 0
        self._longitude = 0
        self._height = 0
        self._time = 0
        self._error = 0
    def get_latitude():
        return 0
    def get_longitude():
        return 0
    def get_height():
        return 0
    def get_time():
        return 0
    def get_error():
        return 0
    
    def gps_init(ip, port):
        return
    def gps_finish():
        return

if not MOCKED:
    try:
        # try to import gps module
        from gps import gps
    except ImportError:
        # on failure, use mocked gps
        print("GPSInterface: gps module not found, using mock gps")
        gps = fake_gps
        MOCKED = True
else:
    gps = fake_gps

# import os
# os.chdir(os.path.dirname(os.path.abspath(__file__)))
# Class that computes functions related to location of Rover
class GPSInterface:
    def __init__(self, mocked=False):
        self.latitude = 0
        self.longitude = 0
        self.old_latitude = 0
        self.old_longitude = 0
        self.height = 0
        self.time = 0
        self.error = 0
        self.bearing = 0.0
        self.running = True
        self.all_zero = True
        self._update_period = .25

    def get_position(self):
        """
        Returns current latitude and longitude

        Returns:
            tuple: (latitude, longitude)
        """
        return (self.latitude, self.longitude)

    def distance_to(self, lat:float, lon:float):
        """
        Returns distance in kilometers between self and given latitude and longitude

        Args:
            lat (float): Latitude of second point
            lon (float): Longitude of second point

        Returns:
            float: Distance in kilometers between self and given latitude and longitude
        """
        return Location.distance_to(self.latitude, self.longitude, lat, lon)

    def bearing_to(self, lat:float, lon:float):
        """ 
        Calculates difference between given bearing to location and current bearing
        Positive is turn right, negative is turn left

        Args:
            lat (float): Latitude of second point
            lon (float): Longitude of second point

        Returns:
            float: Difference between given bearing to location and current bearing
        """
        resultbearing = Location.calc_bearing(self.latitude, self.longitude, lat, lon) - self.bearing
        if resultbearing < -180:
            resultbearing += 360
        elif resultbearing > 180:
            resultbearing -= 360
        return resultbearing

    def start(self, ip, port):
        """
        Starts the GPS
        """
        self.swift_ip = ip
        self.swift_port = port
        gps.gps_init(self.swift_ip, self.swift_port)
        self.running = True
        self._thread = Thread(target=self._update_fields_loop, name=(
                'update GPS fields'), args=())
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        """
        Stops the GPS
        """
        self.running = False
        self._thread.join()
        gps.gps_finish()        
    

    def _update_fields_loop(self):
        """
        Updates the GPS fields repeatedly
        """
        while(self.running):
            if gps.get_latitude() != 0 or gps.get_longitude() != 0:
                self.old_latitude = self.latitude
                self.old_longitude = self.longitude

                self.latitude = gps.get_latitude()
                self.longitude = gps.get_longitude()
                self.height = gps.get_height()
                self.time = gps.get_time()
                self.error = gps.get_error()
                self.bearing = self.calc_bearing(self.old_latitude, self.old_longitude, self.latitude, self.longitude)
                self.all_zero = False
            else:
                self.all_zero = True
            # maybe print info or sleep or something idk
            sleep(self._update_period)

class MockedGPSInterface(GPSInterface):
    def __init__(self, wheels):
        super().__init__()
        self._wheels = wheels
        self._real_lat = 0 # degrees
        self._real_lon = 0 # degrees
        self._real_bearing = 0 # radians
        self._noise_dev = 0.03 # 30 centimeters
        self._noise_generator = Generator(PCG64(time.time_ns()))
        self._SPEED = 1
        self._single_side_authority = .53

    def update_position(self, dt):
        left_wheel_speed, right_wheel_speed = self._wheels._speeds
        # print(f'left_wheel_speed: {left_wheel_speed}, right_wheel_speed: {right_wheel_speed}')
        distance_traveled = (self._SPEED * (left_wheel_speed + right_wheel_speed) / 2) * dt
        bearing_change = (left_wheel_speed - right_wheel_speed) * self._single_side_authority
        self._real_bearing += bearing_change * dt
        d_lat = cos(self._real_bearing) * Location.meters_to_degrees(distance_traveled)
        d_lon = sin(self._real_bearing) * Location.meters_to_degrees(distance_traveled)

        self._real_lat += d_lat
        self._real_lon += d_lon

    def get_real_position(self):
        return self._real_lat, self._real_lon
    
    def get_real_bearing(self):
        return self._real_bearing

    def _update_fields_loop(self):
        """
        Updates the GPS fields repeatedly
        """
        while(self.running):
            self.old_latitude = self.latitude
            self.old_longitude = self.longitude

            scaled_noise = Location.meters_to_degrees(self._noise_dev)

            self.latitude = self._real_lat + self._noise_generator.normal(0, scaled_noise)
            self.longitude = self._real_lon + self._noise_generator.normal(0, scaled_noise)
            # self.latitude = self._real_lat
            # self.longitude = self._real_lon
            # self.height = gps.get_height()
            # self.time = gps.get_time()
            # self.error = gps.get_error()
            new_bearing = Location.calc_bearing(self.old_latitude, self.old_longitude, self.latitude, self.longitude)
            self.bearing = self.bearing + (new_bearing - self.bearing) / 2
            sleep(self._update_period)