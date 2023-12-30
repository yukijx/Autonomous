from threading import Thread
import os

from numpy import sign

from libs.Wheels import WheelInterface
os.chdir(os.path.dirname(os.path.abspath(__file__)))
from time import sleep
from libs.utilities import abs_clamp
from libs.GPSInterface import GPSInterface
from libs.Wheels import WheelInterface
from libs.ObjectTracker import ObjectTracker
from libs.Lights import Lights

class Navigation:
    
    def __init__(self, gps: GPSInterface, wheels: WheelInterface, tracker: ObjectTracker, lights: Lights):
        self.base_speed = .5

        self._gps = gps
        self._wheels = wheels
        self._tracker = tracker
        self._lights = lights

        self._locations = []
        
        self._accumulated_error = 0.0

        self._STOPPING_DISTANCE = 1.0 # meters
        self._WAIT_PERIOD = .05
        self._MAX_SPEED = 0.9
        self._MIN_SPEED = 0.1
        self._REVERSE_WHEEL_MOD = 0.7
        self._MAX_BEARING_ERROR = 200
        self._K_P = 0.0165
        self._K_I = 0.002

        self._target_found = False
        
        #starts the thread that sends wheel speeds
        self._running = False
        self._thread = Thread(target=self._drive_along_coordinates, name=('drive along coordinates'))

    def configure(self, config: dict):
        self._K_P = float(config['K_P'])
        self._K_I = float(config['K_I'])
        self._STOPPING_DISTANCE = float(config['STOPPING_DISTANCE'])
        self._WAIT_PERIOD = float(config['NAVIGATION_UPDATE_PERIOD'])
        self._MAX_SPEED = float(config['MAX_SPEED'])
        self._MIN_SPEED = float(config['MIN_SPEED'])
        self._REVERSE_WHEEL_MOD = float(config['REVERSE_WHEEL_MOD'])
        self._MAX_BEARING_ERROR = float(config['MAX_BEARING_ERROR'])

    def start(self, locations):
        self._locations = locations
        if not self._running:
            self._running = True
            self._thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()

    def _drive_along_coordinates(self):
        print('starting drive along coordinates')
        self._wheels.set_wheel_speeds(self.base_speed, self.base_speed)
        sleep(1)
        location_ind = 0
        self._accumulated_error = 0
        while self._running:
            if not self._tracker.any_targets_found():
                lat, lon = self._locations[location_ind]
                destination_reached = self._drive_to_location(lat, lon)
                if destination_reached:
                    print('made it to a checkpoint')
                    location_ind += 1
                    if location_ind >= len(self._locations):
                        break
            else:
                target = self._tracker.get_closest_target()
                lat, lon, dist = target.get_position()
                destination_reached = self._drive_to_location(lat, lon, dist)
                if destination_reached:
                    print('made it to a marker')
                    self._running = False

            sleep(self._WAIT_PERIOD)
        self._wheels.set_wheel_speeds(0, 0)
        print('finished drive along coordinates')
        self._lights.found()

    def _drive_to_location(self, lat, lon, marker_dist=None):
        self._bearing_to = self._gps.bearing_to(lat, lon)
        self._calc_speeds = self._calculate_speeds(.8, self._bearing_to, self._WAIT_PERIOD , self._K_P, self._K_I)
        self._wheels.set_wheel_speeds(*self._calc_speeds)
        if marker_dist is None:
            dist = self._gps.distance_to(lat, lon) * 1000
        else:
            dist = min(marker_dist, self._gps.distance_to(lat, lon) * 1000)
        self.print_info(dist)
        return dist < self._STOPPING_DISTANCE

    def print_info(self, dist: float) -> None:
        print('dist             {0: 3.2f}'.format(dist))
        print('current bearing  {0: 3.2f}'.format(self._gps.bearing))
        print('target bearing   {0: 3.2f}'.format(self._bearing_to))
        print('accum. error     {0: 3.2f}'.format(self._accumulated_error))
        print('left speed       {0: 3.2f}'.format(self._calc_speeds[0]*100))
        print('right speed      {0: 3.2f}'.format(self._calc_speeds[1]*100))
        print()

    def _calculate_speeds(self, speed, bearing_error, time, kp, ki):
        """
        Gets the adjusted speed values for the wheels based off of the current
        bearing error as well as the accumulated bearing error. (A PID loop using
        the P and I constants).
        
        Args:
            speed: The speed the rover is going (float, in range 0-1)
            bearing_error: The error in the bearing (degrees)
            time: The time between calls to this function (seconds)
            kp: The proportional constant
            ki: The integral constant   
            
        Returns:
            The adjusted speed values for the wheels
        """
        left_speed = 0
        right_speed = 0

        #Updates the error accumulation
        self._accumulated_error += bearing_error * time
        self._accumulated_error = abs_clamp(self._accumulated_error, -self._MAX_BEARING_ERROR, self._MAX_BEARING_ERROR)

        #Gets the adjusted speed values
        left_speed = speed + (bearing_error * kp + self._accumulated_error * ki)
        right_speed = speed - (bearing_error * kp + self._accumulated_error * ki)

        # Makes sure the speeds are within the min and max
        left_speed = abs_clamp(left_speed, self._MIN_SPEED, self._MAX_SPEED)
        right_speed = abs_clamp(right_speed, self._MIN_SPEED, self._MAX_SPEED)
        
        # prevents complete pivots
        # the side that is going backwards is slowed down
        if abs(left_speed - right_speed) > self._MAX_SPEED * 2 - .2:
            if sign(left_speed) > 0:
                right_speed += abs(right_speed) * (1-self._REVERSE_WHEEL_MOD)
            else:
                left_speed += abs(left_speed) * (1-self._REVERSE_WHEEL_MOD)

        return (left_speed, right_speed)