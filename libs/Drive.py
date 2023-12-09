from threading import Thread
from threading import Timer
import os

from numpy import sign

from libs.Wheels import WheelInterface
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import math
from time import sleep
from nis import maps

import sys
sys.path.append('../../Mission Control/RoverMap/')

from libs.utilities import Location, send_udp, abs_clamp
from libs.GPSInterface import GPSInterface
from libs.Wheels import WheelInterface
from libs import ARTracker

class Navigation:
    
    def __init__(self, gps: GPSInterface, wheels: WheelInterface, tracker: ARTracker):
        self.base_speed = .5

        self._gps = gps
        self._wheels = wheels
        self._tracker = tracker
        
        #Starts everything needed by the map
        # self.mapServer = MapServer()
        # self.mapServer.register_routes()
        # self.mapServer.start(debug=False)
        # self.startMap(self.updateMap, .5)
        # sleep(.1)

        self._accumulated_error = 0.0

        self._wait_period = .05
        
        #starts the thread that sends wheel speeds
        self._running = False

    def start(self, locations):
        self._locations = locations
        self._thread = Thread(target=self._drive_along_coordinates, name=('drive along coordinates'))
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
        for l in self._locations:
            print(l[0], l[1])
            self.drive_to_location(l[0], l[1])
            print('made it to a checkpoint')
        self._wheels.set_wheel_speeds(0, 0)
        print('finished drive along coordinates')

    def drive_to_location(self, lat, lon):
        self._accumulated_error = 0
        while self._running and self._gps.distance_to(lat, lon) > .0025:
            bearing_to = self._gps.bearing_to(lat, lon)
            speeds = self.calculate_speeds(.8, bearing_to, self._wait_period , kp=.0165, ki=.002)
            self._wheels.set_wheel_speeds(*speeds)
            print('dist             {0: 3.2f}'.format(self._gps.distance_to(lat, lon) * 1000))
            print('current bearing  {0: 3.2f}'.format(self._gps.bearing))
            print('target bearing   {0: 3.2f}'.format(bearing_to))
            print('accum. error     {0: 3.2f}'.format(self._accumulated_error))
            print('left speed       {0: 3.2f}'.format(speeds[0]*100))
            print('right speed      {0: 3.2f}'.format(speeds[1]*100))
            print()

            sleep(self._wait_period)

        self._wheels.set_wheel_speeds(0, 0)

    # def startMap(self, func, seconds):
    #     def func_wrapper():
    #         self.startMap(func, seconds)
    #         func()
    #     t = Timer(seconds, func_wrapper)
    #     t.start()
    #     return t

    # def updateMap(self):
    #     self.mapServer.update_rover_coords([self.gps.latitude, self.gps.longitude])

    def calculate_speeds(self, speed, bearing_error, time, kp = .35, ki=.000035):
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

        #p and i constants if doing a pivot turn
        if speed == 0:
            kp = .9
            ki = .001

        #Updates the error accumulation
        self._accumulated_error += bearing_error * time
        self._accumulated_error = abs_clamp(self._accumulated_error, -200, 200)

        #Gets the adjusted speed values
        left_speed = speed + (bearing_error * kp + self._accumulated_error * ki)
        right_speed = speed - (bearing_error * kp + self._accumulated_error * ki)

        min_speed = .1
        max_speed = .9

        # print('getspeed', left_speed, right_speed)

        # Makes sure the speeds are within the min and max
        left_speed = abs_clamp(left_speed, min_speed, max_speed)
        right_speed = abs_clamp(right_speed, min_speed, max_speed)

        # prevents complete pivots
        if abs(left_speed - right_speed) > max_speed * 2 - .2:
            if sign(left_speed) > 0:
                right_speed += abs(right_speed) * .2
            else:
                left_speed += abs(left_speed) * .2

        return (left_speed, right_speed)
    
    #Cleaner way to print out the wheel speeds
    def print_speeds(self):
        print("Left wheels: ", round(self.speeds[0],1))
        print("Right wheels: ", round(self.speeds[1],1))
    
    #Drives along a given list of GPS coordinates while looking for the given ar markers
    #Keep id2 at -1 if looking for one post, set id1 to -1 if you aren't looking for AR markers 
    def driveAlongCoordinates(self, locations, id1, id2=-1):
        #Starts the GPS
        self.gps.start_GPS_thread()
        print('Waiting for GPS connection...')
        #while self.gps.all_zero: 
        #    continue
        print('Connected to GPS')
        
        #backs up and turns to avoid running into the last detected sign. Also allows it to get a lock on heading
        if(id1 > -1):
            self.speeds = [-60,-60]
            self.print_speeds()
            sleep(2)
            self.speeds = [0,0]
            self.print_speeds()
            sleep(2)
            self.speeds = [80,20]
            self.print_speeds()
            sleep(4)
        else:
            self.speeds = (self.base_speed, self.base_speed)
            self.print_speeds()
            sleep(3)

        #navigates to each location
        for l in locations:
            self._accumulated_error = 0
            while self.gps.distance_to(l[0], l[1]) > .0025: #.0025km
                bearingTo = self.gps.bearing_to(l[0], l[1])
                print(self.gps.distance_to(l[0], l[1]) )
                self.speeds = self.calculate_speeds(self.base_speed, bearingTo, 100) #It will sleep for 100ms
                sleep(.1) #Sleeps for 100ms
                self.print_speeds()
                
                if(id1 != -1 and self.tracker.findMarker(id1, id2)):
                    self.gps.stop_GPS_thread()
                    print('Found Marker!')
                    self.speeds = [0,0]
                    return True
                    
        self.gps.stop_GPS_thread()
        print('Made it to location without seeing marker(s)')
        self.speeds = [0,0]
        return False
                
    def trackARMarker(self, id1, id2=-1):
        stopDistance = 350 #stops when 250cm from markers TODO make sure rover doesn't stop too far away with huddlys
        timesNotFound = -1
        self.tracker.findMarker(id1, id2, cameras=1) #Gets and initial angle from the main camera
        self._accumulated_error = 0
           
        count = 0
        #Centers the middle camera with the tag
        while self.tracker.angleToMarker > 14 or self.tracker.angleToMarker < -14:
            if self.tracker.findMarker(id1, id2, cameras=1): #Only looking with the center camera right now
                if timesNotFound == -1:
                    self.speeds = [0,0]
                    sleep(.5)
                    self.speeds = [self.base_speed, self.base_speed]
                    sleep(.8)
                    self.speeds = [0,0]
                else:
                    self.speeds = self.calculate_speeds(0, self.tracker.angleToMarker, 100)
                print(self.tracker.angleToMarker, " ", self.tracker.distanceToMarker)
                timesNotFound = 0
            elif timesNotFound == -1: #Never seen the tag with the main camera
                if(math.ceil(int(count/20)/5) % 2 == 1):
                    self.speeds = [self.base_speed+5,-self.base_speed-5]
                else:
                    self.speeds = [-self.base_speed-5,self.base_speed+5]
            elif timesNotFound < 15: #Lost the tag for less than 1.5 seconds after seeing it with the main camera
                timesNotFound += 1
                print(f"lost tag {timesNotFound} times")
            else:
                self.speeds = [0,0]
                print("lost it") #TODO this is bad
                timesNotFound = -1
                #return False
            self.print_speeds()
            sleep(.1)
            count+=1
        self.speeds = [0,0]
        sleep(.5)
            
        if id2 == -1:            
            self._accumulated_error = 0
            print("Locked on and ready to track")
            
            #Tracks down the tag
            while self.tracker.distanceToMarker > stopDistance or self.tracker.distanceToMarker == -1: #-1 means we lost the tag
                markerFound = self.tracker.findMarker(id1, cameras = 1) #Looks for the tag
                
                if self.tracker.distanceToMarker > stopDistance:
                    self.speeds = self.calculate_speeds(self.base_speed-8, self.tracker.angleToMarker, 100, kp = .5, ki = .0001)
                    timesNotFound = 0
                    print(f"Tag is {self.tracker.distanceToMarker}cm away at {self.tracker.angleToMarker} degrees")
                    
                elif self.tracker.distanceToMarker == -1 and timesNotFound < 10:
                    timesNotFound += 1
                    print(f"lost tag {timesNotFound} times")
                    
                elif self.tracker.distanceToMarker == -1:
                    self.speeds = [0,0]
                    print("Lost tag")
                    return False #TODO this is bad
                
                self.print_speeds()
                sleep(.1)
            
            #We scored!
            self.speeds = [0,0]
            print("In range of the tag!")
            return True
        else:
            #Gets the coords to the point that is 4m infront of the gate posts (get_coordinates expects distance in km)
            coords = self.gps.get_coordinates(self.tracker.distanceToMarker/100000.0+.004, self.tracker.angleToMarker)
            
            self.speeds = [self.base_speed, self.base_speed]
            sleep(5)
            
            #TODO: test this more after getting the new GPS
            '''
            #Rover hasn't been moving for a bit so moves to get correct bearing
            self.speeds = [-self.baseSpeed, -self.baseSpeed]
            self.printSpeeds()
            sleep(2)
            self.speeds = [0,0]
            self.printSpeeds()
            sleep(1)
            self.speeds = [self.baseSpeed, self.baseSpeed]
            self.printSpeeds()
            sleep(2)
            
            #Drives to the calculated location
            while self.gps.distance_to(coords[0], coords[1]) > .003: #TODO: might want to adjust distance here...
                bearingTo = self.gps.bearing_to(coords[0], coords[1])
                print(self.gps.distance_to(coords[0], coords[1]) )
                self.speeds = self.getSpeeds(self.baseSpeed, bearingTo, 100) #It will sleep for 100ms
                sleep(.1) #Sleeps for 100ms
                self.printSpeeds()
            '''
                        
        
         
