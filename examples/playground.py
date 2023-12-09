import pygame
from math import sin, cos, pi
from pygame.locals import *

import random

import signal
from libs.Drive import Navigation
from libs.Wheels import MockedWheelInterface
from libs.GPSInterface import MockedGPSInterface
from libs.Rover import MockedRover
from libs.utilities import Location

# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)
signal.signal(signal.SIGINT, signal_handler)

gps_positions = []

RATE = 100
WIDTH = 1280
HEIGHT = 720
SCALE = .1 # meters per pixel

def world_to_screen(width, height, x, y):
    return (x+width//2, height//2-y)

class Simulation:
    def __init__(self):
        self._running = True

        self.gps_measurements = []
        self.gps_targets = []

        self.rover = MockedRover()
        self.rover.start_wheels('',0)
        self.rover.start_gps('',0)

    def create_locs(self, num):
        target_pos_m = []
        spread = 200
        for _ in range(num):
            target_pos_m.append((spread//2-random.random()*spread, spread//2-random.random()*spread))
        self.gps_targets = [tuple([Location.meters_to_degrees(x) for x in t]) for t in target_pos_m]
    
    def start(self):
        self.rover.start_navigation(self.gps_targets)

    def stop(self):
        self._running = False
        self.rover.stop()

    def step(self, dt):
        self.rover._gps.update_position(dt)
        gps_pos = self.rover._gps.get_position()

        if gps_pos not in self.gps_measurements:
            self.gps_measurements.append(gps_pos)

        # draw gps positions
        for lat, lon in gps_positions:
            lat = Location.degrees_to_meters(lat)
            lon = Location.degrees_to_meters(lon)
            rover_x, rover_y = world_to_screen(WIDTH, HEIGHT, lon, lat)
            pygame.draw.circle(self.screen, (255, 0, 0), (rover_x, rover_y), 5)

class Playground:
    def __init__(self, sim: Simulation):
        self.sim = sim
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()
        self._box_size = 30
        self.orig_rover_rect = pygame.surface.Surface((int(self._box_size*2/3), self._box_size), pygame.SRCALPHA)
        self.orig_rover_rect.fill((0, 0, 0))
        self.orig_rover_rect.set_alpha(100)
        front = pygame.surface.Surface((int(self._box_size*2/3), self._box_size/3), pygame.SRCALPHA)
        front.fill((255, 0, 0))
        self.orig_rover_rect.blit(front, (0, 0))
        self.running = True

    def draw_gps_measurements(self, gps_measurements):
        for lat, lon in gps_measurements:
            lat = Location.degrees_to_meters(lat)
            lon = Location.degrees_to_meters(lon)
            rover_x, rover_y = world_to_screen(WIDTH, HEIGHT, lon, lat)
            pygame.draw.circle(self.screen, (255, 0, 0), (rover_x, rover_y), 5)

    def draw_gps_targets(self, gps_targets):
        # draw a blue circle at each target position
        for lat, lon in gps_targets:
            lat = Location.degrees_to_meters(lat)
            lon = Location.degrees_to_meters(lon)
            target_x, target_y = world_to_screen(WIDTH, HEIGHT, lon, lat)
            pygame.draw.circle(self.screen, (0, 0, 255), (target_x, target_y), 5)

    def draw_rover(self, rover):
        # get rover coordinates
        lat, lon = rover._gps.get_real_position()
        rover_x, rover_y = world_to_screen(WIDTH, HEIGHT, Location.degrees_to_meters(lon), Location.degrees_to_meters(lat))

        # get rover bearing        
        real_bearing_rad = rover._gps.get_real_bearing()
        real_bearing_deg = real_bearing_rad * 180 / pi

        # draw rover
        rover_rect = pygame.transform.rotate(self.orig_rover_rect, -real_bearing_deg)
        bounds = rover_rect.get_bounding_rect()
        rover_rect.set_alpha(100)
        self.screen.blit(rover_rect, (rover_x-bounds.width//2, rover_y-bounds.height//2))

    def run(self):
        while self.running:
            self.clock.tick(RATE)
            self.screen.fill((255, 255, 255))

            self.sim.step(1/RATE)

            self.draw_gps_targets(self.sim.gps_targets)
            self.draw_rover(self.sim.rover)
            self.draw_gps_measurements(self.sim.gps_measurements)

            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                    self.sim.stop()
                # check key presses
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.running = False
                        self.sim.stop()
                #     elif event.key == K_UP:
                #         rover._wheels.set_wheel_speeds(1, 1)
                #     elif event.key == K_DOWN:
                #         rover._wheels.set_wheel_speeds(-1, -1)
                #     elif event.key == K_LEFT:
                #         rover._wheels.set_wheel_speeds(-1, 1)
                #     elif event.key == K_RIGHT:
                #         rover._wheels.set_wheel_speeds(1, -1)
                # elif event.type == KEYUP:
                #     if event.key == K_UP or event.key == K_DOWN or event.key == K_LEFT or event.key == K_RIGHT:
                #         rover._wheels.set_wheel_speeds(0, 0)

if __name__ == '__main__':
    sim = Simulation()
    sim.create_locs(1)
    sim.start()
    p = Playground(sim)
    p.run()