import pygame
from math import sin, cos, pi
from pygame.locals import *

import signal
from libs.Drive import Navigation
from libs.Wheels import MockedWheelInterface
from libs.GPSInterface import MockedGPSInterface
from libs.Rover import MockedRover
from libs.utilities import Location

rover = MockedRover()

# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    rover.stop()
    exit(0)
signal.signal(signal.SIGINT, signal_handler)

gps_positions = []

RATE = 60
WIDTH = 1280
HEIGHT = 720
SCALE = .1 # meters per pixel

# meters from origin (0, 0) (lat, lon)
target_1 = 60, 60
target_2 = 100, 0
target_3 = 0, -60

target_positions = [target_3, target_2, target_1]

def world_to_screen(width, height, x, y):
    return (x+width//2, height//2-y)

class Playground:
    def __init__(self):
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


        # self.rover_rect = self.orig_rover_rect.copy()
        self.running = True

    def run(self):
        while self.running:
            self.clock.tick(RATE)
            self.screen.fill((255, 255, 255))

            # print(f'lat: {lat:2.2f}, lon: {lon:2.2f}')
            # print(f'y: {y:2.2f}, x: {x:2.2f}')

            rover._gps.update_position(1/RATE)
            gps_pos = rover._gps.get_position()

            if gps_pos not in gps_positions:
                gps_positions.append(gps_pos)

            real_bearing_rad = rover._gps.get_real_bearing()
            real_bearing_deg = real_bearing_rad * 180 / pi

            # draw a blue circle at each target position
            for target in target_positions:
                target_lat, target_lon = target
                target_x, target_y = world_to_screen(WIDTH, HEIGHT, target_lon, target_lat)
                pygame.draw.circle(self.screen, (0, 0, 255), (target_x, target_y), 5)

            # get rover coordinates
            lat, lon = rover._gps.get_real_position()

            rover_x, rover_y = world_to_screen(WIDTH, HEIGHT, Location.degrees_to_meters(lon), Location.degrees_to_meters(lat))

            # draw rover
            rover_rect = pygame.transform.rotate(self.orig_rover_rect, -real_bearing_deg)
            bounds = rover_rect.get_bounding_rect()
            rover_rect.set_alpha(100)
            self.screen.blit(rover_rect, (rover_x-bounds.width//2, rover_y-bounds.height//2))


            # draw gps positions
            for lat, lon in gps_positions:
                lat = Location.degrees_to_meters(lat)
                lon = Location.degrees_to_meters(lon)
                rover_x, rover_y = world_to_screen(WIDTH, HEIGHT, lon, lat)
                pygame.draw.circle(self.screen, (255, 0, 0), (rover_x, rover_y), 5)

            pygame.display.flip()
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                    rover.stop()
                # check key presses
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.running = False
                        rover.stop()
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
    rover.start_wheels('',0)
    rover.start_gps('',0)
    # rover._wheels.set_wheel_speeds(1, 0)
    target_pos_degrees = [tuple([Location.meters_to_degrees(x) for x in t]) for t in target_positions]
    print(target_pos_degrees)
    rover.start_navigation(target_pos_degrees)
    
    p = Playground()
    p.run()