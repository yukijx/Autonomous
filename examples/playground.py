import pygame
from math import sin, cos, pi
import numpy as np
from pygame.locals import *
from opengl2 import Renderer, MarkerObject
import cv2
from libs.Camera import MockedCamera, Camera

import random

import signal
from libs.Rover import MockedRover
from libs.utilities import degrees_to_meters, meters_to_degrees

# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)
signal.signal(signal.SIGINT, signal_handler)

intrinsic = np.load('/home/benton/Documents/code/Autonomous/tools/papalook_1920_1080_intrinsic.npy')
distortion = np.load('/home/benton/Documents/code/Autonomous/tools/papalook_1920_1080_distortion.npy')

print(intrinsic)


RATE = 100
WIDTH = 1280
HEIGHT = 720

class Simulation:
    def __init__(self):
        self._running = False

        self.gps_measurements = []
        self.gps_targets = []
        self.aruco_markers = []
        self.ARUCO_SIZE = 0.20 # meters (20 cm tag size)

        self.renderer = Renderer(1920, 1080, hide_window=False)
        self.renderer.set_camera_matrix(intrinsic, distortion)

        self.rover = MockedRover()
        self.rover._object_tracker._known_marker_width = self.ARUCO_SIZE
        self.mocked_camera = MockedCamera(self.renderer, 0, 1920, 1080, 30)
        # self.mocked_camera = Camera(4, 1920, 1080, 30)

        self.rover._object_tracker._show_frames = False

        self.mocked_camera.set_intrinsic(intrinsic)
        self.mocked_camera.set_distortion(distortion)

        self.rover.start_tracking(self.mocked_camera)
        self.rover.start_wheels('',0)
        self.rover.start_gps('',0)
        self.rover.start_lights('',0)

    def create_random_locs(self, num):
        spread = 200
        for _ in range(num):
            lat_m = spread//2-random.random()*spread
            lon_m = spread//2-random.random()*spread
            self.create_gps_target(lat_m, lon_m, degrees=False)

    def create_gps_target(self, lat, lon, degrees=False):
        if not degrees:
            lat = meters_to_degrees(lat)
            lon = meters_to_degrees(lon)
        self.gps_targets.append((lat, lon))
    
    def create_aruco_target(self, id, lat, lon, degrees=False):
        if degrees:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
        target_pos_m = []
        m = MarkerObject(id, self.ARUCO_SIZE)
        pos = np.array([lon, 1, -lat])
        m.set_position(pos)
        rotation = np.array([0, random.random()*2*pi, 0])
        m.set_rotation(rotation)
        self.renderer.add_marker_object(m)
        target_pos_m.append((lat, lon))

        target_pos_deg = [tuple([meters_to_degrees(x) for x in t]) for t in target_pos_m]
        indices = list(range(len(target_pos_deg)))
        self.aruco_markers = [(ind, pos) for ind, pos in zip(indices, target_pos_deg)]

    def start(self):
        self._running = True
        self.rover.start_navigation(self.gps_targets)
        self.renderer.start()

    def stop(self):
        self._running = False
        self.rover.stop()
        self.renderer.stop()

    def step(self, dt):
        self.rover._gps.update_position(dt)
        gps_pos = self.rover._gps.get_position()

        real_pos = self.rover._gps.get_real_position()
        real_bearing = self.rover._gps.get_real_bearing()

        lat_m = degrees_to_meters(real_pos[0])
        lon_m = degrees_to_meters(real_pos[1])

        self.renderer.set_camera_position(lat_m, lon_m, 1)
        self.renderer.set_camera_rotation(real_bearing)

        if gps_pos not in self.gps_measurements:
            self.gps_measurements.append(gps_pos)

class Viewer:
    def __init__(self, sim: Simulation):
        self.sim = sim
        pygame.init()
        self._width = WIDTH
        self._height = HEIGHT
        self.screen = pygame.display.set_mode((self._width, self._height), RESIZABLE)
        self.clock = pygame.time.Clock()
        self._camera_pos = np.array([0,0], dtype=np.float16)
        self._scale = 50 / self._width # meters per pixel
        self._rover_size = 1 # meter
        self.orig_rover_rect = pygame.surface.Surface((int(100*2/3), 100), pygame.SRCALPHA)
        self.orig_rover_rect.fill((0, 0, 0))
        self.orig_rover_rect.set_alpha(100)
        self.running = True
        self._mouse_pressed = False

    def world_to_screen(self, x, y):
        x = (x - self._camera_pos[0]) / self._scale
        y = (y - self._camera_pos[1]) / self._scale
        return int(x + self._width/2), int(self._height/2 - y)

    def draw_gps_measurements(self, gps_measurements):
        for lat, lon in gps_measurements:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
            rover_x, rover_y = self.world_to_screen(lon, lat)
            pygame.draw.circle(self.screen, (255, 0, 0), (rover_x, rover_y), 5)

    def draw_gps_targets(self, gps_targets):
        # draw a blue circle at each target position
        for lat, lon in gps_targets:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
            target_x, target_y = self.world_to_screen(lon, lat)
            pygame.draw.circle(self.screen, (0, 0, 255), (target_x, target_y), 10)

    def draw_marker_objects(self, marker_objects):
        for id, pos in marker_objects:
            lat_m = degrees_to_meters(pos[0])
            lon_m = degrees_to_meters(pos[1])
            x, y = self.world_to_screen(lon_m, lat_m)
            pygame.draw.circle(self.screen, (0, 255, 0), (x, y), 10)

    def draw_tracked_objects(self, rover):
        tracked_objects = rover._object_tracker.tracked_objects
        for id, obj in tracked_objects.items():
            lat, lon = obj.get_position()
            lat_m = degrees_to_meters(lat)
            lon_m = degrees_to_meters(lon)
            x, y = self.world_to_screen(lon_m, lat_m)
            pygame.draw.circle(self.screen, (200, 180, 0), (x, y), 10)

    def draw_rover(self, rover):
        # get rover coordinates
        lat, lon = rover._gps.get_real_position()
        rover_x, rover_y = self.world_to_screen(degrees_to_meters(lon), degrees_to_meters(lat))

        # get rover bearing        
        real_bearing_rad = rover._gps.get_real_bearing()
        real_bearing_deg = real_bearing_rad * 180 / pi

        # draw rover
        front = pygame.surface.Surface((int(100*2/3), 100//3), pygame.SRCALPHA)
        if rover._lights.current_color == 'g':
            front.fill((0, 255, 0))
        elif rover._lights.current_color == 'o':
            front.fill((255, 165, 0))
        elif rover._lights.current_color == 'r':
            front.fill((255, 0, 0))
        self.orig_rover_rect.blit(front, (0, 0))
        rover_rect = pygame.transform.rotate(self.orig_rover_rect, -real_bearing_deg)
        rover_rect = pygame.transform.scale(rover_rect, (int(self._rover_size/self._scale), int(self._rover_size/self._scale)))
        bounds = rover_rect.get_bounding_rect()
        rover_rect.set_alpha(100)
        self.screen.blit(rover_rect, (rover_x-bounds.width//2, rover_y-bounds.height//2))

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                    self.sim.stop()
                # check key presses
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.running = False
                        self.sim.stop()
                    elif event.key == K_UP:
                        self._camera_pos[1] += 50 * self._scale
                    elif event.key == K_DOWN:
                        self._camera_pos[1] -= 50 * self._scale
                    elif event.key == K_LEFT:
                        self._camera_pos[0] -= 50 * self._scale
                    elif event.key == K_RIGHT:
                        self._camera_pos[0] += 50 * self._scale
                    elif event.key == K_EQUALS:
                        self._scale /= 1.1
                    elif event.key == K_MINUS:
                        self._scale *= 1.1

                # check mouse presses
                elif event.type == MOUSEBUTTONDOWN:
                    self._mouse_pressed = True
                elif event.type == MOUSEBUTTONUP:
                    self._mouse_pressed = False

                elif event.type == MOUSEWHEEL:
                    self._scale *= 1 + event.y * -.1

            self._width = self.screen.get_width()
            self._height = self.screen.get_height()
            rel_x, rel_y = pygame.mouse.get_rel()
            if self._mouse_pressed:
                self._camera_pos[0] -= rel_x * self._scale
                self._camera_pos[1] += rel_y * self._scale

            self.screen.fill((255, 255, 255))

            self.sim.step(1/RATE)

            self.draw_gps_targets(self.sim.gps_targets)
            self.draw_rover(self.sim.rover)
            self.draw_gps_measurements(self.sim.gps_measurements)
            self.draw_marker_objects(self.sim.aruco_markers)
            self.draw_tracked_objects(self.sim.rover)

            pygame.display.flip()

            if self.sim.rover._object_tracker._show_frames:
                frame = self.sim.rover._object_tracker._processed_frame
                if frame is not None:
                    frame = cv2.resize(frame, (640, 360))
                    cv2.imshow('frame', frame)
                    cv2.waitKey(33)

            self.clock.tick(RATE)

if __name__ == '__main__':
    sim = Simulation()
    sim.create_gps_target(30, 0, degrees=False)
    last_pos = sim.gps_targets[-1]
    sim.create_aruco_target(0, 32, 10, degrees=False)
    sim.rover.add_ar_marker(0)
    sim.start()
    p = Viewer(sim)
    p.run()