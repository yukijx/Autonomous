import random
import signal
from math import cos, pi, sin
from time import sleep, perf_counter_ns
from threading import Timer
import os

import cv2
import numpy as np
import pygame
from pygame.locals import *

from examples.renderer import MarkerObject, Renderer
from libs.Camera import Camera, MockedCamera, get_cameras_from_file
from libs.Rover import MockedRover
from libs.utilities import degrees_to_meters, meters_to_degrees, parse_config_file

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)


signal.signal(signal.SIGINT, signal_handler)

SIMULATION_FREQ = 100  # Hz
VIEWER_WIDTH = 1280
VIEWER_HEIGHT = 720

def main():
    sim = Simulation()
    sim.create_gps_target(10, 0, degrees=False)
    sim.create_aruco_target(0, 14, 0, degrees=False)
    sim.rover.add_ar_marker(0)

    rover_lat = 30
    rover_lon = 0
    sim.set_rover_position(rover_lat, rover_lon)
    sim.set_rover_bearing(180, True)
    # sim.create_gps_target(0, 30, degrees=False)
    # d_lat = np.random.random()*20-10
    # d_lon = np.random.random()*20-10
    # tag_lat = 20
    # tag_lon = 50
    # sim.create_aruco_target(3, tag_lat, tag_lon, degrees=False)
    # sim.create_gps_target(tag_lat+d_lat, tag_lon+d_lon, degrees=False)
    # sim.rover.add_ar_marker(3)

    # sim.renderer.add_cameras(sim.cameras)
    # sim.renderer.start()
    # sim.renderer.set_rover_bearing(np.deg2rad(180))
    # sim.renderer.set_rover_position(0, 0, 1)

    # sleep(1)

    sim.start(looking_for_target=True)
    # sim.rover.halt()

    # stop_timer = Timer(10, sim.stop)
    # stop_timer.start()

    viewer = Viewer(sim)
    while True:
        pass
        # sim.step(1/SIMULATION_FREQ)
        # sleep(1/SIMULATION_FREQ)
    
    viewer.run()


class Simulation:
    def __init__(self):
        self._running = False

        self.gps_measurements = []
        self.gps_targets = []
        self.aruco_markers = []

        cfg = parse_config_file('../cfg/config.ini')
        self._aruco_size = float(cfg['TRACKER']['ARUCO_SIZE'])

        self.renderer = Renderer(hide_window=False)

        self.cameras = get_cameras_from_file('cameralist_sim.json', self.renderer)

        self.rover = MockedRover()
        self.rover.configure(cfg)

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
        m = MarkerObject(id, self._aruco_size)
        pos = np.array([lon, 1 + (np.random.random()-.5), -lat])
        m.set_position(pos)
        rotation = np.array([0, random.random()*2*pi, 0])
        m.set_rotation(rotation)
        self.renderer.add_marker_object(m)
        target_pos_m.append((lat, lon))

        target_pos_deg = [tuple([meters_to_degrees(x)
                                for x in t]) for t in target_pos_m]
        indices = list(range(len(target_pos_deg)))
        self.aruco_markers = [(ind, pos)
                              for ind, pos in zip(indices, target_pos_deg)]

    def start(self, looking_for_target):
        if not self._running:
            self._running = True
            self.renderer.add_cameras(self.cameras)
            self.renderer.start()
            self.rover.start_tracking(self.cameras)
            self.rover.start_wheels('', 0)
            self.rover.start_gps('', 0)
            self.rover.start_lights('', 0)
            self.rover.start_navigation(self.gps_targets, looking_for_target)

    def stop(self):
        if self._running:
            self._running = False
            self.renderer.stop()
            self.rover.stop()

    def set_rover_position(self, lat, lon, degrees=False):
        if not degrees:
            lat = meters_to_degrees(lat)
            lon = meters_to_degrees(lon)
        self.rover._gps._real_lat = lat
        self.rover._gps._real_lon = lon
        self.rover._gps.old_latitude = lat
        self.rover._gps.old_longitude = lon

    def set_rover_bearing(self, bearing, degrees=False):
        # i let the calculated bearing be stored as degrees
        # and the "real" or simulated bearing be stored as radians
        # because i am stupid
        if degrees:
            self.rover._gps._real_bearing = np.deg2rad(bearing)
            self.rover._gps.bearing = bearing
            self.rover._gps._set_avg_bearing_list(bearing, self.rover._gps._average_bearing_length)
        else:
            self.rover._gps._real_bearing = bearing
            bearing_deg = np.rad2deg(bearing)
            self.rover._gps.bearing = bearing_deg
            self.rover._gps._set_avg_bearing_list(bearing_deg, self.rover._gps._average_bearing_length)

    def step(self, dt):
        self.rover._gps.update_position(dt)
        real_pos = self.rover._gps.get_real_position()
        real_bearing_deg = self.rover._gps.get_real_bearing()
        real_bearing_rad = np.deg2rad(real_bearing_deg)

        lat_m = degrees_to_meters(real_pos[0])
        lon_m = degrees_to_meters(real_pos[1])

        self.renderer.set_rover_position(lat_m, lon_m, 1)
        self.renderer.set_rover_bearing(real_bearing_rad)

        gps_pos = self.rover._gps.get_position()
        if gps_pos not in self.gps_measurements:
            self.gps_measurements.append(gps_pos)


class Viewer:
    def __init__(self, sim: Simulation):
        self.sim = sim
        pygame.init()
        self._width = VIEWER_WIDTH
        self._height = VIEWER_HEIGHT
        self._screen = pygame.display.set_mode(
            (self._width, self._height), RESIZABLE)
        self.clock = pygame.time.Clock()
        lat_deg, lon_deg = sim.rover._gps.get_real_position()
        lat_m = degrees_to_meters(lat_deg)
        lon_m = degrees_to_meters(lon_deg)
        self._camera_pos = np.array([lon_m, lat_m], dtype=np.float32)
        self._scale = 50 / self._width  # meters per pixel
        self._rover_size = 1  # meter
        self._orig_rover_rect = pygame.surface.Surface(
            (int(100*2/3), 100), pygame.SRCALPHA)
        self._orig_rover_rect.fill((0, 0, 0))
        self._orig_rover_rect.set_alpha(100)
        self._running = True
        self._mouse_pressed = False

    def coords_in_screen(self, x, y):
        return 0 <= x < self._width and 0 <= y < self._height

    def world_to_screen(self, x, y):
        x = (x - self._camera_pos[0]) / self._scale
        y = (y - self._camera_pos[1]) / self._scale
        return int(x + self._width/2), int(self._height/2 - y)

    def draw_gps_measurements(self, gps_measurements):
        for lat, lon in gps_measurements:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
            lat_x, lat_y = self.world_to_screen(lon, lat)
            if self.coords_in_screen(lat_x, lat_y):
                pygame.draw.circle(
                    self._screen, (255, 0, 0), (lat_x, lat_y), 5)

    def draw_gps_targets(self, gps_targets):
        # draw a blue circle at each target position
        for lat, lon in gps_targets:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
            target_x, target_y = self.world_to_screen(lon, lat)
            if self.coords_in_screen(target_x, target_y):
                pygame.draw.circle(self._screen, (0, 0, 255),
                                   (target_x, target_y), 10)

    def draw_marker_objects(self, marker_objects):
        for id, pos in marker_objects:
            lat_m = degrees_to_meters(pos[0])
            lon_m = degrees_to_meters(pos[1])
            x, y = self.world_to_screen(lon_m, lat_m)
            if self.coords_in_screen(x, y):
                pygame.draw.circle(self._screen, (0, 255, 0), (x, y), 10)

    def draw_tracked_objects(self, rover):
        tracked_objects = rover._object_tracker.tracked_objects
        for id, obj in tracked_objects.items():
            lat, lon, _ = obj.get_position()
            lat_m = degrees_to_meters(lat)
            lon_m = degrees_to_meters(lon)
            x, y = self.world_to_screen(lon_m, lat_m)
            if self.coords_in_screen(x, y):
                pygame.draw.circle(self._screen, (200, 180, 0), (x, y), 10)

    def draw_rover(self, rover):
        # get rover coordinates
        lat, lon = rover._gps.get_real_position()
        rover_x, rover_y = self.world_to_screen(
            degrees_to_meters(lon), degrees_to_meters(lat))

        # get rover bearing
        real_bearing_deg = rover._gps.get_real_bearing()
        real_bearing_rad = np.deg2rad(real_bearing_deg)

        # draw rover
        front = pygame.surface.Surface((int(100*2/3), 100//3), pygame.SRCALPHA)
        if rover._lights.current_color == 'g':
            front.fill((0, 255, 0))
        elif rover._lights.current_color == 'o':
            front.fill((255, 165, 0))
        elif rover._lights.current_color == 'r':
            front.fill((255, 0, 0))
        self._orig_rover_rect.blit(front, (0, 0))
        rover_rect = pygame.transform.rotate(
            self._orig_rover_rect, -real_bearing_deg)
        rover_rect = pygame.transform.scale(rover_rect, (int(
            self._rover_size/self._scale), int(self._rover_size/self._scale)))
        bounds = rover_rect.get_bounding_rect()

        rover_rect.set_alpha(100)
        self._screen.blit(rover_rect, (rover_x-bounds.width //
                                       2, rover_y-bounds.height//2))

        calc_bearing_deg = rover._gps.get_bearing()
        calc_bearing_rad = calc_bearing_deg * pi / 180

        # draw dots represnting camera positions
        for camera in rover._object_tracker.cameras:
            camera_offset_side_deg = meters_to_degrees(camera._position[0])
            camera_offset_front_deg = meters_to_degrees(camera._position[2])
            camera_lon_deg = lon + \
                cos(real_bearing_rad) * camera_offset_side_deg + \
                -sin(real_bearing_rad) * camera_offset_front_deg
            camera_lat_deg = lat + \
                -sin(real_bearing_rad) * camera_offset_side_deg + \
                -cos(real_bearing_rad) * camera_offset_front_deg

            camera_x, camera_y = self.world_to_screen(
                degrees_to_meters(camera_lon_deg), degrees_to_meters(camera_lat_deg))
            if self.coords_in_screen(camera_x, camera_y):
                pygame.draw.circle(self._screen, (0, 0, 0),
                                   (camera_x, camera_y), 5)

        # draw rover bearing
        bearing_x = rover_x + sin(calc_bearing_rad) * 50
        bearing_y = rover_y - cos(calc_bearing_rad) * 50
        pygame.draw.line(self._screen, (0, 0, 0),
                         (rover_x, rover_y), (bearing_x, bearing_y), 3)

    def run(self):
        last_time = perf_counter_ns()
        while self._running:
            current_time = perf_counter_ns()
            dt = (current_time - last_time) / 1e9
            last_time = current_time
            print('loop time', dt*1e3)

            for event in pygame.event.get():
                if event.type == QUIT:
                    self._running = False
                    self.sim.stop()
                # check key presses
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self._running = False
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
                    self._scale = max(self._scale, 1e-6)

            self._width = self._screen.get_width()
            self._height = self._screen.get_height()
            rel_x, rel_y = pygame.mouse.get_rel()
            if self._mouse_pressed:
                self._camera_pos[0] -= rel_x * self._scale
                self._camera_pos[1] += rel_y * self._scale

            self._screen.fill((255, 255, 255))


            sim_step_start_time = perf_counter_ns()

            self.sim.step(dt)

            sim_step_end_time = perf_counter_ns()
            print('sim', (sim_step_end_time-sim_step_start_time)/1e6)

            pygame_start_time = perf_counter_ns()

            self.draw_gps_targets(self.sim.gps_targets)
            self.draw_rover(self.sim.rover)
            self.draw_gps_measurements(self.sim.gps_measurements)
            self.draw_marker_objects(self.sim.aruco_markers)
            self.draw_tracked_objects(self.sim.rover)

            pygame_end_time = perf_counter_ns()
            print('draw time', (pygame_end_time-pygame_start_time)/1e6)

            display_start_time = perf_counter_ns()

            pygame.display.flip()

            display_end_time = perf_counter_ns()
            print('display', (display_end_time-display_start_time)/1e6)

            # actual_time = self.clock.tick(SIMULATION_FREQ)
            # print('time', actual_time)


if __name__ == '__main__':
    main()