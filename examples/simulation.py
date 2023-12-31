import random
import signal
from math import cos, pi, sin
from time import sleep
from threading import Timer

import cv2
import numpy as np
import pygame
from pygame.locals import *

from examples.renderer import MarkerObject, Renderer
from libs.Camera import Camera, MockedCamera
from libs.Rover import MockedRover
from libs.utilities import degrees_to_meters, meters_to_degrees, parse_config_file, get_camera_matrices


# this stuff makes it ctrl-c-able
def signal_handler(sig, frame):
    exit(0)


signal.signal(signal.SIGINT, signal_handler)

SIMULATION_FREQ = 100  # Hz
VIEWER_WIDTH = 1280
VIEWER_HEIGHT = 720
CAMERA_V_FOV = 50


class Simulation:
    def __init__(self):
        self._running = False

        self.gps_measurements = []
        self.gps_targets = []
        self.aruco_markers = []

        cfg = parse_config_file('../cfg/config.ini')
        self._aruco_size = float(cfg['TRACKER']['ARUCO_SIZE'])
        self._camera_width = int(cfg['CAMERA']['WIDTH'])
        self._camera_height = int(cfg['CAMERA']['HEIGHT'])
        self._camera_fps = int(cfg['CAMERA']['FPS'])

        self.renderer = Renderer(
            self._camera_width, self._camera_height, CAMERA_V_FOV, hide_window=False)

        camera_name = cfg['CAMERA']['NAME']

        intrinsic, distortion = get_camera_matrices(
            camera_name, self._camera_width, self._camera_height)
        
        camera_center_pos = np.array([0, 0, 0], dtype=np.float32)
        camera_center_yaw = 0
        camera_left_pos = np.array([-.2, 0, 0], dtype=np.float32)
        camera_left_yaw = -30
        camera_right_pos = np.array([.2, 0, 0], dtype=np.float32)
        camera_right_yaw = 30

        self.mocked_camera_center = MockedCamera(
            self.renderer, 0, self._camera_width, self._camera_height, self._camera_fps,
            intrinsic=intrinsic,
            distortion=distortion, 
            position=camera_center_pos, 
            yaw=camera_center_yaw)

        self.mocked_camera_left = MockedCamera(
            self.renderer, 1, self._camera_width, self._camera_height, self._camera_fps,
            intrinsic=intrinsic,
            distortion=distortion, 
            position=camera_left_pos, 
            yaw=camera_left_yaw)


        self.mocked_camera_right = MockedCamera(
            self.renderer, 2, self._camera_width, self._camera_height, self._camera_fps,
            intrinsic=intrinsic,
            distortion=distortion, 
            position=camera_right_pos, 
            yaw=camera_right_yaw)

        self.renderer.add_camera(camera_center_pos, camera_center_yaw)
        self.renderer.add_camera(camera_left_pos, camera_left_yaw)
        self.renderer.add_camera(camera_right_pos, camera_right_yaw)

        cameras = [ self.mocked_camera_center, self.mocked_camera_left, self.mocked_camera_right]

        self.rover = MockedRover()
        self.rover.configure(cfg)

        self.rover._object_tracker._show_frames = False

        self.rover.start_tracking(cameras)
        self.rover.start_wheels('', 0)
        self.rover.start_gps('', 0)
        self.rover.start_lights('', 0)

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
        self._running = True
        self.rover.start_navigation(self.gps_targets, looking_for_target)
        self.renderer.start()

    def stop(self):
        self._running = False
        self.renderer.stop()
        self.rover.stop()

    def step(self, dt):
        self.rover._gps.update_position(dt)
        gps_pos = self.rover._gps.get_position()

        real_pos = self.rover._gps.get_real_position()
        real_bearing = self.rover._gps.get_real_bearing()

        lat_m = degrees_to_meters(real_pos[0])
        lon_m = degrees_to_meters(real_pos[1])

        self.renderer.set_rover_position(lat_m, lon_m, 1)
        self.renderer.set_rover_bearing(real_bearing)

        if gps_pos not in self.gps_measurements:
            self.gps_measurements.append(gps_pos)


class Viewer:
    def __init__(self, sim: Simulation):
        self.sim = sim
        pygame.init()
        self._width = VIEWER_WIDTH
        self._height = VIEWER_HEIGHT
        self.screen = pygame.display.set_mode(
            (self._width, self._height), RESIZABLE)
        self.clock = pygame.time.Clock()
        self._camera_pos = np.array([0, 0], dtype=np.float32)
        self._scale = 50 / self._width  # meters per pixel
        self._rover_size = 1  # meter
        self.orig_rover_rect = pygame.surface.Surface(
            (int(100*2/3), 100), pygame.SRCALPHA)
        self.orig_rover_rect.fill((0, 0, 0))
        self.orig_rover_rect.set_alpha(100)
        self.running = True
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
                pygame.draw.circle(self.screen, (255, 0, 0), (lat_x, lat_y), 5)

    def draw_gps_targets(self, gps_targets):
        # draw a blue circle at each target position
        for lat, lon in gps_targets:
            lat = degrees_to_meters(lat)
            lon = degrees_to_meters(lon)
            target_x, target_y = self.world_to_screen(lon, lat)
            if self.coords_in_screen(target_x, target_y):
                pygame.draw.circle(self.screen, (0, 0, 255),
                                   (target_x, target_y), 10)

    def draw_marker_objects(self, marker_objects):
        for id, pos in marker_objects:
            lat_m = degrees_to_meters(pos[0])
            lon_m = degrees_to_meters(pos[1])
            x, y = self.world_to_screen(lon_m, lat_m)
            if self.coords_in_screen(x, y):
                pygame.draw.circle(self.screen, (0, 255, 0), (x, y), 10)

    def draw_tracked_objects(self, rover):
        tracked_objects = rover._object_tracker.tracked_objects
        for id, obj in tracked_objects.items():
            lat, lon, _ = obj.get_position()
            lat_m = degrees_to_meters(lat)
            lon_m = degrees_to_meters(lon)
            x, y = self.world_to_screen(lon_m, lat_m)
            if self.coords_in_screen(x, y):
                pygame.draw.circle(self.screen, (200, 180, 0), (x, y), 10)

    def draw_rover(self, rover):
        # get rover coordinates
        lat, lon = rover._gps.get_real_position()
        rover_x, rover_y = self.world_to_screen(
            degrees_to_meters(lon), degrees_to_meters(lat))

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
        rover_rect = pygame.transform.rotate(
            self.orig_rover_rect, -real_bearing_deg)
        rover_rect = pygame.transform.scale(rover_rect, (int(
            self._rover_size/self._scale), int(self._rover_size/self._scale)))
        bounds = rover_rect.get_bounding_rect()

        rover_rect.set_alpha(100)
        self.screen.blit(rover_rect, (rover_x-bounds.width //
                         2, rover_y-bounds.height//2))

        calc_bearing = rover._gps.get_bearing()
        calc_bearing_rad = calc_bearing * pi / 180

        # draw rover bearing
        bearing_x = rover_x + sin(calc_bearing_rad) * 50
        bearing_y = rover_y - cos(calc_bearing_rad) * 50
        pygame.draw.line(self.screen, (0, 0, 0),
                         (rover_x, rover_y), (bearing_x, bearing_y), 3)

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
                    self._scale = max(self._scale, 1e-6)

            self._width = self.screen.get_width()
            self._height = self.screen.get_height()
            rel_x, rel_y = pygame.mouse.get_rel()
            if self._mouse_pressed:
                self._camera_pos[0] -= rel_x * self._scale
                self._camera_pos[1] += rel_y * self._scale

            self.screen.fill((255, 255, 255))

            self.sim.step(1/SIMULATION_FREQ)

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

            self.clock.tick(SIMULATION_FREQ)


if __name__ == '__main__':
    sim = Simulation()
    sim.create_gps_target(30, 0, degrees=False)
    sim.create_aruco_target(0, 40, 10, degrees=False)
    sim.rover.add_ar_marker(0)
    # sim.create_gps_target(0, 30, degrees=False)
    # d_lat = np.random.random()*20-10
    # d_lon = np.random.random()*20-10
    # tag_lat = 20
    # tag_lon = 50
    # sim.create_aruco_target(3, tag_lat, tag_lon, degrees=False)
    # sim.create_gps_target(tag_lat+d_lat, tag_lon+d_lon, degrees=False)
    # sim.rover.add_ar_marker(3)
    sim.start(looking_for_target=True)

    # stop_timer = Timer(10, sim.stop)
    # stop_timer.start()

    viewer = Viewer(sim)
    viewer.run()
