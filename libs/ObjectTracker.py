import os
import threading
from datetime import datetime
from time import time

import cv2
import cv2.aruco as aruco
import numpy as np

from libs.Camera import Camera
from libs.GPSInterface import GPSInterface
from libs.utilities import (degrees_to_meters, get_coordinates,
                            get_marker_location)

'''
darknetPath = os.path.dirname(os.path.abspath(__file__)) + '/../YOLO/darknet/'
sys.path.append(darknetPath)
from darknet_images import *
from darknet import load_network
'''

class TrackedObject:
    def __init__(self, max_estimates):
        self._MAX_ESTIMATES = max_estimates
        self._position_estimates = []
        self._position_estimate_length = 0
        self._position_estimate_index = 0
        self._last_update_time = None

    def get_position(self):
        avg_lat = sum([x[0][0] for x in self._position_estimates]
                      ) / len(self._position_estimates)
        avg_lon = sum([x[0][1] for x in self._position_estimates]
                      ) / len(self._position_estimates)
        avg_dist = sum([x[1] for x in self._position_estimates]
                       ) / len(self._position_estimates)
        return (avg_lat, avg_lon, avg_dist)

    def update(self, position, distance):
        self._last_update_time = time()
        if self._position_estimate_length < self._MAX_ESTIMATES:
            self._position_estimates.append((position, distance))
            self._position_estimate_length += 1
        else:
            self._position_estimates[self._position_estimate_index] = (
                position, distance)
            self._position_estimate_index = (
                self._position_estimate_index + 1) % self._position_estimate_length

    def get_last_update_time(self):
        return self._last_update_time

    def get_last_position(self):
        last_index = (self._position_estimate_index -
                      1) % self._position_estimate_length
        return self._position_estimates[last_index]

    def clear_estimates(self):
        self._position_estimates = []
        self._position_estimate_length = 0
        self._position_estimate_index = 0
        self._last_update_time = None


class ObjectTracker:
    """
    Class to visually track and estimate the location of objects

    Attributes:
        _gps (GPSInterface): GPS interface to get the current location of the rover
        _save_to_disk (bool): Whether or not to save the video to disk
        _use_yolo (bool): Whether or not to use YOLO to detect objects
        markers_to_track (list[int]): List of AR markers to track
        tracked_objects (dict[int, TrackedObject]): Dictionary of tracked objects, keyed by AR marker ID
        _tracked_object_n_locations (int): Number of locations to average for each tracked object
        _marker_dict (cv2.aruco_Dictionary): Dictionary of AR markers
        _last_frame (np.ndarray): Last frame received from the camera
        _marker_mode (bool): Whether or not to track AR markers
        _running (bool): Whether or not the thread is running
        _thread (threading.Thread): Thread that runs the tracking loop
        _aruco_size (float): Size of the AR markers in meters
        _object_valid_time (float): Time in seconds before a tracked object is considered invalid
        _video_writer (cv2.VideoWriter): Video writer to save the video to disk
    """

    def __init__(self, gps: GPSInterface, save_to_disk=False, use_yolo=False):
        self._gps = gps
        self._save_to_disk = save_to_disk
        self._use_yolo = use_yolo  # not implemented

        self.markers_to_track = []
        self.tracked_objects = {}

        self._tracked_object_n_locations = 10

        # Set the ar marker dictionary
        # hardcoded because we only use one dictionary
        self._marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        self._last_frame = None
        self._marker_mode = True
        self._running = False
        self._thread = threading.Thread(
            target=self._tracking_loop, name='AR search')

        self._aruco_size = 0.020  # meters

        self._object_valid_time = 5.0  # seconds

        # sets up yolo
        # if use_YOLO:
        #     os.chdir(darknetPath)
        #     weights = config['YOLO']['WEIGHTS']
        #     cfg = config['YOLO']['CFG']
        #     data = config['YOLO']['DATA']
        #     self.thresh = float(config['YOLO']['THRESHOLD'])
        #     self.network, self.class_names, self.class_colors = load_network(cfg, data, weights, 1)
        #     os.chdir(os.path.dirname(os.path.abspath(__file__)))

        #     self.networkWidth = darknet.network_width(self.network)
        #     self.networkHeight = darknet.network_height(self.network)

    def __del__(self):
        self.stop()

    def configure(self, config: dict) -> None:
        """
        Configures the object tracker

        Args:
            config (dict): Dictionary of config values
        """
        self._save_to_disk = config['SAVE_TO_DISK'].lower() == 'true'
        self._aruco_size = float(config['ARUCO_SIZE'])
        self._tracked_object_n_locations = int(
            config['TRACKED_OBJECT_LOCATION_ESTIMATES'])
        self._object_valid_time = float(config['TRACKED_OBJECT_VALID_TIME'])

    def start(self, camera: Camera) -> None:
        """
        Starts the object tracker

        Args:
            camera (Camera): Camera to get frames from
        """
        self.camera = camera
        # get properties of camera
        self._frame_width = self.camera._width
        self._frame_height = self.camera._height
        fps = self.camera._framerate
        # Initialize camera
        self.camera.start()

        # start the thread
        if not self._running:
            self._running = True
            if self._save_to_disk:
                start_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                self._video_writer = cv2.VideoWriter(f"../autonomous_{start_timestamp}.avi", cv2.VideoWriter_fourcc(
                    *'MJPG'), fps, (self._frame_width, self._frame_height), False)
            print('starting ar thread')
            self._thread.start()

    def stop(self) -> None:
        """
        Stops the object tracker
        """
        if self._running:
            self._running = False
            self._thread.join()
            self.camera.stop()
            if self._save_to_disk:
                self._video_writer.release()

    def set_markers_to_track(self, markers: "list[int]"):
        """
        Sets the markers to track

        Args:
            markers (list[int]): List of AR marker IDs to track
        """
        self.markers_to_track = markers

    def any_targets_found(self) -> bool:
        """
        Returns whether or not any targets are found and
        whether or not they have been seen recently

        Returns:
            bool: Whether or not any targets are found
        """
        for marker_id in self.markers_to_track:
            target = self.tracked_objects.get(marker_id)
            if target is not None:
                return time() - target.get_last_update_time() < self._object_valid_time

    def get_closest_target(self) -> TrackedObject:
        """
        Returns the closest target

        Returns:
            TrackedObject: Closest target
        """
        closest_marker = None
        closest_distance = 999999999
        for marker_id in self.markers_to_track:
            if self.tracked_objects.get(marker_id) is not None:
                _, distance = self.tracked_objects[marker_id].get_last_position(
                )
                if distance < closest_distance:
                    closest_distance = distance
                    closest_marker = marker_id
        return self.tracked_objects[closest_marker]

    def _tracking_loop(self):
        while self._running:
            frame = self.camera.get_frame()
            if frame is not None and frame is not self._last_frame:
                self._last_frame = frame
                intrinsic = self.camera.get_intrinsic()
                distortion = self.camera.get_distortion()
                self._search_for_markers(frame, intrinsic, distortion)

    def _search_for_markers(self, frame: np.ndarray, intrinsic: np.ndarray, distortion: np.ndarray):
        print('searching for markers')
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, marker_ids, _ = aruco.detectMarkers(frame, self._marker_dict)
        if marker_ids is not None:
            for marker_id, corner in zip(marker_ids, corners):
                if marker_id[0] in self.markers_to_track:
                    print('found marker', marker_id[0])
                    self._marker_found = True
                    angle, distance = get_marker_location(
                        corner, self._aruco_size, intrinsic, distortion)
                    print('ad', angle, distance)
                    if self.tracked_objects.get(marker_id[0]) is None:
                        self.tracked_objects[marker_id[0]] = TrackedObject(
                            self._tracked_object_n_locations)
                    rover_lat, rover_lon = self._gps.get_position()
                    rover_bearing = self._gps.get_bearing()
                    measured_coord = get_coordinates(
                        rover_lat, rover_lon, distance/1000, rover_bearing + angle)
                    mc_lat, mc_lon = measured_coord
                    mc_lat_m = degrees_to_meters(mc_lat)
                    mc_lon_m = degrees_to_meters(mc_lon)
                    rover_lat_m = degrees_to_meters(rover_lat)
                    rover_lon_m = degrees_to_meters(rover_lon)
                    print('rover', rover_lat_m, rover_lon_m)
                    print('distance', distance)
                    print('mc', mc_lat_m, mc_lon_m)
                    self.tracked_objects[marker_id[0]].update(
                        measured_coord, distance)

            if self._save_to_disk:
                frame = cv2.aruco.drawDetectedMarkers(
                    frame, corners, marker_ids)
        if self._save_to_disk:
            self._video_writer.write(frame)

    """
    Leaving this method commented out here because it may be useful in the future, but it is not currently used
    """
    # helper method to convert YOLO detections into the aruco corners format
    # def _convert_to_corners(self, detections, num_corners):
    #     corners = []
    #     xCoef = self._frame_width / self.networkWidth
    #     yCoef = self._frame_height / self.networkHeight
    #     if len(detections) < num_corners:
    #         print('ERROR, convertToCorners not used correctly')
    #         raise ValueError
    #     for i in range(0, num_corners):
    #         tagData = list(detections[i][2]) #Gets the x, y, width, height

    #         #YOLO resizes the image so this sizes it back to what we're exepcting
    #         tagData[0] *= xCoef
    #         tagData[1]*= yCoef
    #         tagData[2] *= xCoef
    #         tagData[3] *= yCoef

    #         #Gets the corners
    #         topLeft = [tagData[0] - tagData[2]/2, tagData[1] - tagData[3]/2]
    #         topRight = [tagData[0] + tagData[2]/2, tagData[1] - tagData[3]/2]
    #         bottomRight = [tagData[0] + tagData[2]/2, tagData[1] + tagData[3]/2]
    #         bottomLeft = [tagData[0] - tagData[2]/2, tagData[1] + tagData[3]/2]

    #         #appends the corners with the same format as aruco
    #         corners.append([[topLeft, topRight, bottomRight, bottomLeft]])

    #     return corners