import cv2
import cv2.aruco as aruco
import numpy as np
from libs.Camera import Camera
from libs.utilities import get_marker_location, get_coordinates, degrees_to_meters
from libs.GPSInterface import GPSInterface
from time import sleep, time
import threading
import os
from enum import Enum

'''
darknetPath = os.path.dirname(os.path.abspath(__file__)) + '/../YOLO/darknet/'
sys.path.append(darknetPath)
from darknet_images import *
from darknet import load_network
'''

class ObjectTracker:

    # Constructor
    #Cameras should be a list of file paths to cameras that are to be used
    #set write to True to write to disk what the cameras are seeing
    #set useYOLO to True to use yolo when attempting to detect the ar tags
    def __init__(self, gps: GPSInterface, save_to_disk=False, use_yolo = False):
        self._save_to_disk=save_to_disk
        self._use_yolo = use_yolo
        self._gps = gps

        self.markers_to_track = []
        self.tracked_objects = {}

        # Set the ar marker dictionary
        self._marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        self._show_frames = False
        self._processed_frame = None

        self._marker_mode = True
        self._running = False
        self._last_frame = None
        self._thread = threading.Thread(target=self._tracking_loop, name='AR search')

        self._known_marker_width = 0.020

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

    def start(self, camera: Camera):
        self.camera = camera
        # Initialize cameras
        self.camera.start()
        if self._save_to_disk:
            self._video_writer = cv2.VideoWriter("autonomous.avi", cv2.VideoWriter_fourcc(
                *'MJPG'), 5, (self._frame_width, self._frame_height), False)
            
        self._running = True
        print('starting ar thread')
        self._thread.start()

    def stop(self):
        if self._running:
            self._running = False
            self._thread.join()
            self.camera.stop()
            if self._save_to_disk:
                self._video_writer.release()

    def set_markers_to_track(self, markers: "list[int]"):
        self.markers_to_track = markers

    def any_targets_found(self):
        for marker_id in self.markers_to_track:
            if self.tracked_objects.get(marker_id) is not None:
                return True
            
    def get_closest_target(self):
        closest_marker = None
        closest_distance = 999999999
        for marker_id in self.markers_to_track:
            if self.tracked_objects.get(marker_id) is not None:
                _, distance = self.tracked_objects[marker_id].get_last_position()
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
                    angle, distance = get_marker_location(corner, self._known_marker_width, intrinsic, distortion)
                    print('ad', angle, distance)
                    if self.tracked_objects.get(marker_id[0]) is None:
                        self.tracked_objects[marker_id[0]] = TrackedObject()
                    rover_lat, rover_lon = self._gps.get_position()
                    rover_bearing = self._gps.get_bearing()
                    measured_coord = get_coordinates(rover_lat, rover_lon, distance/1000, rover_bearing + angle)
                    mc_lat, mc_lon = measured_coord
                    mc_lat_m = degrees_to_meters(mc_lat)
                    mc_lon_m = degrees_to_meters(mc_lon)
                    rover_lat_m = degrees_to_meters(rover_lat)
                    rover_lon_m = degrees_to_meters(rover_lon)
                    print('rover', rover_lat_m, rover_lon_m)
                    print('distance', distance)
                    print('mc', mc_lat_m, mc_lon_m)
                    self.tracked_objects[marker_id[0]].update(measured_coord, distance)
            if self._show_frames:
                self._processed_frame = cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
            
    #helper method to convert YOLO detections into the aruco corners format
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

    #id1 is the main ar tag to track, id2 is if you're looking at a gatepost, image is the image to analyze
    # def search_for_markers__(self, image, id2=-1):
    #     # converts to grayscale
    #     cv2.cvtColor(image, cv2.COLOR_RGB2GRAY, image)  
        
    #     self.index1 = -1
    #     self.index2 = -1
    #     bw = image #will hold the black and white image
    #     # tries converting to b&w using different different cutoffs to find the perfect one for the current lighting
    #     for i in range(40, 221, 60):
    #         bw = cv2.threshold(image,i,255, cv2.THRESH_BINARY)[1]
    #         if self.markers_to_track is not None:
    #             print('', end='') #I have not been able to reproduce an error when I have a print statement here so I'm leaving it in    
    #             if id2==-1: #single post
    #                 self.index1 = -1 
    #                 # this just checks to make sure that it found the right marker
    #                 for m in range(len(self.markers_to_track)):  
    #                     if self.markers_to_track[m] == id1:
    #                         self.index1 = m  
    #                         break  
                
    #                 if self.index1 != -1:
    #                     print("Found the correct marker!")
    #                     if self._save_to_disk:
    #                         self._video_writer.write(bw)   #purely for debug   
    #                         cv2.waitKey(1)
    #                     break                    
                    
    #                 else:
    #                     print("Found a marker but was not the correct one") 
                
    #             else: #gate post
    #                 self.index1 = -1
    #                 self.index2 = -1
    #                 if len(self.markers_to_track) == 1: 
    #                    print('Only found marker ', self.markers_to_track[0])
    #                 else:
    #                     for j in range(len(self.markers_to_track) - 1, -1,-1): #I trust the biggest markers the most
    #                         if self.markers_to_track[j][0] == id1:
    #                             self.index1 = j 
    #                         elif self.markers_to_track[j][0] == id2:
    #                             self.index2 = j
    #                 if self.index1 != -1 and self.index2 != -1:
    #                     print('Found both markers!')
    #                     if self._save_to_disk:
    #                         self.videoWriter.write(bw)   #purely for debug   
    #                         cv2.waitKey(1)
    #                     break                        
                     
            # if i == 220:  #did not find any AR markers with any b&w cutoff using aruco                
            #     #Checks to see if yolo can find a tag
            #     if self._use_YOLO:
            #         detections = []
            #         if not self._save_to_disk:
            #             #this is a simpler detection function that doesn't return the image
            #             detections = simple_detection(image, self.network, self.class_names, self.thresh)
            #         else:
            #             #more complex detection that returns the image to be written
            #             image, detections = complex_detection(image, self.network, self.class_names, self.class_colors, self.thresh)
            #         #cv2.imwrite('ar.jpg', image)
            #         for d in detections:
            #             print(d)
                        
            #         if id2 == -1 and len(detections) > 0:
            #             self.corners = self._convert_to_corners(detections, 1)
            #             self.index1 = 0 #Takes the highest confidence ar tag
            #             if self._save_to_disk:
            #                 self._video_writer.write(image)   #purely for debug   
            #                 cv2.waitKey(1)                        
            #         elif len(detections) > 1:
            #             self.corners = self._convertToCorners(detections, 2)
            #             self.index1 = 0 #takes the two highest confidence ar tags
            #             self.index2 = 1
            #             if self._save_to_disk:
            #                 self._video_writer.write(image)   #purely for debug   
            #                 cv2.waitKey(1)
            #         print(self.corners)    
                
                #Not even YOLO saw anything
        #         if self.index1 == -1 or (self.index2 == -1 and id2 != -1): 
        #             if self._save_to_disk:
        #                 self._video_writer.write(image) 
        #                 #cv2.imshow('window', image)
        #                 cv2.waitKey(1)
        #             self.distance_to_marker = -1 
        #             self.angle_to_marker = -999 
        #             return False 
        
        # if id2 == -1:
        
        # if id1==-1:
        #     pass
        # else:
        #     centerXMarker1 = (self.corners[self.index1][0][0][0] + self.corners[self.index1][0][1][0] + \
        #         self.corners[self.index1][0][2][0] + self.corners[self.index1][0][3][0]) / 4
        #     centerXMarker2 = (self.corners[self.index2][0][0][0] + self.corners[self.index2][0][1][0] + \
        #         self.corners[self.index2][0][2][0] + self.corners[self.index2][0][3][0]) / 4
        #     self.angle_to_marker = self._degrees_per_pixel * ((centerXMarker1 + centerXMarker2)/2 - self._frame_width/2) 
        
        #     hAngleToMarker1 = abs(self._v_degrees_per_pixel * (centerXMarker1 - self._frame_width/2))
        #     hAngleToMarker2 = abs(self._v_degrees_per_pixel * (centerXMarker2 - self._frame_width/2))
        #     centerYMarker1 = (self.corners[self.index1][0][0][1] + self.corners[self.index1][0][1][1] + \
        #         self.corners[self.index1][0][2][1] + self.corners[self.index1][0][3][1]) / 4
        #     centerYMarker2 = (self.corners[self.index2][0][0][1] + self.corners[self.index2][0][1][1] + \
        #         self.corners[self.index2][0][2][1] + self.corners[self.index2][0][3][1]) / 4
        #     vAngleToMarker1 = abs(self._v_degrees_per_pixel * (centerYMarker1 - self._frame_height/2))
        #     vAngleToMarker2 = abs(self._v_degrees_per_pixel * (centerYMarker2 - self._frame_height/2))
        #     realFocalLength1 = self._focal_length + (hAngleToMarker1/30) * (self._focal_length_30_h - self._focal_length) + \
        #         (vAngleToMarker1/30) * (self._focal_length_30_v - self._focal_length)
        #     realFocalLength2 = self._focal_length + (hAngleToMarker2/30) * (self._focal_length_30_h - self._focal_length) + \
        #         (vAngleToMarker2/30) * (self._focal_length_30_v - self._focal_length)     
        #     widthOfMarker1 = ((self.corners[self.index1][0][1][0] - self.corners[self.index1][0][0][0]) + \
        #         (self.corners[self.index1][0][2][0] - self.corners[self.index1][0][3][0])) / 2
        #     widthOfMarker2 = ((self.corners[self.index2][0][1][0] - self.corners[self.index2][0][0][0]) + \
        #         (self.corners[self.index1][0][2][0] - self.corners[self.index1][0][3][0])) / 2

        #     #distanceToAR = (knownWidthOfMarker(20cm) * focalLengthOfCamera) / pixelWidthOfMarker
        #     distanceToMarker1 = (self._known_marker_width * realFocalLength1) / widthOfMarker1
        #     distanceToMarker2 = (self._known_marker_width * realFocalLength2) / widthOfMarker2
        #     print(f"1: {distanceToMarker1}, 2: {distanceToMarker2}")
        #     self.distance_to_marker = (distanceToMarker1 + distanceToMarker2) / 2
    
        # return True 


class TrackedObject:
    def __init__(self):
        self._MAX_ESTIMATES = 10
        self._position_estimates = []
        self._position_estimate_length = 0
        self._position_estimate_index = 0
        self._last_update_time = None

    def get_position(self):
        avg_lat = sum([x[0][0] for x in self._position_estimates]) / len(self._position_estimates)
        avg_lon = sum([x[0][1] for x in self._position_estimates]) / len(self._position_estimates)
        return (avg_lat, avg_lon)
    
    def update(self, position, distance):
        self._last_update_time = time()
        if self._position_estimate_length < self._MAX_ESTIMATES:
            self._position_estimates.append((position, distance))
            self._position_estimate_length += 1
        else:
            self._position_estimates[self._position_estimate_index] = (position, distance)
            self._position_estimate_index = (self._position_estimate_index + 1) % self._position_estimate_length

    def get_last_update_time(self):
        return self._last_update_time
    
    def get_last_position(self):
        last_index = (self._position_estimate_index - 1) % self._position_estimate_length
        return self._position_estimates[last_index]
    
    def clear_estimates(self):
        self._position_estimates = []
        self._position_estimate_length = 0
        self._position_estimate_index = 0
        self._last_update_time = None