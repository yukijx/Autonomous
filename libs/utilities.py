from math import cos, radians, degrees, sin, atan2, pi, sqrt, asin
import numpy as np
import cv2
import socket
from argparse import ArgumentParser

def get_marker_location(corners, marker_size, intrinsic, distortion):
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, intrinsic, distortion)

    # this distance to the marker is the magnitude of the translation vector
    distance_to_marker = np.linalg.norm(tvecs[0][0])

    # this angle is the angle between the camera's x-axis and the marker's z-axis
    angle = degrees(atan2(tvecs[0][0][0], tvecs[0][0][2]))

    return angle, distance_to_marker


def abs_clamp(n: float, minn: float, maxn: float) -> float:
    """
    Clamps a number between a minimum and maximum value, but keeps the sign of the number

    Args:
        n (float): Number to clamp
        minn (float): Minimum value
        maxn (float): Maximum value

    Returns:
        float: Clamped number
    """
    sign = 1 if n > 0 else -1
    return max(min(maxn, abs(n)), minn) * sign

def send_udp(host: str, port: int, message: bytearray) -> None: 
    """
    Sends a message over UDP to a specific host and port
    
    Args:
        host (str): IP address of host
        port (int): Port of host
        message (bytearray): Message to send
    """
    #sends a message over UDP to a specific host and port
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect((host, port))
            s.sendall(message)
        except OSError:
            pass
        #TODO: add better error handling or at least reporting

def parse_arguments():
    """
    Parses command line arguments
    
    Returns:
        args: Arguments
    """
    arg_parser = ArgumentParser()
    arg_parser.add_argument("-ll", "--latLong", required=True, type=str, help="takes a filename for a text file, then reads that file for latlong coordinates")
    arg_parser.add_argument("-c --camera", default=0, type=int, help="takes a number representing which camera to use")
    arg_parser.add_argument("-t1", "--tag1", default=-1, type=int, help="takes the id value of the first tag, defaults to -1 if id not assigned")
    arg_parser.add_argument("-t2", "--tag2", default=-1, type=int, help="takes the id value of the second tag, defaults to -1 if id not assigned")
    args = arg_parser.parse_args()
    return args

def parse_latlong_file(filename) -> list:
    """
    Parses a file containing latitude and longitude coordinates
    
    Args:
        filename (str): Name of file to parse
        
    Returns:
        list: List of coordinates
    """
    locations = []
    with open(filename) as f:
        line_number = 0
        for line in f:
            line_number += 1
            try:
                coords = [float(item.replace('\ufeff',"")) for item in line.strip().split()]
            except:
                print("Parse Error on line " + str(line_number) + ": Please enter <lat long>")
                break
            else:
                if len(coords) != 2:
                    print("Error on line " + str(line_number) + ": Insufficient number of coordinates. Please enter <lat long>")
                    break        
                locations.append(coords)

    return locations

def calc_average_bearing(bearings: list) -> float:
    """
    Calculates the average bearing from a list of bearings

    Args:
        bearings (list): List of bearings between -180 and 180

    Returns:
        float: Average bearing
    """
    # if the average magnitude is greater than 90, then the average
    # is on the bottom side of the circle, so we need to
    # add 360 to all the negative bearings in the queue to properly
    # calculate the average
    average_mag = sum([abs(x) for x in bearings]) / len(bearings)
    if average_mag > 90:
        temp_bearings = [x + 360 if x < 0 else x for x in bearings]
        avg = (sum(temp_bearings) / len(bearings))
        # if the result is over 180, must be mapped back to -180 to 180
        if avg > 180:
            avg = -(360-avg)
    else:
        avg = sum(bearings) / len(bearings)
    return avg

def distance_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """ 
    Returns distance in kilometers between given latitude and longitude

    Args:
        lat1 (float): Latitude of first point
        lon1 (float): Longitude of first point
        lat2 (float): Latitude of second point
        lon2 (float): Longitude of second point

    Returns:
        float: Distance in kilometers between given latitude and longitude
    """
    EARTH_RADIUS = 6371.301
    delta_lat = (lat2 - lat1) * (pi/180.0)
    delta_lon = (lon2 - lon1) * (pi/180.0)

    a = sin(delta_lat/2) * sin(delta_lat/2) + cos(lat1 * (pi/180.0)) * cos(lat2 * (pi/180.0)) * sin(delta_lon/2) * sin(delta_lon/2)
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return EARTH_RADIUS * c

def calc_bearing(lat1:float, lon1:float, lat2:float, lon2:float) -> float:
    """
    Calculates bearing between two points. (0 is North, 90 is East, +/-180 is South, -90 is West)

    Args:
        lat1 (float): Latitude of first point
        lon1 (float): Longitude of first point
        lat2 (float): Latitude of second point
        lon2 (float): Longitude of second point

    Returns:
        float: Bearing between two points. (0 is North, 90 is East, +/-180 is South, -90 is West)
    """
    x = cos(lat2 * (pi/180.0)) * sin((lon2-lon1) * (pi/180.0))
    y = cos(lat1 * (pi/180.0)) * sin(lat2 * (pi/180.0)) - sin(lat1 * (pi/180.0)) * cos(lat2 * (pi/180.0)) * cos((lon2-lon1) * (pi/180.0))
    return (180.0/pi) * atan2(x,y)

def get_coordinates(lat: float, lon: float, distance: float, bearing: float) -> "(float, float)":
    """
    Calculate latitude and longitude given distance (in km) and bearing (in degrees)

    Args:
        lat (float): Latitude of the starting point
        lon (float): Longitude of the starting point
        distance (float): Distance to the destination point in kilometers
        bearing (float): Bearing to the destination point in degrees

    Returns:
        tuple: A tuple containing the latitude and longitude of the destination point
    """
    # https://stackoverflow.com/questions/7222382/get-lat-long-given-current-point-distance-and-bearing
    EARTH_RADIUS = 6371.301
    brng = radians(bearing)      # Assuming bearing is in degrees
    d = distance

    lat1 = radians(lat)   # Current lat point converted to radians
    lon1 = radians(lon)  # Current long point converted to radians

    lat2 = asin(sin(lat1)*cos(d/EARTH_RADIUS) + 
                cos(lat1)*sin(d/EARTH_RADIUS)*cos(brng))
    lon2 = lon1 + atan2(sin(brng)*sin(d/EARTH_RADIUS)*cos(lat1),
                        cos(d/EARTH_RADIUS)-sin(lat1)*sin(lat2))
    return degrees(lat2), degrees(lon2)

def degrees_to_meters(degrees: float) -> float:
    """
    Converts degrees to meters

    Args:
        degrees (float): Degrees to convert

    Returns:
        float: Meters
    """
    return degrees * 111139

def meters_to_degrees(meters: float) -> float:
    """
    Converts meters to degrees

    Args:
        meters (float): Meters to convert

    Returns:
        float: Degrees
    """
    return meters / 111139