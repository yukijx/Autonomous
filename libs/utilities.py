from math import cos, radians, degrees, sin, atan2, pi, sqrt, asin
import socket
from argparse import ArgumentParser

def abs_clamp(n, minn, maxn):
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

def send_udp(host,port,message): 
    #sends a message over UDP to a specific host and port
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((host, port))
        s.sendall(message)

def parse_arguments():
    #parses the arguments from the command line
    arg_parser = ArgumentParser()
    arg_parser.add_argument("-ll", "--latLong", required=True, type=str, help="takes a filename for a text file, then reads that file for latlong coordinates")
    arg_parser.add_argument("-c --camera", default=0, type=int, help="takes a number representing which camera to use")
    arg_parser.add_argument("-t1", "--tag1", default=-1, type=int, help="takes the id value of the first tag, defaults to -1 if id not assigned")
    arg_parser.add_argument("-t2", "--tag2", default=-1, type=int, help="takes the id value of the second tag, defaults to -1 if id not assigned")
    args = arg_parser.parse_args()
    return args

def parse_latlong_file(filename):
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



# Class that computes functions related to gps locations
class Location:
    EARTH_RADIUS = 6371.301
    @classmethod
    def distance_to(cls, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
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
        delta_lat = (lat2 - lat1) * (pi/180.0)
        delta_lon = (lon2 - lon1) * (pi/180.0)

        a = sin(delta_lat/2) * sin(delta_lat/2) + cos(lat1 * (pi/180.0)) * cos(lat2 * (pi/180.0)) * sin(delta_lon/2) * sin(delta_lon/2)
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        return cls.EARTH_RADIUS * c

    @staticmethod
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

    @staticmethod
    def get_coordinates(lat: float, lon: float, distance: float, bearing: float) -> (float, float):
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
        R = 6371.301
        brng = radians(bearing)      # Assuming bearing is in degrees
        d = distance

        lat1 = radians(lat)   # Current lat point converted to radians
        lon1 = radians(lon)  # Current long point converted to radians

        lat2 = asin(sin(lat1)*cos(d/R) + 
                    cos(lat1)*sin(d/R)*cos(brng))
        lon2 = lon1 + atan2(sin(brng)*sin(d/R)*cos(lat1),
                            cos(d/R)-sin(lat1)*sin(lat2))
        return degrees(lat2), degrees(lon2)
    
    @staticmethod
    def degrees_to_meters(degrees: float) -> float:
        """
        Converts degrees to meters

        Args:
            degrees (float): Degrees to convert

        Returns:
            float: Meters
        """
        return degrees * 111139
    
    @staticmethod
    def meters_to_degrees(meters: float) -> float:
        """
        Converts meters to degrees

        Args:
            meters (float): Meters to convert

        Returns:
            float: Degrees
        """
        return meters / 111139