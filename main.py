import os
from libs.Rover import Rover
from libs.utilities import parse_arguments, parse_latlong_file, parse_config_file
from time import sleep

path = (os.path.dirname(os.path.abspath(__file__)))

CONFIG_FILE = 'config.ini'
CONFIG_PATH = os.path.join(path, 'cfg', CONFIG_FILE)

def main():
    print("Welcome to the REMI Autonomous Traversal System (RATS)!")

    args = parse_arguments()
    locations = parse_latlong_file(path, args.latLong)
    camera = args.camera
    id1 = args.tag1
    id2 = args.tag2

    config = parse_config_file(CONFIG_PATH)
    mbed_ip = str(config['CONNECTION']['MBED_IP'])
    mbed_port = int(config['CONNECTION']['MBED_PORT'])
    swift_ip = str(config['CONNECTION']['SWIFT_IP'])
    swift_port = int(config['CONNECTION']['SWIFT_PORT'])
    

    rover = Rover()
    rover.start_gps(swift_ip, swift_port)
    rover.start_wheels(mbed_ip, mbed_port)
    rover.start_lights(mbed_ip, mbed_port)

    rover.start_navigation(locations, False)

if __name__ == "__main__":
    main()