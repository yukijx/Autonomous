import os
path = (os.path.dirname(os.path.abspath(__file__)))
import argparse
import configparser
from libs import Drive
from libs.Rover import Rover
from libs.utilities import parse_arguments, parse_latlong_file
import threading
from time import sleep

CONFIG_FILE = 'config.ini'

def main():
    print("Welcome to the REMI Autonomous Traversal System (RATS)!")

    args = parse_arguments()
    locations = parse_latlong_file(args.latLong)
    camera = args.camera
    id1 = args.tag1
    id2 = args.tag2

    config = configparser.ConfigParser(allow_no_value=True)
    if not config.read(CONFIG_FILE):
        print("DID NOT OPEN CONFIG")
        exit(-2)
    mbed_ip = str(config['CONNECTION']['MBED_IP'])
    mbed_port = int(config['CONNECTION']['MBED_PORT'])
    swift_ip = str(config['CONNECTION']['SWIFT_IP'])
    swift_port = int(config['CONNECTION']['SWIFT_PORT'])
    
    artracker_settings = config['ARTRACKER']

    rover = Rover()
    rover.start_gps(swift_ip, swift_port)
    rover.start_wheels(mbed_ip, mbed_port)
    rover.start_lights(mbed_ip, mbed_port)

    rover.start_navigation(locations)
    rover.drive()

#Gets a list of coordinates from user and drives to them and then tracks the tag
#Set id1 to -1 if not looking for a tag
def drive(rover):
    global flashing
    idList = [-1,-1]
    locations = []

    if args.ids is not None:
        for i in range(len(args.ids)):
            idList[i] = args.ids[i]
    
    id1 = idList[0]
    id2 = idList[1]

    flashing = False
    UDPOut.sendLED(mbedIP, mbedPort, 'r')
    found = rover.driveAlongCoordinates(locations,id1, id2)
    
    if id1 != -1:
        rover.trackARMarker(id1, id2)

if __name__ == "__main__":
    main()

def shit():
    os.chdir(path)
    print(os.getcwd())
    #user input (args from system)
    if args.cameraInput is None:
        print("ERROR: must at least specify one camera")
        exit(-1)
    
    #gets the mbed ip and port
    config = configparser.ConfigParser(allow_no_value=True)
    if not config.read('config.ini'):
        print("DID NOT OPEN CONFIG")
        exit(-2)
    mbedIP = str(config['CONFIG']['MBED_IP'])
    mbedPort = int(config['CONFIG']['MBED_PORT'])

    rover = Drive.Navigation(50, args.cameraInput)
    
    drive(rover)