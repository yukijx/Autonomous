import cv2
import cv2.aruco as aruco
import numpy as np
import configparser
import os


def preprocess_image(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Enhance contrast using histogram equalization
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    equalized = clahe.apply(gray)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(equalized, (5, 5), 0)
    
    # Apply adaptive thresholding to segment the image
    print(type(blurred))
    thresholded = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 4)
    
    return thresholded

def configCam(cam, configFile):
        # Open the config file
        config = configparser.ConfigParser(allow_no_value=True)
        if not config.read(configFile):
            print(f"ERROR OPENING AR CONFIG:", end="")
            if os.path.isabs(configFile):
                print(configFile)
            else:
                print("{os.getcwd()}/{configFile}")
            exit(-2)

        # Set variables from the config file
        degreesPerPixel = float(config['ARTRACKER']['DEGREES_PER_PIXEL'])
        vDegreesPerPixel = float(config['ARTRACKER']['VDEGREES_PER_PIXEL'])
        focalLength = float(config['ARTRACKER']['FOCAL_LENGTH'])
        focalLength30H = float(config['ARTRACKER']['FOCAL_LENGTH30H'])
        focalLength30V = float(config['ARTRACKER']['FOCAL_LENGTH30V'])
        knownMarkerWidth = float(config['ARTRACKER']['KNOWN_TAG_WIDTH'])
        format = config['ARTRACKER']['FORMAT']
        frameWidth = int(config['ARTRACKER']['FRAME_WIDTH'])
        frameHeight = int(config['ARTRACKER']['FRAME_HEIGHT'])
        
        # Set the camera properties
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, frameWidth)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1) # greatly speeds up the program but the writer is a bit wack because of this
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(format[0], format[1], format[2], format[3]))

if __name__ == "__main__":
    # Create the camera object
    cam = cv2.VideoCapture(1)
    configCam(cam, "config.ini")

    # Create the aruco dictionary
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    while True:
        # Read the image from the camera and convert to grayscale
        ret, frame = cam.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the markers
        print(type(gray))
        adapted_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 3)
        
        # Display the image
        cv2.imshow('adapted thresh', adapted_thresh)
        cv2.waitKey(1)
        print(adapted_thresh[:][0])
        (corners, markerIDs, rejected) = aruco.detectMarkers(adapted_thresh, markerDict)  
        if len(corners) == 1 and markerIDs[0] == 1:
            print(markerIDs)
        else:
            print("No markers found")
            cv2.imshow('adapted thresh', adapted_thresh)
            cv2.waitKey(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

