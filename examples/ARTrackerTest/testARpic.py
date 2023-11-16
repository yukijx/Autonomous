import cv2
import cv2.aruco as aruco
import argparse

# Parse the command line arguments
parser = argparse.ArgumentParser(description="Test the ARTracker with a picture")
parser.add_argument("file", help="The file to test the ARTracker with")
args = parser.parse_args() # parse the arguments

if __name__ == "__main__":
    # Read the image from the file
    if args.file.endswith(".jpg") or args.file.endswith(".png"):
        image = cv2.imread(args.file)
    else:
        print("ERROR: File must be a jpg or png")
        exit(-1)
    
    # Show image
    cv2.namedWindow('unprocessed image', cv2.WINDOW_KEEPRATIO)
    cv2.imshow('unprocessed image', image)
    cv2.resizeWindow('unprocessed image', image.shape[1] // 2, image.shape[0] // 2)

    # Process the image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    adapted_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 3)
    cv2.namedWindow('adapted thresh', cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow('adapted thresh', image.shape[1] // 2, image.shape[0] // 2)
    cv2.imshow('adapted thresh', adapted_thresh)

    # Detect the markers
    markerDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    (corners, markerIDs, rejected) = aruco.detectMarkers(adapted_thresh, markerDict)
    if (markerIDs is not None) :
        print("Marker found")
        print("Found IDs:" + markerIDs)
    else:
        print("No markers found")
    
    if cv2.waitKey(0) & 0xFF == ord('q'):
        exit(0)

