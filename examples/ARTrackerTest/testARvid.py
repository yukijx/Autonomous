import cv2
import cv2.aruco as aruco
from time import sleep
import argparse

# Parse the command line arguments
parser = argparse.ArgumentParser(description="Test the ARTracker with a video or picture")
parser.add_argument("file", help="The file to test the ARTracker with")
args = parser.parse_args() # parse the arguments

if __name__ == "__main__":    

    # Import either a video or a picture
    if args.file.endswith(".mp4") or args.file.endswith(".avi"):
        file = cv2.VideoCapture(args.file)
    else:
        print("file must be a video, either .mp4 or .avi")
        exit(-1)

    # Play video or display picture
    index = 0
    markerTicks = 0
    soonestFrame = 0
    while True:
        # Create the aruco dictionary
        markerDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Capture and display the unprocessed frame
        #sleep(1/45) # 30 fps NOTE: This does not accurately represent the actual fps of the video
        
        ret, frame = file.read()
        cv2.namedWindow('unprocessed frame', cv2.WINDOW_KEEPRATIO)
        cv2.imshow('unprocessed frame', frame)
        cv2.resizeWindow('unprocessed frame', frame.shape[1] // 2, frame.shape[0] // 2)

        # Process the image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Process image by changing the contrast
        """for i in range(40, 221, 60):
            process_image = cv2.threshold(gray,i,255, cv2.THRESH_BINARY)[1]
            (corners, markerIDs, rejected) = aruco.detectMarkers(process_image, markerDict)
            if markerIDs is not None:
                break
        """

        # Process image using Adaptive Thresholding
        process_image = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 25, 3)
        
        # Show the processed image
        cv2.namedWindow('processed image', cv2.WINDOW_KEEPRATIO)
        cv2.imshow('processed image', process_image)
        cv2.resizeWindow('processed image', process_image.shape[1] // 2, process_image.shape[0] // 2)

        # Detect the markers
        (corners, markerIDs, rejected) = aruco.detectMarkers(process_image, markerDict)
        if (markerIDs is not None) and (markerIDs[0] == 1):
            if markerTicks == 0:
                soonestFrame = index
            print("Marker found")
            print("Found IDs:" + str(markerIDs))
            markerTicks += 1

        print("Frame " + str(index))
        index += 1
        # Exit the program if the user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Number of times marker was found:" + str(markerTicks))
            print("Soonest frame:" + str(soonestFrame))
            break
        

