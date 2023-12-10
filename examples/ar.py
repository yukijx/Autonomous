from time import sleep

import os
from libs import ARTracker

tracker = ARTracker.ARTracker(['/dev/video1'], save_to_disk=False, use_yolo = False) #ARTracker requires a list of camera files

while True:
    tracker.findMarker(1)#, id2 = 1)
    print('Distance (in cm): ', tracker.distance_to_marker)
    print('Angle: ', tracker.angle_to_marker)
    sleep(.5)
