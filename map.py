#!/usr/bin/env python3

'''
rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar

Copyright (C) 2018 Simon D. Levy
This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

MAP_SIZE_PIXELS = 500 #in pixels
MAP_SIZE_METERS = 15 #in meters
LIDAR_DEVICE = 'COM6'

# Ideally we could use all 250 or so samples that the RPLidar delivers in one
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES = 180

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
from roboviz import MapVisualizer
import cv2 as cv
import numpy as np

if __name__ == '__main__':

    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar
    iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles = None

    # First scan is crap, so ignore it
    next(iterator)

    while True:

        # Extract (quality, angle, distance) triples from current scan
        items = [item for item in next(iterator)]

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles = [item[1] for item in items]

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles = angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            slam.update(previous_distances, scan_angles_degrees=previous_angles)

        # Get current robot position
        x, y, theta = slam.getpos()

        x_pix, y_pix = (x/1000)*(MAP_SIZE_PIXELS/MAP_SIZE_METERS), (y/1000)*(MAP_SIZE_PIXELS/MAP_SIZE_METERS)


        #print("Coord: ", x_pix, y_pix, theta)


        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        #img = cv.imread(mapbytes, 0)
        #decoded = cv.imdecode(np.frombuffer(mapbytes, np.uint8), -1)

        mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

        map_RGB = np.dstack([mapimg, mapimg, mapimg])

        #radius = 10
        #xDir = round(radius*math.cos(theta)+x_pix)
        #yDir = round(radius*math.sin(theta)+y_pix)


        map_RGB = cv.circle(map_RGB, (round(x_pix), round(y_pix)), 10, (0, 0, 255), -1)
        #map_RGB = cv.circle(map_RGB, (xDir, yDir), 5, (255, 0, 0), -1)
        #rot = cv.getRotationMatrix2D((MAP_SIZE_PIXELS/2, MAP_SIZE_PIXELS/2), 90, 1)

        #rotate map so that top is front
        #map_RGB = cv.warpAffine(map_RGB,rot,mapimg.shape)

        map_RGB = cv.putText(map_RGB, 'Wisp Map - Size(m): ' + str(MAP_SIZE_METERS) + 'x' + str(MAP_SIZE_METERS), (1,30), cv.FONT_HERSHEY_SIMPLEX,
                            1, (255,0,0), 2, cv.LINE_AA)

        map_RGB = cv.putText(map_RGB, '(X,Y,D)(px): ' + str(round(x_pix)) + ',' + str(round(y_pix)) + ',' + str(round(theta)) + 'deg', (1,60), cv.FONT_HERSHEY_SIMPLEX,
                             1, (0,255,0),1, cv.LINE_AA)


        cv.imshow("Map", map_RGB)
        cv.waitKey(100)

        #mapMAT = cv.fromarray(mapimg)
        #print(type(mapMAT))
        #cv.imshow("Map", img)


        # Display map and robot pose, exiting gracefully if user closes it
        #if not viz.display(x / 1000., y / 1000., theta, mapbytes):
        #    exit(0)

    # Shut down the lidar connection
    lidar.stop()
    lidar.disconnect()