"""
Authors: Masafumi Endo
Date: 02/01/2021
Version: 1.0
Objective: Implement a greedy solver for the discrete problem
"""

import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
from networkFolder.functionList import WorldEstimatingNetwork, DigitClassificationNetwork

class GreedyNavigator:
    def __init__(self):
        # init NN that estimates the world
        self.uNet = WorldEstimatingNetwork()

    def getAction(self, robot, map, map_gt):
        """ Greedily select a valid direction for the robot to travel
        Hint: The robot should look one step ahead and move to the location that gains the maximal information based on
        the neural network prediction.
        :return: direction
        """

        # This loop shows how you can create a mask, an grid of 0s and 1s
        # where 0s represent unexplored areas and 1s represent explored areas
        # This mask is used by the world estimating network
        mask = np.zeros((28, 28))
        for x in range(0, 28):
            for y in range(0, 28):
                if map[x, y] != 128:
                    mask[x, y] = 1
        # creates an estimate of what the world looks like before moving
        map_prediction = self.uNet.runNetwork(map, mask)
        # initialize dictionary
        dict_info_quality = {}

        # add possible movements w/ zero info quality
        location_curr = robot.getLoc()
        if location_curr[0] - 1 >= 0:
            dict_info_quality['left'] = 0
        if location_curr[0] + 1 <= 27:
            dict_info_quality['right'] = 0
        if location_curr[1] + 1 <= 27:
            dict_info_quality['down'] = 0
        if location_curr[1] - 1 >= 0:
            dict_info_quality['up'] = 0

        fig, (ax1, ax2, ax3) = plt.subplots(ncols=3)
        pos = ax1.imshow(map_gt)
        pos = ax2.imshow(map)
        pos = ax3.imshow(map_prediction, cmap='gray')
        plt.show()

        # calculate info quality for possible movements
        for direction in dict_info_quality:
            dict_info_quality[direction] = self._calc_info_quality(location_curr, direction, mask, map, map_prediction)
        # determine direction that gains the maximal information
        direction = max(dict_info_quality, key=dict_info_quality.get)
        print(location_curr)
        print(dict_info_quality)
        print(direction)
        return direction

    def _calc_info_quality(self, location_curr, direction, mask, map, map_prediction):
        """ Calculate information quality for the possible movement
        Hint: Consider the values of the pixels in the predicted map.
        :return: info_quality
        """

        location_next = self._get_next_location(location_curr, direction)
        info_quality = self._get_info(mask, map, map_prediction, location_next)

        return info_quality

    def _get_next_location(self, location_curr, direction):
        """ Get next location of the robot
        :return: location_next
        """
        xLoc = location_curr[0]
        yLoc = location_curr[1]

        if direction == 'left':
            xLoc = xLoc - 1
        elif direction == 'right':
            xLoc = xLoc + 1
        elif direction == 'down':
            yLoc = yLoc + 1
        elif direction == 'up':
            yLoc = yLoc - 1
        else:
            raise ValueError(f"Robot received invalid direction: {direction}!")

        location_next = (xLoc, yLoc)

        return location_next
    
    def _get_info(self, mask, map, map_prediction, location):
        """ Get information at the given location
        :return: info_quality
        """
        info_quality = 0
        # iterate through all nearby squares
        for x in range(-1, 2):
            for y in range(-1, 2):
                # if the robot is off the map, do nothing
                if location[0]+x > 27 or location[0]+x < 0 or location[1]+y > 27 or location[1]+y < 0:
                    continue
                # otherwise update info_quality if there are unexplored areas
                else:
                    if mask[location[0]+x, location[1]+y] == 0:
                        info_quality += map_prediction[location[0]+x, location[1]+y]
                    else:
                        info_quality += map[location[0]+x, location[1]+y]

        return info_quality