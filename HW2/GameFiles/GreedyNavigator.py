"""
Authors: Masafumi Endo
Date: 02/01/2021
Version: 1.0
Objective: Implement a greedy solver for the discrete problem
"""

import numpy as np
from PIL import Image
from networkFolder.functionList import WorldEstimatingNetwork, DigitClassificationNetwork

class GreedyNavigator:
    def __init__(self):
        # init NN that estimates the world
        self.uNet = WorldEstimatingNetwork()

    def getAction(self, robot, map):
        """ Greedily select a valid direction for the robot to travel
        Hint: The robot should look one step ahead and move to the location that gains the maximal information based on
        the neural network prediction.
        :return: direction
        """

        # creates an estimate of what the world looks like before moving
        mask = np.zeros((28, 28))
        for x in range(0, 28):
            for y in range(0, 28):
                if map[x, y] != 128:
                    mask[x, y] = 1
        image = self.uNet.runNetwork(map, mask)
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

        # calculate info quality for possible movements
        for direction in dict_info_quality:
            dict_info_quality[direction] = self._calc_info_quality(location_curr, direction, image)
        # determine direction that gains the maximal information
        direction = max(dict_info_quality, key=dict_info_quality.get)
        return direction

    def _calc_info_quality(self, location_curr, direction, image):
        """ Calculate information quality for the possible movement
        Hint: Consider the values of the pixels in the prediction image.
        :return: info_quality
        """

        location_next = self._get_next_location(location_curr, direction)
        info_quality = image[location_next[0], location_next[1]]

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