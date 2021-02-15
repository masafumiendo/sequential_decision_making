"""
Authors: Masafumi Endo
Date: 02/01/2021
Version: 1.0
Objective: Implement a greedy solver for the discrete problem
"""

import copy
from random import randint
import numpy as np
from matplotlib import pyplot as plt
import torch
from networkFolder.functionList import WorldEstimatingNetwork, DigitClassificationNetwork

class GreedyNavigator:
    def __init__(self):
        # init NN that estimates the world
        self.uNet = WorldEstimatingNetwork()
        self.classNet = DigitClassificationNetwork()
        self.softmax = torch.nn.Softmax(dim=1)

    def getActionShortest(self, robot, goal):
        """ Select a valid direction for the robot to travel towards the goal position
        :return: direction
        """

        location_curr = robot.getLoc()
        # x-axis direction
        if location_curr[0] - goal[0] > 0:
            direction = 'left'
        elif location_curr[0] - goal[0] < 0:
            direction = 'right'
        # y-axis direction
        if location_curr[1] - goal[1] > 0:
            direction = 'up'
        elif location_curr[1] - goal[1] < 0:
            direction = 'down'

        return direction

    def getActionRandom(self, robot):
        """ Select a valid direction for the robot to travel randomly
        :return: direction
        """
        direction = None

        while direction is None:

            randNumb = randint(0,3)

            if randNumb == 0:
                if robot.getLoc()[0] - 1 >= 0:
                    direction = 'left'
            if randNumb == 1:
                if robot.getLoc()[0] + 1 <= 27:
                    direction = 'right'
            if randNumb == 2:
                if robot.getLoc()[1] + 1 <= 27:
                    direction = 'down'
            if randNumb == 3:
                if robot.getLoc()[1] - 1 >= 0:
                    direction = 'up'

        return direction

    def getActionGreedy(self, robot, map):
        """ Greedily select a valid direction for the robot to travel
        Hint: The robot should look one step ahead and move to the location that gains the maximal information based on
        the neural network prediction.
        :return: direction
        """

        # create a mask for estimation
        mask = np.zeros((28, 28))
        for x in range(0, 28):
            for y in range(0, 28):
                if map[y, x] != 128:
                    mask[y, x] = 1
        # creates an estimate of what the world looks like before moving
        map_prediction = self.uNet.runNetwork(map, mask)
        # get entropy for the current state based on map_prediction
        entropy_curr = self._get_entropy_curr(map_prediction)

        # initialize dictionary
        dict_info_gain = {}

        # add possible movements w/ init.
        location_curr = robot.getLoc()
        if location_curr[0] - 1 >= 0:
            dict_info_gain['left'] = 0
        if location_curr[0] + 1 <= 27:
            dict_info_gain['right'] = 0
        if location_curr[1] + 1 <= 27:
            dict_info_gain['down'] = 0
        if location_curr[1] - 1 >= 0:
            dict_info_gain['up'] = 0

        # calculate info gain for possible movements
        for direction in dict_info_gain:
            location_next = self._get_next_location(location_curr, direction)
            entropy_next = self._get_entropy_next(mask, map, map_prediction, location_next)
            if entropy_curr == entropy_next:
                dict_info_gain[direction] = np.nan
            else:
                dict_info_gain[direction] = entropy_curr - entropy_next

        # remove the direction where no new information can be obtained
        dict_info_gain = {k: dict_info_gain[k] for k in dict_info_gain if not np.isnan(dict_info_gain[k])}

        if len(dict_info_gain) > 0:
            # determine direction that gains the maximal information
            direction = max(dict_info_gain, key=dict_info_gain.get)
            flag_stuck = False
        else:
            direction = self.getActionRandom(robot)
            flag_stuck = True

        return direction, flag_stuck

    def _get_entropy_curr(self, map_prediction):
        """ Get entropy for the current state and estimated map
        :return: entropy
        """

        # get a guess of what "world" we are in
        char = self.classNet.runNetwork(map_prediction)
        # calculate probability
        prob = self.softmax(torch.from_numpy(char).clone()).numpy()
        # calculate entropy
        entropy = self._calc_entropy(prob[0])

        return entropy

    def _get_entropy_next(self, mask, map, map_prediction, location):
        """ Get entropy for the next state and hallucinate map
        :return: entropy
        """
        map_ = np.copy(map)
        # update map with predicted pixel values
        for x in range(-1, 2):
            for y in range(-1, 2):
                # if the robot is off the map, do nothing
                if location[0]+x > 27 or location[0]+x < 0 or location[1]+y > 27 or location[1]+y < 0:
                    continue
                # Otherwise update the explored map with the predicted value of the map
                else:
                    # 0 is unexplored
                    if mask[location[1]+y, location[0]+x] == 0:
                        map_[location[1]+y, location[0]+x] = map_prediction[location[1]+y, location[0]+x]
                    else:
                        continue

        # creates an mask for NN prediction
        mask_ = np.zeros((28, 28))
        for x in range(0, 28):
            for y in range(0, 28):
                if map_[y, x] != 128:
                    mask_[y, x] = 1
        # creates an estimate of what the world looks like after moving
        map_hallucinate = self.uNet.runNetwork(map_, mask_)

        # get a guess of what "world" we are in
        char = self.classNet.runNetwork(map_hallucinate)
        # calculate probability
        prob = self.softmax(torch.from_numpy(char).clone()).numpy()
        # calculate entropy
        entropy = self._calc_entropy(prob[0])

        return entropy

    def _calc_entropy(self, prob):
        """ Calculate entropy from probabilities
        :return: entropy
        """
        entropy = 0
        for prob_ in prob:
            entropy += - prob_ * np.log2(prob_)

        return entropy

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