"""
Authors: Masafumi Endo
Date: 02/01/2021
Version: 1.0
Objective: Implement a greedy solver for the discrete problem
"""

import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
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

    def getActionGreedy(self, robot, map, map_gt):
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
                if map[y, x] != 128:
                    mask[y, x] = 1
        # creates an estimate of what the world looks like before moving
        map_prediction = self.uNet.runNetwork(map, mask)
        # initialize dictionary
        dict_info_quality = {}

        # add possible movements w/ init.
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
            dict_info_quality[direction] = self._calc_info_quality(location_curr, direction, mask, map, map_prediction)
        # remove the direction where no new information can be obtained
        dict_info_quality = {k: dict_info_quality[k] for k in dict_info_quality if not np.isnan(dict_info_quality[k])}
        # determine direction that gains the maximal information
        direction = max(dict_info_quality, key=dict_info_quality.get)

        self._summarize_progress(location_curr, dict_info_quality, direction, map_gt, map, map_prediction)

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
        mask = np.zeros((28, 28))
        for x in range(0, 28):
            for y in range(0, 28):
                if map_[y, x] != 128:
                    mask[y, x] = 1
        # creates an estimate of what the world looks like after moving
        map_hallucinate = self.uNet.runNetwork(map_, mask)
        # get a guess of what "world" we are in before and after taking action
        char_bef_action = self.classNet.runNetwork(map_prediction)
        char_aft_action = self.classNet.runNetwork(map_hallucinate)

        # calculate probability
        prob_bef = self.softmax(torch.from_numpy(char_bef_action).clone()).numpy()
        prob_aft = self.softmax(torch.from_numpy(char_aft_action).clone()).numpy()

        # calculate entropy
        entropy_bef = self._calc_entropy(prob_bef[0])
        entropy_aft = self._calc_entropy(prob_aft[0])
        # calculate information gain
        if entropy_bef == entropy_aft:
            info_quality = np.nan
        else:
            info_quality = entropy_bef - entropy_aft

        return info_quality

    def _calc_entropy(self, prob):
        """ Calculate entropy from probabilities
        :return:
        """
        entropy = 0
        for prob_ in prob:
            entropy += - np.log2(prob_)

        return entropy

    def _summarize_progress(self, location, dict_info_quality, direction, map_gt, map, map_prediction):
        print(" ")
        print(f"before one iteration of the game -> Robot at: ({location[0]}, {location[1]})")
        print(dict_info_quality)
        print("{0} is selected as a next robot direction".format(direction))

        char = self.classNet.runNetwork(map_prediction)
        estimated_digit = char.argmax()
        
        print("{0} is the predicted value based on the exploration so far".format(estimated_digit))

        fig, (ax1, ax2, ax3) = plt.subplots(ncols=3)
        pos = ax1.imshow(map_gt)
        pos = ax2.imshow(map)
        pos = ax3.imshow(map_prediction, cmap='gray')
        plt.show()

    # def _get_info(self, mask, map, map_prediction, location):
    #     """ Get information at the given location
    #     :return: info_quality
    #     """
    #     info_quality = 0
    #     # iterate through all nearby squares
    #     for x in range(-1, 2):
    #         for y in range(-1, 2):
    #             # if the robot is off the map, do nothing
    #             if location[0]+x > 27 or location[0]+x < 0 or location[1]+y > 27 or location[1]+y < 0:
    #                 continue
    #             # otherwise update info_quality if there are unexplored areas
    #             else:
    #                 # 0 is unexplored
    #                 if mask[location[1]+y, location[0]+x] == 0:
    #                     info_quality += map_prediction[location[1]+y, location[0]+x]
    #                 else:
    #                     continue
    #
    #     return info_quality