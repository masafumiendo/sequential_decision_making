"""
Authors: Masafumi Endo
Date: 02/03/2021
Version: 1.0
Objective: Modify "GameClass" from the original class
"""

import numpy as np
import torch
from GameClass import Game
from networkFolder.functionList import WorldEstimatingNetwork, DigitClassificationNetwork


class GreedyGame(Game):
    def __init__(self, truthMap, digit, navigator, robot, specified_prob):
        super().__init__(truthMap, digit, navigator, robot)

        self.specified_prob = specified_prob
        self.flag_greedy = True
        self.uNet = WorldEstimatingNetwork()
        self.classNet = DigitClassificationNetwork()
        self.softmax = torch.nn.Softmax(dim=1)

    def tick(self):
        self.iterations += 1
        # Generate an action for the robot
        if self.flag_greedy:
            action = self.navigator.getActionGreedy(self.robot, self.exploredMap, self.truthMap)
        else:
            action = self.navigator.getActionShortest(self.robot, self._goal)
        # Move the robot
        self.robot.move(action)
        # Update the explored map based on robot position
        self._updateMap(self.robot, self.exploredMap, self.truthMap)

        # creates an estimate of what the world looks like after moving
        mask = np.zeros((28, 28))
        for x in range(0, 28):
            for y in range(0, 28):
                if self.exploredMap[y, x] != 128:
                    mask[y, x] = 1
        image = self.uNet.runNetwork(self.exploredMap, mask)
        # creates an estimate of what world we are in
        char = self.classNet.runNetwork(image)
        estimated_digit = char.argmax()

        # calculate probability to check condition
        char_tensor = torch.from_numpy(char).clone()
        prob_tensor = self.softmax(char_tensor)
        prob = prob_tensor.numpy()
        prob_max = prob[0][estimated_digit]

        # check condition to end up greedy exploration
        if estimated_digit == self._digit and prob_max >= self.specified_prob:
            # end greedy exploration
            self.flag_greedy = False

        if self.robot.getLoc() == self._goal:
            self.score += 100
            return True
        else:
            self.score -= 1
            return False