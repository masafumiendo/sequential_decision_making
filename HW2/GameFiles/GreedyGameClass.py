"""
Authors: Masafumi Endo
Date: 02/03/2021
Version: 1.0
Objective: Modify "GameClass" from the original class
"""

import numpy as np
from GameClass import Game
from networkFolder.functionList import WorldEstimatingNetwork, DigitClassificationNetwork

class GreedyGame(Game):
    def __init__(self, truthMap, digit, navigator, robot):
        super().__init__(truthMap, digit, navigator, robot)
        self.uNet = WorldEstimatingNetwork()
        self.classNet = DigitClassificationNetwork()

    def tick(self):
        self.iterations += 1
        # Generate an action for the robot
        action = self.navigator.getAction(self.robot, self.exploredMap)
        # Move the robot
        self.robot.move(action)
        # Update the explored map based on robot position
        self._updateMap(self.robot, self.exploredMap, self.truthMap)

        # creates an estimate of what the world looks like after moving
        mask = np.zeros((28, 28))
        for x in range(0, 28):
            for y in range(0, 28):
                if self.exploredMap[x, y] != 128:
                    mask[x, y] = 1
        image = self.uNet.runNetwork(self.exploredMap, mask)
        # creates an estimate of what world we are in
        char = self.classNet.runNetwork(image)
        estimated_digit = char.argmax()

        # check if we are at the goal and correctly estimate the digit based on exploration
        if self.robot.getLoc() == self._goal:
            if estimated_digit == self.digit:
                self.score += 100
                return True
            else:
                self.score -= 1
                return False
        # if not, the robot has to do exploration
        else:
            self.score -= 1
            return False